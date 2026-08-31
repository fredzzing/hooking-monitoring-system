#!/usr/bin/env python3
"""verify_witmotion.py -- WitMotion WT901SDCL 实机验收脚本 (hook-trajectory)

板载一次性验收工具: MPU6050 替换为 WitMotion WT901SDCL (USB 串口, pyserial)
后, 用户拨开模块电源开关, 运行本脚本即可完成全部验收:

    1. 设备探测    : 列出候选串口 (/dev/ttyUSB* 优先, 排除 /dev/ttyACM0 = GPS)
    2. 打开 IMU    : IMUReader() 自动探测波特率 + 配置 (imu_reader 已处理)
    3. 采样率验证  : 采集 N 个样本, 统计实际采样率 (48-52 Hz) 与 p95 间隔抖动
                     (< 5 ms), 打印前 3 个样本原始值证明数据流动
    4. 静止校准验证: calibrate(duration) 后校验 mount_gravity 单位矢量、
                     静止重力幅值 ∈ [9.5, 10.1] m/s^2、gyro_bias 各轴 < 5 rad/s
    5. 融合链路冒烟(可选 --fusion): Attitude + EKF9 喂 50 个样本跑 predict,
                     静止位置漂移应 < 0.5 m (验证 attitude/ekf 与新 imu_reader
                     接口兼容)
    6. 汇总        : RESULT: N passed, M failed

退出码:
    0 = 全部通过
    1 = 至少一项失败
    2 = 无串口设备 / 无 IMU 数据 (请检查 USB 数据线 / 模块电源开关)

用法:
    python3 verify_witmotion.py [--samples N] [--calibrate-time S]
                                [--offline] [--fusion]

    --offline : 离线自测模式, 不依赖硬件。用 FakeSer 合成 50 Hz 帧流验证
                采样率统计逻辑, mock calibrate 返回固定 dict 验证校准判定
                逻辑, 坏数据/低采样率断言 FAIL 判定逻辑正确。
    --fusion  : 额外跑 attitude + ekf 融合链路冒烟 (50 样本)。

本脚本只读使用 imu_reader/attitude/ekf 模块, 不修改任何现有文件。
"""
import argparse
import math
import os
import sys
import time

# ---- local modules (与 recorder.py 相同的导入方式) -------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_reader  # noqa: E402
from imu_reader import PKT_ACCEL, PKT_GYRO  # noqa: E402

# ---------------------------------------------------------------- thresholds
G_STD = 9.80665
RATE_MIN = 48.0          # Hz
RATE_MAX = 52.0          # Hz
JITTER_P95_MAX = 0.005   # s (5 ms)
ACCEL_MAG_MIN = 9.5      # m/s^2 静止重力幅值下界
ACCEL_MAG_MAX = 10.1     # m/s^2 上界
GYRO_BIAS_MAX = 5.0      # rad/s (WitMotion 出厂校准过, 应远小于此)
FUSION_N = 50            # 融合冒烟样本数
FUSION_DRIFT_MAX = 0.5   # m, 静止位置漂移门限


# ---------------------------------------------------------------- helpers
def _median(vals):
    s = sorted(vals)
    return s[len(s) // 2]


def _p95(vals):
    """95th percentile of a sorted-able sequence (nearest-rank)."""
    s = sorted(vals)
    k = int(math.ceil(0.95 * len(s))) - 1
    return s[max(0, k)]


def _frame(ptype, d0, d1, d2, d3=0):
    """Build a checksum-valid 11-byte WitMotion frame (offline synthetic)."""
    body = bytearray([0x55, ptype])
    for d in (d0, d1, d2, d3):
        body.append(d & 0xFF)
        body.append((d >> 8) & 0xFF)
    body.append(sum(body) & 0xFF)
    return bytes(body)


class FakeSer(object):
    """Minimal pyserial stand-in (offline only, 同 test_imu_reader.py 模式):
    serves a looping chunk stream. Timing is VIRTUAL: each delivered chunk
    advances a shared virtual clock by `period` seconds (one sample = accel
    frame + gyro frame = 2 chunks), and `time.monotonic` is patched to
    return that virtual clock while offline tests run. This keeps the
    50 Hz stream deterministic on any OS (Windows time.monotonic is
    quantized to ~15.6 ms - unusable for 10 ms wall-clock pacing)."""

    def __init__(self, chunks=(), period=0.01):
        self._chunks = [bytes(c) for c in chunks]
        self._idx = 0
        self._period = period          # virtual stream seconds per chunk
        self.virt = [0.0]              # shared virtual clock (list = ref)
        self.in_waiting = 0
        self.timeout = 0.2
        self.written = []
        self.closed = False

    def read(self, n):
        if not self._chunks:
            return b""
        c = self._chunks[self._idx]
        self._idx = (self._idx + 1) % len(self._chunks)
        self.virt[0] += self._period   # advance virtual time by the chunk
        return c

    def write(self, data):
        self.written.append(bytes(data))

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


def _make_fake(stream, period):
    """Build an IMUReader without touching a real serial port (offline)."""
    r = imu_reader.IMUReader.__new__(imu_reader.IMUReader)
    r.ser = FakeSer(stream, period=period)
    r._buf = bytearray()
    r.cal = None
    r.port = "FAKE:ttyUSB0"
    r.baud = 115200
    return r


def _install_virtual_clock(virt):
    """Patch time.monotonic to return the shared virtual clock; returns the
    original function for restore. (imu_reader shares the same time module,
    so its t_mono stamps and stall guard see the virtual time too.)"""
    real = time.monotonic

    def fake():
        return virt[0]

    time.monotonic = fake
    imu_reader.time.monotonic = fake
    return real


def _q0_from_mount_gravity(mgv):
    """Pre-alignment quaternion: body gravity dir -> earth +z (recorder.py
    同款公式), 使融合冒烟从静止姿态开始。"""
    _dot = max(-1.0, min(1.0, mgv[2]))
    if _dot > 0.9999:
        return (1.0, 0.0, 0.0, 0.0)
    if _dot < -0.9999:
        return (0.0, 1.0, 0.0, 0.0)
    _n = math.hypot(mgv[1], mgv[0])
    _ax, _ay = mgv[1] / _n, -mgv[0] / _n
    _ang = math.acos(_dot)
    _s = math.sin(_ang / 2.0)
    return (math.cos(_ang / 2.0), _ax * _s, _ay * _s, 0.0)


# ---------------------------------------------------------------- verifier
class Verifier(object):
    """逐项 PASS/FAIL 记录与打印 (test_imu_reader.py 风格)。"""

    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, cond, detail=""):
        if cond:
            self.passed += 1
            print("PASS: %s" % name, flush=True)
        else:
            self.failed += 1
            print("FAIL: %s | %s" % (name, detail), flush=True)

    # ------------------------------------------------------------ 1) probe
    def probe_devices(self):
        """列出候选串口; 无设备返回 None (调用方转退出码 2)。"""
        devs = imu_reader._find_devices()
        print("设备探测: 候选串口 = %s (排除 /dev/ttyACM0 = u-blox GPS)"
              % (devs if devs else "[]"), flush=True)
        if not devs:
            print("FAIL: 设备探测 | 未检测到串口，请检查 USB 数据线", flush=True)
            self.failed += 1
            return None
        self.check("设备探测: 发现候选串口 %s" % devs, True)
        return devs

    # ---------------------------------------------------------- 3) sampling
    def sample_rate(self, imu, n):
        """采集 n 个样本: 实际采样率 48-52 Hz + p95 间隔抖动 < 5 ms + 前 3
        样本原始值 (证明数据流动)。"""
        print("采样率验证: 采集 %d 个样本 ..." % n, flush=True)
        samples = []
        t0 = time.monotonic()
        for _ in range(n):
            samples.append(imu.read_sample())
        t1 = time.monotonic()
        elapsed = t1 - t0
        rate = n / elapsed if elapsed > 0 else 0.0
        self.check("实际采样率 %.2f Hz ∈ [%.0f, %.0f]"
                   % (rate, RATE_MIN, RATE_MAX),
                   RATE_MIN <= rate <= RATE_MAX,
                   "%.2f Hz (n=%d, 总耗时 %.2f s)" % (rate, n, elapsed))
        ivs = [samples[i].t_mono - samples[i - 1].t_mono
               for i in range(1, n)]
        if len(ivs) >= 2:
            med = _median(ivs)
            p95 = _p95([abs(x - med) for x in ivs])
            self.check("p95 间隔抖动 < 5 ms", p95 < JITTER_P95_MAX,
                       "p95=%.3f ms (中位间隔 %.3f ms)"
                       % (p95 * 1000.0, med * 1000.0))
        else:
            self.check("p95 间隔抖动 < 5 ms", False, "样本不足")
        print("前 3 个样本 (ax/ay/az m/s^2 | gx/gy/gz rad/s):", flush=True)
        for s in samples[:3]:
            print("  t=%.4f ax=%.4f ay=%.4f az=%.4f gx=%.5f gy=%.5f gz=%.5f "
                  "sat=%d" % (s.t_mono, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                              s.sat_flag), flush=True)
        finite = all(math.isfinite(getattr(s, f))
                     for s in samples
                     for f in ("ax", "ay", "az", "gx", "gy", "gz"))
        self.check("数据流动: 样本值均为有限数", finite)
        return samples, rate

    # -------------------------------------------------------- 4) calibration
    def calibrate_checks(self, cal):
        """校验 calibrate() 返回 dict 的语义 (单位矢量 / 静止重力幅值 /
        gyro_bias 合理性 / 样本数)。"""
        print("校准结果:")
        print("  gyro_bias     = [%.5f, %.5f, %.5f] rad/s"
              % tuple(cal["gyro_bias"]), flush=True)
        print("  accel_bias    = [%.4f, %.4f, %.4f] m/s^2"
              % tuple(cal["accel_bias"]), flush=True)
        print("  mount_gravity = [%.4f, %.4f, %.4f] (unit)"
              % tuple(cal["mount_gravity"]), flush=True)
        print("  n_samples     = %d, duration_s = %.1f"
              % (cal["n_samples"], cal["duration_s"]), flush=True)
        mg = cal["mount_gravity"]
        norm = math.sqrt(mg[0] * mg[0] + mg[1] * mg[1] + mg[2] * mg[2])
        self.check("|mount_gravity| ≈ 1.0 (单位矢量)", abs(norm - 1.0) < 0.001,
                   "|mg|=%.5f" % norm)
        bias = cal["accel_bias"]
        mean_ax = G_STD * mg[0] + bias[0]
        mean_ay = G_STD * mg[1] + bias[1]
        mean_az = G_STD * mg[2] + bias[2]
        mag = math.sqrt(mean_ax * mean_ax + mean_ay * mean_ay + mean_az * mean_az)
        self.check("静止重力幅值 ∈ [%.1f, %.1f] m/s^2" % (ACCEL_MAG_MIN,
                                                          ACCEL_MAG_MAX),
                   ACCEL_MAG_MIN <= mag <= ACCEL_MAG_MAX,
                   "%.3f m/s^2" % mag)
        for name, b in zip(("x", "y", "z"), cal["gyro_bias"]):
            self.check("gyro_bias[%s] 幅值 < %.1f rad/s" % (name, GYRO_BIAS_MAX),
                       abs(b) < GYRO_BIAS_MAX, "%.5f rad/s" % b)
        self.check("n_samples >= 10", cal["n_samples"] >= 10,
                   "n=%d" % cal["n_samples"])

    # ------------------------------------------------------- 5) fusion smoke
    def fusion_smoke(self, imu, cal, n=FUSION_N):
        """Attitude + EKF9 冒烟: 喂 n 个 (bias 校正后的) 样本跑 predict,
        验证 attitude/ekf 与新 imu_reader 接口兼容, 静止位置漂移 < 0.5 m。"""
        import attitude as att_mod  # noqa: E402
        import ekf as ekf_mod       # noqa: E402

        q0 = _q0_from_mount_gravity(cal["mount_gravity"])
        att = att_mod.Attitude(q0=q0, init_bias=(0.0, 0.0, 0.0))
        ekf = ekf_mod.EKF9(p0_pos=(5.0, 5.0, 8.0))
        prev = None
        for _ in range(n):
            s = imu.read_sample()          # bias 已由 imu_reader 校正
            if prev is None:
                dt = 0.02
            else:
                dt = s.t_mono - prev
                if not (math.isfinite(dt) and 1e-4 < dt < 0.5):
                    dt = 0.02
            prev = s.t_mono
            att.update((s.gx, s.gy, s.gz), (s.ax, s.ay, s.az), dt)
            R = att.rotation_matrix()
            a_nav = (R[0][0] * s.ax + R[0][1] * s.ay + R[0][2] * s.az,
                     R[1][0] * s.ax + R[1][1] * s.ay + R[1][2] * s.az,
                     R[2][0] * s.ax + R[2][1] * s.ay + R[2][2] * s.az - G_STD)
            ekf.predict(dt, a_nav)
        st = ekf.state()
        drift = math.hypot(math.hypot(st[0], st[1]), st[2])
        print("融合链路冒烟: %d 样本, 最终位置 n=%.3f e=%.3f u=%.3f m, "
              "3D 漂移=%.3f m" % (n, st[0], st[1], st[2], drift), flush=True)
        self.check("融合链路冒烟: 静止位置漂移 < %.1f m" % FUSION_DRIFT_MAX,
                   drift < FUSION_DRIFT_MAX,
                   "3D 漂移=%.3f m" % drift)

    # ------------------------------------------------------------- summary
    def summary(self):
        print("")
        print("RESULT: %d passed, %d failed" % (self.passed, self.failed),
              flush=True)


# ------------------------------------------------------------- hardware run
def run_hardware(args):
    v = Verifier()
    print("=== verify_witmotion.py: WitMotion WT901SDCL 实机验收 ===",
          flush=True)

    # 1) 设备探测
    devs = v.probe_devices()
    if devs is None:
        v.summary()
        return 2

    # 2) 打开 IMU (自动探测波特率 + 配置)
    try:
        imu = imu_reader.IMUReader()
    except imu_reader.IMUError as exc:
        print("FAIL: 打开 IMU | %s" % exc, flush=True)
        v.failed += 1
        print("未检测到 IMU 数据，请检查 USB 数据线 / 模块电源开关",
              flush=True)
        v.summary()
        return 2
    v.check("打开 IMU: 自动探测波特率+配置", imu.ser is not None,
            "port=%s baud=%d" % (imu.port, imu.baud))

    try:
        # 3) 采样率验证
        try:
            v.sample_rate(imu, max(20, args.samples))
        except imu_reader.IMUError as exc:
            print("FAIL: 采样中断 | %s" % exc, flush=True)
            v.failed += 1
            v.summary()
            return 1

        # 4) 静止校准验证
        print("静止校准: calibrate(duration=%.1f s) -- 请保持模块静止 ..."
              % args.calibrate_time, flush=True)
        try:
            cal = imu.calibrate(args.calibrate_time)
        except (imu_reader.IMUError, RuntimeError) as exc:
            print("FAIL: 静止校准 | %s" % exc, flush=True)
            v.failed += 1
            v.summary()
            return 1
        v.calibrate_checks(cal)

        # 5) 融合链路冒烟 (可选)
        if args.fusion:
            try:
                v.fusion_smoke(imu, cal)
            except imu_reader.IMUError as exc:
                print("FAIL: 融合冒烟 | %s" % exc, flush=True)
                v.failed += 1
    finally:
        imu.close()

    # 6) 汇总
    v.summary()
    return 0 if v.failed == 0 else 1


# --------------------------------------------------------------- offline run
def run_offline(args):
    v = Verifier()
    print("=== verify_witmotion.py OFFLINE 自测 (无硬件, 合成 50 Hz 帧流) ===",
          flush=True)

    # 1) 设备探测 (模拟)
    v.check("设备探测 (模拟): 候选串口 ['/dev/ttyUSB0']", True)

    # 2) 打开 IMU (模拟): FakeSer 合成流, 虚拟时钟 0.01 s/chunk
    #    -> 样本间隔精确 0.02 s = 50 Hz (不依赖 OS 睡眠精度, 秒级出结果)
    stream = [_frame(PKT_ACCEL, 0, 0, 2048),    # +z 1 g (raw 2048 = 32768/16)
              _frame(PKT_GYRO, 0, 0, 0)]
    r = _make_fake(stream, period=0.01)
    v.check("打开 IMU (模拟): FakeSer 50 Hz 帧流", r.ser is not None,
            "port=%s baud=%d" % (r.port, r.baud))

    # 虚拟时钟: 样本 t_mono / 耗时统计全部走虚拟时间 (确定性, 无 OS 量化)
    real_monotonic = _install_virtual_clock(r.ser.virt)
    try:
        # 3) 采样率统计逻辑 (合成 50 Hz 流 -> 应全 PASS)
        v.sample_rate(r, max(20, args.samples))

        # 4) calibrate 路径逻辑 (mock 返回固定 dict -> 判定应全 PASS)
        good = {"gyro_bias": [0.002, -0.001, 0.001],
                "accel_bias": [0.01, -0.02, 0.03],
                "mount_gravity": [0.0, 0.0, 1.0],
                "n_samples": 500, "duration_s": 10.0}
        r.calibrate = lambda duration: dict(good)
        cal = r.calibrate(10.0)
        print("calibrate (mock) 返回固定 dict: %s" % cal, flush=True)
        v.calibrate_checks(cal)

        # FAIL 判定逻辑: 坏校准数据必须被拒绝 (以下 FAIL 为预期)
        print("--- 坏数据判定验证 (以下 FAIL 为预期) ---", flush=True)
        bad = {"gyro_bias": [6.5, 0.0, 0.0],
               "accel_bias": [0.0, 0.0, 0.0],
               "mount_gravity": [0.70710678, 0.0, 0.70710678],
               "n_samples": 3, "duration_s": 10.0}
        sub = Verifier()
        sub.calibrate_checks(bad)
        v.check("FAIL 判定逻辑: 坏校准数据被拒绝 (gyro_bias 超限/n<10)",
                sub.failed > 0, "%d 项被判 FAIL" % sub.failed)

        # FAIL 判定逻辑: 低采样率必须被拒绝 (0.05 s/chunk -> 样本 0.1 s ≈ 10 Hz)
        print("--- 低采样率判定验证 (以下 FAIL 为预期) ---", flush=True)
        slow = _make_fake(stream, period=0.05)
        slow.ser.virt = r.ser.virt        # 共用同一虚拟时钟
        sub2 = Verifier()
        sub2.sample_rate(slow, 20)
        v.check("FAIL 判定逻辑: 低采样率流被拒绝", sub2.failed > 0,
                "%d 项被判 FAIL" % sub2.failed)

        # 5) 融合链路冒烟 (合成流静止 -> a_nav ≈ 0 -> 漂移 < 0.5 m)
        if args.fusion:
            r.cal = good                      # read_sample 应用 bias 校正
            v.fusion_smoke(r, good)
    finally:
        time.monotonic = real_monotonic       # 恢复真实时钟
        imu_reader.time.monotonic = real_monotonic

    # 6) 汇总
    v.summary()
    return 0 if v.failed == 0 else 1


# --------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="WitMotion WT901SDCL 实机验收: 设备探测 -> 打开 IMU -> "
                    "采样率验证 -> 静止校准验证 -> (可选) 融合冒烟。"
                    "退出码 0=全过 / 1=任一失败 / 2=无设备或数据")
    ap.add_argument("--samples", type=int, default=200,
                    help="采样率验证的样本数 (默认 200, 约 4 s)")
    ap.add_argument("--calibrate-time", type=float, default=10.0,
                    help="静止校准时长秒 (默认 10.0)")
    ap.add_argument("--offline", action="store_true",
                    help="离线自测模式: 合成 50 Hz 帧流, 不依赖硬件")
    ap.add_argument("--fusion", action="store_true",
                    help="额外跑 attitude + ekf 融合链路冒烟 (50 样本)")
    args = ap.parse_args(argv)
    if args.offline:
        return run_offline(args)
    return run_hardware(args)


if __name__ == "__main__":
    sys.exit(main())

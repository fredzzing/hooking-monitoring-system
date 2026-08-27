#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""radio_recorder.py -- 吊物轨迹录制守护进程 (HOOK 协议 over E90-DTU LoRa 网关).

角色: TCP **客户端** 连接无线电网关 (默认 192.168.0.127:8886, --host/--port 可配置),
      接收 HOOK 命令并驱动 /root/trajectory/recorder.py 录制轨迹。
      由 systemd 单元 radio-recorder.service 常驻 (Restart=always, RestartSec=5)。

HOOK 协议 (与 PC 端 hang_ctrl.py 严格一致):
  PC -> Pi : HOOK REC_START [seconds] | HOOK REC_STOP | HOOK STATUS | HOOK TEST
  Pi -> PC : HOOK ACK | HOOK REC_STARTED | HOOK REC_DONE <stem>
           | HOOK REC_FAILED | HOOK REC_BUSY | HOOK STATUS <idle|recording|busy> [stem]

  REC_START 可选携带 <seconds> (如 'HOOK REC_START 30'): 守护按该时长录制;
  缺省或解析失败时用 DEFAULT_DURATION (1800.0s)。

关键纪律 (计划 Task 2):
  - 单实例锁 (PID 文件): 第二个守护实例直接退出; 录制中重复 REC_START -> REC_BUSY
  - REC_STOP -> SIGTERM (绝不 SIGKILL) -> 等子进程退出 + meta.json 生成 -> REC_DONE <stem>
  - 子进程退出码: 0 -> REC_DONE; 2 (无 GPS fix) -> REC_FAILED; 其他 -> REC_FAILED
    (非 0 绝不发 REC_DONE)
  - 启动时孤儿清理: 终止遗留 recorder.py 进程 + 标记无 meta.json 的 csv
  - 断线自动重连, 指数退避 (2s 起, 上限 30s); 仅 stdlib, 自包含

注意: 本文件只 spawn recorder.py, 绝不修改 recorder.py 本身。
"""

import argparse
import os
import re
import select
import signal
import socket
import subprocess
import sys
import time

DEFAULT_HOST = "192.168.0.127"
DEFAULT_PORT = 8886
DEFAULT_DURATION = 1800.0
DEFAULT_PIDFILE = "/run/radio_recorder.pid"
RECORDER_DIR = "/root/trajectory"
RECORDER_SCRIPT = "recorder.py"
BACKOFF_INIT = 2.0
BACKOFF_MAX = 30.0
POLL_INTERVAL = 1.0        # select 轮询粒度 (s)
STOP_GRACE = 60.0          # SIGTERM 后等子进程退出的最长时间 (s)
META_WAIT = 15.0           # 子进程退出后等 meta.json 出现的最长时间 (s)
ORPHAN_GRACE = 8.0         # 孤儿清理时等进程退出的时间 (s)


def log(msg):
    """时间戳日志 -> stderr (systemd journal 捕获)。"""
    print("[%s] %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg),
          file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# 纯函数: 消息解析/构造 (selftest 覆盖)
# ---------------------------------------------------------------------------

def parse_hook(line):
    """解析一行命令文本 -> (命令, 参数列表) 或 None。

    只认 'HOOK <CMD>' 前缀 (与船控 CMD 前缀隔离), 大小写敏感, 容忍多余空白。
    """
    if not isinstance(line, str):
        return None
    line = line.strip()
    if not line.startswith("HOOK"):
        return None
    parts = line.split()
    if len(parts) < 2:
        return None
    if parts[0] != "HOOK":
        return None
    return parts[1], parts[2:]


def parse_duration_arg(args):
    """HOOK REC_START 时长参数: args[0] 为正数 -> float(秒); 缺省/解析失败/非正数 -> None。

    None 表示使用守护默认 DEFAULT_DURATION (协议: HOOK REC_START [<seconds>])。
    """
    if not args:
        return None
    try:
        val = float(args[0])
    except (TypeError, ValueError):
        return None
    if val > 0:
        return val
    return None


def feed_lines(data, buf):
    """行缓冲: 追加字节 -> (完整行列表, 剩余缓冲)。"""
    buf += data
    lines = []
    while b"\n" in buf:
        raw, buf = buf.split(b"\n", 1)
        lines.append(raw)
    return lines, buf


def reply_ack():
    return "HOOK ACK"


def reply_rec_started():
    return "HOOK REC_STARTED"


def reply_rec_done(stem):
    return "HOOK REC_DONE %s" % stem


def reply_rec_failed():
    return "HOOK REC_FAILED"


def reply_rec_busy():
    return "HOOK REC_BUSY"


def reply_status(state, stem=None):
    msg = "HOOK STATUS %s" % state
    if stem:
        msg += " %s" % stem
    return msg


def exit_code_to_reply(code, stem):
    """子进程退出码 -> 回复消息。

    纪律: 仅退出码 0 **且** 拿到 stem 才发 REC_DONE; 其余一律 REC_FAILED
    (退出码 2 = 无 GPS fix, 绝不触发 REC_DONE)。
    """
    if code == 0 and stem:
        return reply_rec_done(stem)
    return reply_rec_failed()


def extract_stem_from_output(out_bytes):
    """从 recorder.py 标准输出 (末尾 'meta=<path>' 行) 提取 stem (YYYYMMDD-HHMMSS)。"""
    m = re.search(rb"meta=(\S*?)([^/\\]+)\.meta\.json", out_bytes)
    if m:
        return m.group(2).decode("ascii", errors="replace")
    return None


def stem_from_meta_path(path):
    base = os.path.basename(path)
    if base.endswith(".meta.json"):
        return base[: -len(".meta.json")]
    return None


# ---------------------------------------------------------------------------
# 单实例锁 (PID 文件)
# ---------------------------------------------------------------------------

def _pid_alive(pid):
    return pid > 0 and os.path.isdir("/proc") and os.path.exists("/proc/%d" % pid)


def _is_radio_recorder(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            data = fh.read().replace(b"\0", b" ").decode(errors="replace")
        return "radio_recorder.py" in data
    except OSError:
        return False


def acquire_lock(pidfile):
    """获取单实例锁。返回 (True, None) 或 (False, 占用者 PID)。"""
    if os.path.exists(pidfile):
        try:
            with open(pidfile, "r", encoding="ascii") as fh:
                old = int(fh.read().strip())
        except (OSError, ValueError):
            old = 0
        if old and _pid_alive(old) and _is_radio_recorder(old):
            return False, old
        log("stale pidfile (pid=%s) removed" % old)
    try:
        with open(pidfile, "w", encoding="ascii") as fh:
            fh.write("%d\n" % os.getpid())
    except OSError as exc:
        log("WARN: cannot write pidfile %s: %s (continuing unlocked)" % (pidfile, exc))
        return True, None
    return True, None


def release_lock(pidfile):
    try:
        with open(pidfile, "r", encoding="ascii") as fh:
            cur = int(fh.read().strip())
        if cur == os.getpid():
            os.remove(pidfile)
    except (OSError, ValueError):
        pass


# ---------------------------------------------------------------------------
# 孤儿清理
# ---------------------------------------------------------------------------

def find_orphan_recorders():
    """扫描 /proc 找遗留的 recorder.py 进程 (排除本守护自身)。

    匹配 cmdline 中 'recorder.py' 且不含 'radio_recorder.py'。
    """
    if not os.path.isdir("/proc"):
        return []
    pids = []
    me = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open("/proc/%s/cmdline" % entry, "rb") as fh:
                cmdline = fh.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "radio_recorder.py" in cmdline:
            continue
        if re.search(r"(^|[\s/])recorder\.py(\s|$)", cmdline):
            pids.append(pid)
    return pids


def cleanup_orphans():
    """启动时孤儿清理: SIGTERM 遗留 recorder.py; 标记无 meta.json 的 csv。"""
    pids = find_orphan_recorders()
    if pids:
        for pid in pids:
            log("orphan cleanup: SIGTERM legacy recorder.py pid=%d" % pid)
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + ORPHAN_GRACE
        while time.monotonic() < deadline and any(_pid_alive(p) for p in pids):
            time.sleep(0.25)
        for pid in pids:
            if _pid_alive(pid):
                log("orphan cleanup WARN: pid=%d still alive after SIGTERM "
                    "(left running; no SIGKILL by policy)" % pid)
    runs = os.path.join(RECORDER_DIR, "runs")
    if os.path.isdir(runs):
        try:
            names = sorted(os.listdir(runs))
        except OSError:
            names = []
        for fn in names:
            if not fn.endswith(".csv"):
                continue
            meta = os.path.join(runs, fn[: -len(".csv")] + ".meta.json")
            if not os.path.exists(meta):
                marker = os.path.join(runs, fn + ".incomplete")
                try:
                    with open(marker, "a"):
                        pass
                    log("orphan cleanup: csv without meta.json marked: %s" % fn)
                except OSError as exc:
                    log("orphan cleanup WARN: cannot mark %s: %s" % (fn, exc))
    else:
        log("orphan cleanup: runs dir %s missing" % runs)


# ---------------------------------------------------------------------------
# 守护主类
# ---------------------------------------------------------------------------

class RecorderDaemon(object):
    def __init__(self, host, port, duration):
        self.host = host
        self.port = port
        self.duration = duration
        self.sock = None
        self.running = True
        self.child = None          # subprocess.Popen | None
        self.child_start_wall = 0.0
        self.child_stdout = b""
        self.child_stderr = b""
        self.stem = None           # 当前录制 stem (child 存活期间为 None, 退出后填)
        self.state = "idle"        # idle | recording | busy

    # -- 连接管理 -----------------------------------------------------------
    def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while self.running and time.monotonic() < end:
            time.sleep(0.1)

    def connect(self):
        """连网关, 失败指数退避 (2s 起, 上限 30s)。成功返回 True。"""
        delay = BACKOFF_INIT
        while self.running:
            try:
                self.sock = socket.create_connection((self.host, self.port),
                                                     timeout=5.0)
                log("connected to gateway %s:%d" % (self.host, self.port))
                return True
            except OSError as exc:
                log("connect %s:%d failed: %s (retry in %.1fs)"
                    % (self.host, self.port, exc, delay))
                self._sleep(delay)
                delay = min(delay * 2.0, BACKOFF_MAX)
        return False

    def close_sock(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    # -- 发送 ---------------------------------------------------------------
    def send(self, msg):
        if self.sock is None:
            log("TX skipped (no connection): %s" % msg)
            return
        try:
            self.sock.sendall((msg + "\n").encode("utf-8"))
            log("TX: %s" % msg)
        except OSError as exc:
            log("TX error: %s" % exc)

    # -- 命令分发 -----------------------------------------------------------
    def handle_command(self, cmd, args):
        if cmd == "TEST":
            self.send(reply_ack())
        elif cmd == "STATUS":
            self.send(reply_status(self.state, self.stem))
        elif cmd == "REC_START":
            self._on_rec_start(args)
        elif cmd == "REC_STOP":
            self._on_rec_stop()
        else:
            log("unknown HOOK command ignored: %s %s" % (cmd, args))

    def _on_rec_start(self, args=None):
        if self.child is not None:
            # 已录制/回收中 -> 幂等拒绝 (半双工命令可能重复到达)
            self.send(reply_rec_busy())
            log("REC_START while %s -> REC_BUSY" % self.state)
            return
        dur = parse_duration_arg(args)
        eff = dur if dur is not None else self.duration
        log("REC_START: duration arg=%r -> effective %.1fs (daemon default %.1fs)"
            % (args, eff, self.duration))
        try:
            self.child = subprocess.Popen(
                ["python3", RECORDER_SCRIPT,
                 "--duration", str(eff),
                 "--out", "runs/"],
                cwd=RECORDER_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
        except OSError as exc:
            log("REC_START: failed to spawn recorder.py: %s" % exc)
            self.child = None
            self.state = "idle"
            self.send(reply_rec_failed())
            return
        self.state = "recording"
        self.stem = None
        self.child_start_wall = time.time()
        self.child_stdout = b""
        self.child_stderr = b""
        log("REC_START: spawned recorder.py pid=%d duration=%.1fs"
            % (self.child.pid, eff))
        self.send(reply_rec_started())

    def _on_rec_stop(self):
        if self.child is not None and self.child.poll() is None:
            log("REC_STOP: SIGTERM recorder.py pid=%d" % self.child.pid)
            try:
                os.kill(self.child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        else:
            log("REC_STOP: nothing recording (state=%s)" % self.state)
        # 立即确认收到 (半双工); 最终结果由 REC_DONE/REC_FAILED 表达
        self.send(reply_ack())

    # -- 子进程回收 ---------------------------------------------------------
    def _poll_child(self):
        if self.child is None:
            return
        code = self.child.poll()
        if code is None:
            return  # 仍在运行
        try:
            out, err = self.child.communicate(timeout=5)
        except Exception:  # noqa: BLE001
            out, err = self.child_stdout, self.child_stderr
        for line in err.decode(errors="replace").splitlines():
            if line.strip():
                log("recorder[stderr]: %s" % line)
        self.child_stdout = out
        self.child_stderr = err
        self.child = None

        stem = extract_stem_from_output(out)
        if code == 0:
            # 纪律: 等 meta.json 确认后才发 REC_DONE
            meta_path = self._wait_for_meta(stem)
            if meta_path is not None:
                stem = stem_from_meta_path(meta_path) or stem
                self.stem = stem
                self.state = "idle"
                log("recorder exited 0, meta confirmed: %s -> REC_DONE %s"
                    % (meta_path, stem))
                self.send(reply_rec_done(stem))
            else:
                self.state = "idle"
                log("recorder exited 0 but no meta.json within %.0fs -> REC_FAILED"
                    % META_WAIT)
                self.send(reply_rec_failed())
        else:
            self.state = "idle"
            log("recorder exited code=%s -> REC_FAILED (no REC_DONE)" % code)
            self.send(reply_rec_failed())

    def _wait_for_meta(self, known_stem):
        """等 meta.json 出现: 已知 stem 直接查文件, 否则扫 runs/ 找最新 meta。"""
        runs = os.path.join(RECORDER_DIR, "runs")
        deadline = time.monotonic() + META_WAIT
        while self.running and time.monotonic() < deadline:
            if known_stem:
                path = os.path.join(runs, known_stem + ".meta.json")
                if os.path.exists(path):
                    return path
            else:
                best = self._newest_meta()
                if best is not None:
                    return best
            time.sleep(0.25)
        if known_stem:
            path = os.path.join(runs, known_stem + ".meta.json")
            if os.path.exists(path):
                return path
        return None

    def _newest_meta(self):
        """runs/ 中 mtime >= 子进程启动时刻-5s 的最新 meta.json 路径。"""
        runs = os.path.join(RECORDER_DIR, "runs")
        if not os.path.isdir(runs):
            return None
        since = self.child_start_wall - 5.0
        best = None
        best_mtime = 0.0
        try:
            names = os.listdir(runs)
        except OSError:
            return None
        for fn in names:
            if not fn.endswith(".meta.json"):
                continue
            path = os.path.join(runs, fn)
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if mtime >= since and mtime > best_mtime:
                best, best_mtime = path, mtime
        return best

    # -- 服务循环 -----------------------------------------------------------
    def _handle_raw_line(self, raw):
        line = raw.decode(errors="replace").strip()
        if not line:
            return
        if not line.startswith("HOOK"):
            log("RX ignored (non-HOOK): %r" % line)
            return
        parsed = parse_hook(line)
        if parsed is None:
            log("RX unparsable: %r" % line)
            return
        cmd, args = parsed
        log("RX: %s %s" % (cmd, " ".join(args) if args else ""))
        self.handle_command(cmd, args)

    def _serve_connection(self):
        buf = b""
        while self.running:
            self._poll_child()
            try:
                readable, _, _ = select.select([self.sock], [], [],
                                               POLL_INTERVAL)
            except (OSError, ValueError):
                return
            if not readable:
                continue
            try:
                data = self.sock.recv(4096)
            except OSError as exc:
                log("recv error: %s" % exc)
                return
            if not data:
                log("gateway closed connection")
                return
            lines, buf = feed_lines(data, buf)
            for raw in lines:
                self._handle_raw_line(raw)

    def serve_forever(self):
        while self.running:
            if self.connect():
                self._serve_connection()
                self.close_sock()
                if self.running:
                    log("connection lost; will reconnect")
        self.shutdown_child()

    def shutdown_child(self):
        """守护退出: SIGTERM 子进程并短暂等待 (绝不 SIGKILL)。"""
        if self.child is not None and self.child.poll() is None:
            log("shutdown: SIGTERM recorder.py pid=%d" % self.child.pid)
            try:
                os.kill(self.child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline \
                    and self.child.poll() is None:
                time.sleep(0.25)
            if self.child.poll() is not None:
                try:
                    self.child.communicate(timeout=5)
                except Exception:  # noqa: BLE001
                    pass
            else:
                log("shutdown WARN: recorder.py still running (no SIGKILL)")


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------

def run_selftest():
    failures = []

    def check(cond, name):
        if cond:
            print("PASS: %s" % name)
        else:
            print("FAIL: %s" % name)
            failures.append(name)

    print("== message parsing ==")
    check(parse_hook("HOOK REC_START") == ("REC_START", []),
          "parse HOOK REC_START")
    check(parse_hook("HOOK REC_STOP\r\n") == ("REC_STOP", []),
          "parse HOOK REC_STOP (CRLF)")
    check(parse_hook("HOOK STATUS") == ("STATUS", []),
          "parse HOOK STATUS")
    check(parse_hook("HOOK TEST") == ("TEST", []),
          "parse HOOK TEST")
    check(parse_hook("  HOOK REC_START  ") == ("REC_START", []),
          "parse HOOK REC_START (whitespace)")
    check(parse_hook("HOOK REC_START extra") == ("REC_START", ["extra"]),
          "parse HOOK REC_START with extra arg")
    check(parse_hook("CMD REC_START") is None,
          "reject CMD prefix (boat-control isolation)")
    check(parse_hook("HOOK") is None, "reject bare HOOK")
    check(parse_hook("") is None, "reject empty line")
    check(parse_hook("hook rec_start") is None, "reject lowercase (case-sensitive)")
    check(parse_hook("garbage") is None, "reject non-HOOK garbage")

    print("== REC_START duration parsing (HOOK REC_START [<seconds>]) ==")
    check(parse_duration_arg([]) is None, "no args -> default (1800)")
    check(parse_duration_arg(["30"]) == 30.0, "arg 30 -> 30.0s")
    check(parse_duration_arg(["45.5"]) == 45.5, "arg 45.5 -> 45.5s")
    check(parse_duration_arg(["30", "junk"]) == 30.0, "arg '30 junk' -> 30.0s")
    check(parse_duration_arg(["abc"]) is None, "arg abc -> default")
    check(parse_duration_arg(["-5"]) is None, "arg -5 -> default")
    check(parse_duration_arg(["0"]) is None, "arg 0 -> default")
    check(parse_duration_arg(None) is None, "args None -> default")

    print("== message construction ==")
    check(reply_ack() == "HOOK ACK", "construct HOOK ACK")
    check(reply_rec_started() == "HOOK REC_STARTED", "construct HOOK REC_STARTED")
    check(reply_rec_done("20260825-073000") == "HOOK REC_DONE 20260825-073000",
          "construct HOOK REC_DONE <stem>")
    check(reply_rec_failed() == "HOOK REC_FAILED", "construct HOOK REC_FAILED")
    check(reply_rec_busy() == "HOOK REC_BUSY", "construct HOOK REC_BUSY")
    check(reply_status("idle") == "HOOK STATUS idle",
          "construct HOOK STATUS idle")
    check(reply_status("recording", "20260825-073000")
          == "HOOK STATUS recording 20260825-073000",
          "construct HOOK STATUS recording <stem>")
    check(reply_status("busy") == "HOOK STATUS busy",
          "construct HOOK STATUS busy")

    print("== exit-code -> reply mapping (REC_FAILED branch) ==")
    check(exit_code_to_reply(0, "S") == "HOOK REC_DONE S",
          "exit 0 + stem -> REC_DONE")
    check(exit_code_to_reply(0, None) == "HOOK REC_FAILED",
          "exit 0 without stem -> REC_FAILED (meta unconfirmed, no REC_DONE)")
    for code in (1, 2, 130, 137, -9):
        reply = exit_code_to_reply(code, "S")
        check(reply == "HOOK REC_FAILED",
              "exit %s -> REC_FAILED (never REC_DONE)" % code)
        check(not reply.startswith("HOOK REC_DONE"),
              "exit %s reply never starts with REC_DONE" % code)
    check(exit_code_to_reply(2, "S") == "HOOK REC_FAILED",
          "exit 2 (no GPS fix) -> REC_FAILED")

    print("== line buffering ==")
    lines, buf = feed_lines(b"HOOK TEST\nHOOK STAT", b"")
    check(lines == [b"HOOK TEST"] and buf == b"HOOK STAT",
          "partial line buffered")
    lines, buf = feed_lines(b"US\n", buf)
    check(lines == [b"HOOK STATUS"] and buf == b"",
          "partial line completed across recv() calls")

    print("== stem extraction ==")
    check(extract_stem_from_output(b"meta=runs/20260825-073000.meta.json\n")
          == "20260825-073000",
          "extract stem from relative meta= line")
    check(extract_stem_from_output(
        b"meta=/root/trajectory/runs/20260825-073001.meta.json\n")
        == "20260825-073001",
        "extract stem from absolute meta= line")
    check(extract_stem_from_output(b"no meta line here") is None,
          "no meta line -> None")

    print("== summary ==")
    if failures:
        print("SELFTEST FAILED: %d failure(s): %s" % (len(failures), failures))
        return 1
    print("SELFTEST OK: all checks passed")
    return 0


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="HOOK 协议守护: TCP 客户端连 LoRa 网关, 驱动 recorder.py")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="无线电网关 IP (default %s)" % DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help="无线电网关 TCP 端口 (default %d)" % DEFAULT_PORT)
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                    help="单次录制时长 s, 传给 recorder.py (default %s)"
                         % DEFAULT_DURATION)
    ap.add_argument("--pidfile", default=DEFAULT_PIDFILE,
                    help="单实例锁 PID 文件 (default %s)" % DEFAULT_PIDFILE)
    ap.add_argument("--selftest", action="store_true",
                    help="消息解析/构造 + 退出码映射自检, 退出码 0 = 通过")
    args = ap.parse_args(argv)

    if args.selftest:
        return run_selftest()

    log("radio_recorder.py starting: gateway=%s:%d duration=%s pidfile=%s"
        % (args.host, args.port, args.duration, args.pidfile))

    ok, holder = acquire_lock(args.pidfile)
    if not ok:
        print("ERROR: another instance is running (pid=%s, pidfile=%s)"
              % (holder, args.pidfile), file=sys.stderr, flush=True)
        return 1

    daemon = RecorderDaemon(args.host, args.port, args.duration)

    def _on_signal(signum, frame):  # noqa: ARG001
        log("received signal %d - shutting down" % signum)
        daemon.running = False
        daemon.close_sock()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    log("orphan cleanup at startup ...")
    cleanup_orphans()

    try:
        daemon.serve_forever()
    finally:
        release_lock(args.pidfile)
        log("exited")
    return 0


if __name__ == "__main__":
    sys.exit(main())

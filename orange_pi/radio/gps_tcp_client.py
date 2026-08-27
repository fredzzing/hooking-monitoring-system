"""GPS + TCP 定位比对客户端。

功能：
1. 从串口读取 NMEA GPS 数据并解析本机经纬度；
2. 周期性地将本机位置通过 TCP 发送到服务器；
3. 接收服务器返回的远程机坐标，计算与本机的相对位置（距离、方位角、东西/南北偏移）。

发送节奏（RADIO_NETWORK_SPEC §4）：
- GPS 时间可用且新鲜（≤2s）时，POS 在 GPS 秒边界 + 0.35s 相位发射（1Hz，与船 USV 错峰）；
- GPS 时间不可用/过期时，退化为墙钟 1Hz + ±50ms 随机抖动。

依赖：pip install pyserial
"""

import argparse
import math
import random
import socket
import threading
import time

import serial
from typing import Optional

EARTH_RADIUS = 6371000.0  # 地球平均半径（米）

# ---- RADIO_NETWORK_SPEC §4 发送时序参数（POS：GPS 秒边界 + 0.35s，1Hz）----
PHASE_OFFSET = 0.35      # POS 相位偏移（秒）：GPS 秒边界 + 0.35s，与船 USV（0.0）错峰
MAX_ALIGN_SLEEP = 0.9    # 对齐 sleep 单次上界（秒），留余量；越界/过期为负立即发送不累积
FIX_STALE_LIMIT = 2.0    # fix 新鲜度窗口（秒）；GPS 时间超期视为不可用
WALL_JITTER = 0.05       # 无 GPS 秒退化：墙钟 ±50ms 随机抖动


# ---------------- NMEA 解析 ----------------

def _nmea_to_decimal(raw: str, hemisphere: str):
    """将 NMEA 的 ddmm.mmmm 格式转换为十进制度数。"""
    if not raw:
        return None
    try:
        dot = raw.index(".")
        degrees = float(raw[: dot - 2])
        minutes = float(raw[dot - 2:])
        value = degrees + minutes / 60.0
        if hemisphere in ("S", "W"):
            value = -value
        return value
    except (ValueError, IndexError):
        return None


def _nmea_utc_seconds(raw: str) -> Optional[float]:
    """将 NMEA 的 hhmmss.ss 时间解析为当日 UTC 秒数（0~86400）；无效返回 None。"""
    if not raw:
        return None
    try:
        hh = int(raw[0:2])
        mm = int(raw[2:4])
        ss = float(raw[4:])
    except (ValueError, IndexError):
        return None
    if hh > 23 or mm > 59 or ss >= 60.0:
        return None  # 越界或闰秒 60 视为无效
    return hh * 3600.0 + mm * 60.0 + ss


def parse_nmea(sentence: str) -> Optional[dict]:
    """解析 GGA / RMC 语句，返回 {'lat','lon','alt','utc_seconds'} 或 None。

    utc_seconds 为当日 UTC 秒数（GGA/RMC field[1] hhmmss.ss）；无时间字段时为 None。
    """
    sentence = sentence.strip()
    if not sentence.startswith("$"):
        return None
    # 校验和检查（无校验和则跳过检查）
    if "*" in sentence:
        body, checksum = sentence[1:].split("*", 1)
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        try:
            if calc != int(checksum[:2], 16):
                return None
        except ValueError:
            return None
        fields = body.split(",")
    else:
        fields = sentence[1:].split(",")

    msg_type = fields[0][2:] if len(fields[0]) >= 2 else fields[0]

    if msg_type == "GGA" and len(fields) >= 10:
        lat = _nmea_to_decimal(fields[2], fields[3])
        lon = _nmea_to_decimal(fields[4], fields[5])
        try:
            quality = int(fields[6])
        except ValueError:
            quality = 0
        if quality == 0 or lat is None or lon is None:
            return None  # 无有效定位
        try:
            alt = float(fields[9])
        except ValueError:
            alt = 0.0
        utc_seconds = _nmea_utc_seconds(fields[1])
        return {"lat": lat, "lon": lon, "alt": alt, "utc_seconds": utc_seconds}

    if msg_type == "RMC" and len(fields) >= 7:
        if fields[2] != "A":  # A=有效，V=无效
            return None
        lat = _nmea_to_decimal(fields[3], fields[4])
        lon = _nmea_to_decimal(fields[5], fields[6])
        if lat is None or lon is None:
            return None
        utc_seconds = _nmea_utc_seconds(fields[1])
        return {"lat": lat, "lon": lon, "alt": 0.0, "utc_seconds": utc_seconds}

    return None


# ---------------- 相对位置计算 ----------------

def relative_position(lat1: float, lon1: float, lat2: float, lon2: float) -> dict:
    """以本机为原点，计算远程机的距离、方位角及东西/南北偏移（米）。"""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    distance = 2 * EARTH_RADIUS * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    bearing = math.degrees(math.atan2(
        math.sin(dlambda) * math.cos(phi2),
        math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda),
    )) % 360

    east = EARTH_RADIUS * dlambda * math.cos((phi1 + phi2) / 2)
    north = EARTH_RADIUS * dphi

    return {"distance": distance, "bearing": bearing, "east": east, "north": north}


# ---------------- 全局状态 ----------------

local_pos = None    # type: Optional[dict]
remote_pos = None   # type: Optional[dict]
pos_lock = threading.Lock()
last_data_time = None   # 最后一次从串口收到任何数据的时间
last_fix_time = None    # 最后一次解析出有效定位的时间


# ---------------- 线程：串口读取 GPS ----------------

def gps_reader(ser: serial.Serial, debug: bool = False) -> None:
    global local_pos, last_data_time, last_fix_time
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print(f"\n[串口错误] {e}")
            return
        if not raw:
            continue
        last_data_time = time.time()
        line = raw.decode("ascii", errors="ignore").strip()
        if debug and line:
            print(f"[RAW] {line}")
        pos = parse_nmea(line)
        if pos:
            with pos_lock:
                local_pos = pos
            last_fix_time = time.time()
            print(f"[GPS] 本机: lat={pos['lat']:.6f}, lon={pos['lon']:.6f}, "
                  f"alt={pos['alt']:.1f}m", flush=True)


# ---------------- 线程：发送本机位置 ----------------

def position_sender(sock: socket.socket, interval: float, stop: threading.Event) -> None:
    """周期发送本机位置（RADIO_NETWORK_SPEC §4）。

    GPS 时间可用且新鲜（last_fix_time 距今 ≤2s 且 pos 含 utc_seconds）时：
    在 GPS 秒边界 + 0.35s 相位发送（1Hz）；GPS 时间不可用/过期时退化为
    墙钟 1Hz + ±50ms 随机抖动。POS 报文格式不变（三端契约）。
    """
    last_sent_utc = None  # 上次发送对应的目标 GPS 秒（避免同一秒重复发送）

    while not stop.is_set():
        with pos_lock:
            pos = local_pos
            fix_at = last_fix_time
        if pos is None:
            # 无有效定位：不发送，等一个周期再试（保持原语义）
            stop.wait(interval)
            continue

        entered = time.time()
        utc = pos.get("utc_seconds")
        gps_ok = (utc is not None and fix_at is not None
                  and entered - fix_at <= FIX_STALE_LIMIT)

        if gps_ok:
            # ---- GPS 秒边界对齐路径（秒边界 + 0.35s 相位）----
            # 下一次发送时刻（GPS 秒）：ceil(utc / interval) * interval + PHASE_OFFSET
            next_utc = math.ceil(utc / interval) * interval + PHASE_OFFSET
            if last_sent_utc is not None and next_utc <= last_sent_utc:
                next_utc += interval  # 该目标已错过/已发，推进到下一个周期边界
            # 映射到墙钟：fix 时刻墙钟与 GPS 秒的差 = fix_at - utc
            target_wall = fix_at + (next_utc - utc)
            delta = target_wall - time.time()
            # sleep 到 target_wall；单次 sleep 上界 0.9s 分步逼近，越界/过期为负立即发送不累积
            while delta > 0 and not stop.is_set():
                stop.wait(min(delta, MAX_ALIGN_SLEEP))
                delta = target_wall - time.time()
            if stop.is_set():
                return
            waited = time.time() - entered
            msg = f"POS {pos['lat']:.7f} {pos['lon']:.7f} {pos['alt']:.1f}\n"
            try:
                sock.sendall(msg.encode("ascii"))
            except (ConnectionError, OSError):
                print("\n[TCP] 发送失败，连接已断开")
                stop.set()
                return
            last_sent_utc = next_utc
            print(f"[POS] @{next_utc:.3f}s (phase=GPS+{PHASE_OFFSET:.2f}s, "
                  f"wait={waited:.3f}s)", flush=True)
        else:
            # ---- 无 GPS 秒退化路径：墙钟 1Hz + ±50ms 随机抖动 ----
            jitter = random.uniform(-WALL_JITTER, WALL_JITTER)
            if stop.wait(max(interval + jitter, 0.0)):
                return
            msg = f"POS {pos['lat']:.7f} {pos['lon']:.7f} {pos['alt']:.1f}\n"
            try:
                sock.sendall(msg.encode("ascii"))
            except (ConnectionError, OSError):
                print("\n[TCP] 发送失败，连接已断开")
                stop.set()
                return
            print(f"[POS] (wall{jitter * 1000:+.1f}ms)", flush=True)


# ---------------- 线程：接收远程机位置 ----------------

def remote_receiver(sock: socket.socket, stop: threading.Event) -> None:
    global remote_pos
    buffer = b""
    while not stop.is_set():
        try:
            data = sock.recv(4096)
        except (ConnectionError, OSError):
            break
        if not data:
            print("\n[TCP] 服务器已断开连接")
            stop.set()
            return
        buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            parts = line.decode("utf-8", errors="replace").split()
            # 协议: "POS lat lon [alt]" 或 "lat lon"
            if len(parts) >= 3 and parts[0] == "POS":
                parts = parts[1:]
            elif len(parts) >= 2 and parts[0] != "POS":
                pass
            else:
                continue
            try:
                rlat, rlon = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            ralt = float(parts[2]) if len(parts) >= 3 else 0.0
            with pos_lock:
                remote_pos = {"lat": rlat, "lon": rlon, "alt": ralt}
                loc = local_pos
            print(f"\n[远程机] lat={rlat:.6f}, lon={rlon:.6f}, alt={ralt:.1f}m")
            if loc:
                rel = relative_position(loc["lat"], loc["lon"], rlat, rlon)
                print(f"[相对位置] 距离={rel['distance']:.1f}m, "
                      f"方位角={rel['bearing']:.1f}°(0=北,90=东), "
                      f"东偏={rel['east']:+.1f}m, 北偏={rel['north']:+.1f}m")
            else:
                print("[相对位置] 本机 GPS 尚未定位，无法计算")


# ---------------- 主函数 ----------------

def main() -> None:
    parser = argparse.ArgumentParser(description="GPS 串口定位 + TCP 相对位置比对")
    parser.add_argument("--serial", required=True, help="串口号，如 COM3 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600, help="串口波特率，默认 9600")
    parser.add_argument("--host", required=True, help="服务器 IP 地址")
    parser.add_argument("--port", type=int, required=True, help="服务器端口")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="位置上报周期(秒)，默认 1.0（GPS 秒边界+0.35s 相位错峰）")
    parser.add_argument("--debug", action="store_true", help="打印串口收到的所有原始 NMEA 报文")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.serial, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"串口打开失败: {e}")
        return
    print(f"串口 {args.serial}@{args.baud} 已打开")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    print(f"正在连接服务器 {args.host}:{args.port} ...")
    try:
        sock.connect((args.host, args.port))
    except (ConnectionError, socket.timeout, OSError) as e:
        print(f"TCP 连接失败: {e}")
        print("请检查：服务器程序是否已启动并监听该端口、IP/端口是否正确、"
              "双方是否在同一网段、服务器防火墙是否放行该端口")
        ser.close()
        return
    sock.settimeout(None)
    print(f"已连接服务器 {args.host}:{args.port}")

    stop = threading.Event()
    threads = [
        threading.Thread(target=gps_reader, args=(ser, args.debug), daemon=True),
        threading.Thread(target=position_sender, args=(sock, args.interval, stop), daemon=True),
        threading.Thread(target=remote_receiver, args=(sock, stop), daemon=True),
    ]
    for t in threads:
        t.start()

    print("等待 GPS 定位...（每 10 秒打印一次运行状态，Ctrl+C 退出）")
    last_status = 0.0
    try:
        while not stop.is_set():
            time.sleep(0.5)
            now = time.time()
            if now - last_status >= 10:
                last_status = now
                if last_data_time is None:
                    print("[状态] 串口无数据，请检查：串口号是否正确、是否被其他程序占用、"
                          "波特率是否匹配、接线（TX/RX 是否交叉）", flush=True)
                elif last_fix_time is None:
                    print("[状态] 串口有数据但尚未有效定位（室内通常无卫星信号，"
                          "请加 --debug 查看原始报文确认）", flush=True)
                else:
                    print(f"[状态] 定位正常，上次更新 {now - last_fix_time:.0f} 秒前",
                          flush=True)
    except KeyboardInterrupt:
        print("\n正在退出...")
    finally:
        stop.set()
        sock.close()
        ser.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hang_ctrl.py — 吊物轨迹录制 PC 端命令行控制 (自包含 CLI)

功能:
  1. 无线电 HOOK 协议客户端: 连 Ebyte E90-DTU LoRa 网关 (TCP 8886, 行缓冲, 自动重连)
     PC 发: HOOK REC_START [秒] / HOOK REC_STOP / HOOK STATUS / HOOK TEST
     守护回: HOOK ACK / HOOK REC_STARTED / HOOK REC_DONE <stem> /
             HOOK REC_FAILED / HOOK REC_BUSY / HOOK STATUS <state> [stem]
  2. SSH 轮询权威完成检测: meta.json 出现即完成 (无线电 REC_DONE 仅快提示,
     半双工不可靠 → 完成判定以 SFTP 目录扫描为准)
  3. paramiko SFTP 拉取: /root/trajectory/runs/<stem>.csv + .meta.json (绝对路径,
     --runs-dir 可覆盖) -> hangpoint\<stem>.csv + .meta.json
     字节校验 (stat size 对比) + meta 校验 (rows>0, stop_reason∈{duration,signal},
     signal(N) 统一规范化为 signal)
     .part 临时名 -> 原子改名; 保留 Pi 原件; 不完整对 (csv 有 meta 无) 跳过不落盘
  4. --mock-sftp <dir>: 本地伪 SFTP 目标模式 (无真实 SSH, 供离线测试)

依赖: 仅 Python 标准库 + paramiko (pip install paramiko)。
自包含: 不依赖任何 %TEMP% 助手脚本; 凭据来自 --user/--password/环境变量, 绝不硬编码。
网关与 SSH 目标为现场探测默认值, 均可用命令行参数覆盖。
"""

import argparse
import datetime
import json
import os
import queue
import re
import select
import socket
import sys
import tempfile
import threading
import time

try:
    import paramiko
except ImportError:  # 允许 --selftest/--mock-sftp 在无 paramiko 时运行
    paramiko = None

# --------------------------------------------------------------------------
# 常量 (IP 为 Task 1 探测现场默认值, 全部可被命令行参数覆盖; 凭据不在此)
# --------------------------------------------------------------------------
DEFAULT_GW_HOST = "192.168.0.127"      # Ebyte E90-DTU LoRa 网关 (WLAN 段)
DEFAULT_GW_PORT = 8886                 # TCP Server 模式
DEFAULT_DURATION = 1800                # 录制时长(秒), 与 Pi 守护默认一致
# SSH 目标链: FRP 主路径 -> WLAN 备用路径 (Task 1 实测两者均 PASS)
DEFAULT_SSH_TARGETS = (
    "root@139.196.232.191:6006",
    "root@192.168.0.123:22",
)
DEFAULT_SSH_USER = "root"
DEFAULT_RUNS_DIR = "/root/trajectory/runs"  # Pi 上真实运行目录 (绝对路径, --runs-dir 可覆盖)
RUNS_REL_DIR = "runs"                        # 仅 mock 模式: <root>/runs/ 回退子目录名
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_HANGPOINT_DIR = os.path.abspath(os.path.join(_HERE, "..", "hangpoint"))

STEM_RE = re.compile(r"^(\d{8})-(\d{6})$")   # YYYYMMDD-HHMMSS
# recorder.py 实写 stop_reason 为 "duration" 或 "signal(15)"; 统一规范化为 signal
VALID_STOP_REASONS = ("duration", "signal", "signal(15)")
SIGNAL_REASON_RE = re.compile(r"^signal\(\s*\d+\s*\)$")
HOOK_LINE_RE = re.compile(r"^HOOK\s+(\S+)(?:\s+(.+?))?\s*$")

REC_STARTED_WAIT_S = 60.0      # 含校准 10s + GPS ≤30s
STOP_WAIT_S = 180.0            # SIGTERM 优雅退出 + meta 写入余量
POLL_INTERVAL_S = 5.0          # SSH 权威轮询间隔
STATUS_WINDOW_S = 5.0
SSH_CONNECT_TIMEOUT_S = 15.0
RADIO_CONNECT_TIMEOUT_S = 3.0
RADIO_RECONNECT_DELAY_S = 2.0


class PullError(Exception):
    """拉取/完成检测失败 (不落盘, 非零退出)。"""


class RadioError(Exception):
    """无线电网关不可达。"""


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------
def _fix_stdio():
    """Windows 控制台默认 cp936, 强制 UTF-8 以便证据/日志可读。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def build_msg(kind, payload=""):
    """构造 HOOK 消息行 (不带换行)。"""
    if payload:
        return "HOOK {} {}".format(kind, payload)
    return "HOOK {}".format(kind)


def parse_hook(line):
    """解析一行, 返回 (kind, payload); 非 HOOK 行返回 (None, None)。"""
    m = HOOK_LINE_RE.match((line or "").strip())
    if not m:
        return None, None
    return m.group(1), (m.group(2) or "")


def is_valid_stem(stem):
    """stem 必须是 YYYYMMDD-HHMMSS 且日期真实有效 (同时防路径穿越)。"""
    if not isinstance(stem, str) or not STEM_RE.match(stem):
        return False
    try:
        datetime.datetime.strptime(stem, "%Y%m%d-%H%M%S")
    except ValueError:
        return False
    return True


def normalize_stop_reason(reason):
    """stop_reason 规范化: duration/signal 原样; signal(N) (如 signal(15)) -> signal; 其他 -> None。"""
    if not isinstance(reason, str):
        return None
    r = reason.strip()
    if r in ("duration", "signal"):
        return r
    if SIGNAL_REASON_RE.match(r):
        return "signal"
    return None


def validate_meta(meta_bytes, stem):
    """meta.json 校验: JSON 可解析 + rows>0 + stop_reason 合法 (duration/signal/signal(N))。

    返回规范化后的 meta (stop_reason 统一为 duration 或 signal)。"""
    try:
        meta = json.loads(meta_bytes.decode("utf-8"))
    except Exception as exc:
        raise PullError("meta.json 解析失败: stem={} err={}".format(stem, exc))
    try:
        rows = int(meta.get("rows", -1))
    except (TypeError, ValueError):
        rows = -1
    if rows <= 0:
        raise PullError("meta 校验失败: stem={} rows={!r} 必须 >0".format(stem, meta.get("rows")))
    reason = meta.get("stop_reason")
    norm = normalize_stop_reason(reason)
    if norm is None:
        raise PullError(
            "meta 校验失败: stem={} stop_reason={!r} 必须属于 {} (signal(N) 规范化为 signal)".format(
                stem, reason, VALID_STOP_REASONS))
    meta["stop_reason"] = norm
    return meta


def write_atomic(target_path, data):
    """.part 临时名写入 -> os.replace 原子改名; 任何失败清理 .part 残留。"""
    tmp_path = target_path + ".part"
    try:
        with open(tmp_path, "wb") as fh:
            fh.write(data)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_path, target_path)
    except Exception:
        try:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
        except OSError:
            pass
        raise


def _expect_pull_error(fn):
    try:
        fn()
    except PullError:
        return True
    except Exception:
        return True
    return False


# --------------------------------------------------------------------------
# 无线电客户端 (结构借鉴 ui_display.py NetworkThread 行缓冲/重连模式, 独立实现)
# --------------------------------------------------------------------------
class RadioClient:
    """后台读线程 + 行缓冲 + 断线自动重连; send()/wait_for() 同步接口。"""

    def __init__(self, host, port,
                 connect_timeout=RADIO_CONNECT_TIMEOUT_S,
                 reconnect_delay=RADIO_RECONNECT_DELAY_S):
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.reconnect_delay = reconnect_delay
        self.rx_queue = queue.Queue()
        self.sock = None
        self.connected = threading.Event()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="radio-rx", daemon=True)
        self.thread.start()

    # -- 内部 --
    def _connect(self):
        s = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
        s.settimeout(None)
        return s

    def _drop(self):
        sock, self.sock = self.sock, None
        self.connected.clear()
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _run(self):
        buf = ""
        while not self.stop_event.is_set():
            if self.sock is None:
                try:
                    self.sock = self._connect()
                except OSError:
                    if self.stop_event.wait(self.reconnect_delay):
                        break
                    continue
                self.connected.set()
                buf = ""
            try:
                readable, _, _ = select.select([self.sock], [], [], 0.2)
                if self.sock in readable:
                    data = self.sock.recv(4096)
                    if not data:
                        raise ConnectionResetError("recv: connection closed by peer")
                    buf += data.decode("utf-8", errors="replace")
                    while "\n" in buf:
                        raw, buf = buf.split("\n", 1)
                        line = raw.strip()
                        if line:
                            self.rx_queue.put(line)
            except (OSError, ValueError):
                self._drop()

    # -- 对外 --
    def is_connected(self):
        return self.sock is not None and self.connected.is_set()

    def send(self, text):
        """发送一行 (自动补 \\n); 断线时短暂等待重连。返回是否成功发出。"""
        if not text.endswith("\n"):
            text += "\n"
        payload = text.encode("utf-8")
        deadline = time.monotonic() + self.connect_timeout + self.reconnect_delay
        while time.monotonic() < deadline:
            if self.sock is not None and self.connected.is_set():
                try:
                    self.sock.sendall(payload)
                    return True
                except OSError:
                    self._drop()
            time.sleep(0.1)
        return False

    def wait_raw(self, timeout):
        """取任意一行, 超时返回 None。"""
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def wait_for(self, predicate, timeout):
        """等待满足 predicate(line) 的一行, 超时返回 None。"""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            line = self.wait_raw(min(remaining, 0.5))
            if line is None:
                continue
            if predicate(line):
                return line

    def close(self):
        self.stop_event.set()
        self._drop()
        try:
            self.thread.join(timeout=1.0)
        except Exception:
            pass


def hook_wait(radio, kinds, timeout):
    """等待指定 kind 集合中的任一行; 返回原始行或 None。"""
    kinds = set(kinds)

    def pred(line):
        kind, _ = parse_hook(line)
        return kind in kinds

    return radio.wait_for(pred, timeout)


# --------------------------------------------------------------------------
# SFTP 后端抽象 (真实 paramiko / 本地 mock 共用拉取逻辑)
# --------------------------------------------------------------------------
class BaseBackend:
    def list_runs(self):
        """返回 {stem: {"csv": size|None, "meta": size|None}}。"""
        raise NotImplementedError

    def read_bytes(self, relpath):
        raise NotImplementedError

    def describe(self):
        raise NotImplementedError

    def close(self):
        pass


class MockBackend(BaseBackend):
    """本地伪 SFTP 目标: <root>/runs/<stem>.csv + .meta.json (无真实 SSH)。
    兼容 <root> 下无 runs/ 子目录时直接扫描 <root> 本身。"""

    def __init__(self, root):
        self.root = os.path.abspath(root)
        runs = os.path.join(self.root, RUNS_REL_DIR)
        self.runs_dir = runs if os.path.isdir(runs) else self.root

    def _entry(self):
        result = {}
        try:
            with os.scandir(self.runs_dir) as it:
                for e in it:
                    if not e.is_file():
                        continue
                    name = e.name
                    if name.endswith(".csv"):
                        stem, key = name[:-4], "csv"
                    elif name.endswith(".meta.json"):
                        stem, key = name[:-len(".meta.json")], "meta"
                    else:
                        continue
                    rec = result.setdefault(stem, {"csv": None, "meta": None})
                    rec[key] = e.stat().st_size
        except OSError:
            pass
        return result

    def list_runs(self):
        return self._entry()

    def read_bytes(self, relpath):
        name = os.path.basename(relpath)
        with open(os.path.join(self.runs_dir, name), "rb") as fh:
            return fh.read()

    def describe(self):
        return "mock-sftp {}".format(self.root)

    def close(self):
        pass


class SshBackend(BaseBackend):
    """paramiko SFTP 后端; targets=[(host,port,user)...] 依序尝试, 失败可重置重连。

    runs_dir 为 Pi 上运行的**绝对路径** (默认 /root/trajectory/runs, --runs-dir 可覆盖),
    不依赖任何相对路径或符号链接。"""

    def __init__(self, targets, password, runs_dir=DEFAULT_RUNS_DIR):
        self.targets = targets
        self.password = password
        self.runs_dir = (runs_dir or DEFAULT_RUNS_DIR).rstrip("/") or "/"
        self.client = None
        self.sftp = None
        self.active = None

    def connect(self):
        last_err = None
        for host, port, user in self.targets:
            client = None
            try:
                client = paramiko.SSHClient()
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                client.connect(
                    host, port=port, username=user, password=self.password,
                    timeout=SSH_CONNECT_TIMEOUT_S, banner_timeout=SSH_CONNECT_TIMEOUT_S,
                    auth_timeout=SSH_CONNECT_TIMEOUT_S,
                    look_for_keys=False, allow_agent=False)
                self.client = client
                self.sftp = client.open_sftp()
                self.active = (host, port, user)
                return True
            except Exception as exc:
                last_err = exc
                if client is not None:
                    try:
                        client.close()
                    except Exception:
                        pass
        raise PullError("SSH 连接失败 (已尝试全部目标): {}".format(last_err))

    def ensure(self):
        if self.sftp is None:
            self.connect()

    def reset(self):
        self.close()

    def list_runs(self):
        self.ensure()
        try:
            attrs = self.sftp.listdir_attr(self.runs_dir)
        except OSError as exc:
            if getattr(exc, "errno", None) == 2:  # runs/ 尚不存在 → 空
                return {}
            self.reset()
            raise
        result = {}
        for a in attrs:
            name = a.filename
            if name.endswith(".csv"):
                stem, key = name[:-4], "csv"
            elif name.endswith(".meta.json"):
                stem, key = name[:-len(".meta.json")], "meta"
            else:
                continue
            rec = result.setdefault(stem, {"csv": None, "meta": None})
            rec[key] = a.st_size
        return result

    def read_bytes(self, relpath):
        self.ensure()
        path = self.runs_dir + "/" + os.path.basename(relpath)
        try:
            with self.sftp.open(path, "rb") as fh:
                data = fh.read()
        except OSError:
            self.reset()
            raise
        return data

    def describe(self):
        if self.active:
            return "ssh {}@{}:{}".format(*self.active)
        return "ssh (未连接)"

    def close(self):
        if self.sftp is not None:
            try:
                self.sftp.close()
            except Exception:
                pass
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
        self.sftp = None
        self.client = None


# --------------------------------------------------------------------------
# 拉取逻辑 (所有子命令共用)
# --------------------------------------------------------------------------
def pull_stem(backend, stem, hangpoint_dir):
    """校验配对/meta/字节后, 原子落盘 <hangpoint>\\<stem>.csv + .meta.json。"""
    if not is_valid_stem(stem):
        raise PullError("非法 stem: {!r} (必须 YYYYMMDD-HHMMSS)".format(stem))
    files = backend.list_runs()
    entry = files.get(stem)
    if entry is None:
        raise PullError("运行目录中无 stem={} (不存在或元数据缺失)".format(stem))
    if entry["csv"] is None or entry["meta"] is None:
        raise PullError(
            "不完整对: stem={} csv={} meta={} → 跳过, 不落盘".format(
                stem,
                "有" if entry["csv"] is not None else "无",
                "有" if entry["meta"] is not None else "无"))
    meta_bytes = backend.read_bytes(stem + ".meta.json")
    meta = validate_meta(meta_bytes, stem)  # 失败即抛, 不落盘
    csv_bytes = backend.read_bytes(stem + ".csv")
    if len(csv_bytes) != entry["csv"]:
        raise PullError("字节校验失败: stem={} 期望 {} 字节, 实读 {} 字节".format(
            stem, entry["csv"], len(csv_bytes)))
    if len(meta_bytes) != entry["meta"]:
        raise PullError("字节校验失败: meta 期望 {} 字节, 实读 {} 字节".format(
            stem, entry["meta"], len(meta_bytes)))
    os.makedirs(hangpoint_dir, exist_ok=True)
    write_atomic(os.path.join(hangpoint_dir, stem + ".csv"), csv_bytes)
    write_atomic(os.path.join(hangpoint_dir, stem + ".meta.json"), meta_bytes)
    return meta


def resolve_ssh_targets(args, user):
    """--ssh-target > 环境变量 HANG_PI_SSH_TARGET > 默认链 (FRP 主 → WLAN 备)。"""
    if args.ssh_target:
        specs = [args.ssh_target]
    else:
        env_spec = os.environ.get("HANG_PI_SSH_TARGET")
        specs = env_spec.split(",") if env_spec else list(DEFAULT_SSH_TARGETS)
    out = []
    for spec in specs:
        spec = spec.strip()
        if not spec:
            continue
        u, rest = user, spec
        if "@" in spec:
            u, rest = spec.split("@", 1)
        if ":" in rest:
            host, _, port_s = rest.rpartition(":")
            port = int(port_s) if port_s else 22
        else:
            host, port = rest, 22
        out.append((host.strip(), port, u.strip()))
    if not out:
        raise PullError("无可用 SSH 目标 (--ssh-target/HANG_PI_SSH_TARGET 为空)")
    return out


def open_backend(args):
    """按参数构造后端: --mock-sftp 本地模式, 否则 paramiko SFTP。"""
    if args.mock_sftp:
        backend = MockBackend(args.mock_sftp)
        print("[ssh] 使用本地伪 SFTP 目标: {}".format(backend.describe()))
        return backend
    if paramiko is None:
        raise PullError("paramiko 未安装: 请执行 pip install paramiko")
    user = args.user or os.environ.get("HANG_PI_USER") or DEFAULT_SSH_USER
    password = args.password if args.password is not None else os.environ.get("HANG_PI_PASS")
    if not password:
        raise PullError("缺少 SSH 凭据: 请用 --password 或环境变量 HANG_PI_PASS (凭据绝不硬编码)")
    targets = resolve_ssh_targets(args, user)
    backend = SshBackend(targets, password, args.runs_dir)
    backend.connect()
    print("[ssh] 已连接 {} (runs-dir: {}, SSH 轮询权威通道)".format(
        backend.describe(), backend.runs_dir))
    return backend


def _pick_latest_complete(backend):
    files = backend.list_runs()
    cands = [s for s, e in files.items() if e["csv"] is not None and e["meta"] is not None]
    if not cands:
        raise PullError("无完整对 (csv+meta.json 齐备) 可拉取")
    return max(cands)


def _poll_new_meta(backend, baseline):
    """SSH 权威: 返回 baseline 之外最新出现的带 meta.json 的 stem, 无则 None。"""
    files = backend.list_runs()
    new = [s for s, e in files.items() if e["meta"] is not None and s not in baseline]
    if not new:
        return None
    return max(new)


# --------------------------------------------------------------------------
# 子命令
# --------------------------------------------------------------------------
def cmd_start(args):
    duration = args.duration
    if duration is not None and duration <= 0:
        print("[err] --duration 必须 > 0", file=sys.stderr)
        return 1
    eff_duration = duration if duration is not None else DEFAULT_DURATION
    start_msg = build_msg("REC_START", str(duration)) if duration is not None else build_msg("REC_START")
    backend = None
    radio = RadioClient(args.host, args.port)
    try:
        backend = open_backend(args)  # 先验证 SSH 通道, 失败则不发 REC_START
        if not radio.send(start_msg):
            raise RadioError("无法连接无线电网关 {}:{}".format(args.host, args.port))
        print("[radio] 已发送: {} (时长: {})".format(start_msg,
              "{}s".format(eff_duration) if duration is not None else "守护默认 {}s".format(DEFAULT_DURATION)))
        line = hook_wait(radio, {"REC_STARTED", "REC_BUSY", "REC_FAILED"}, REC_STARTED_WAIT_S)
        kind, payload = parse_hook(line or "")
        if kind == "REC_FAILED":
            raise PullError("录制失败 (REC_FAILED, 如无 GPS fix) → 不拉取")
        if kind == "REC_BUSY":
            print("[radio] 收到 REC_BUSY (录制已在进行) → 转入 SSH 权威轮询")
        elif kind == "REC_STARTED":
            print("[radio] 收到 REC_STARTED")
            print("[!] 录制已启动: 校准约 10s + GPS 获取 ≤30s, 请保持吊钩静止!")
        else:
            print("[radio] 警告: {:.0f}s 内未收到 REC_STARTED/REC_BUSY/REC_FAILED "
                  "(半双工可能丢包) → 转入 SSH 权威轮询".format(REC_STARTED_WAIT_S))
        baseline = {s for s, e in backend.list_runs().items() if e["meta"] is not None}
        deadline = time.monotonic() + eff_duration + 60.0
        stem = None
        radio_stem = None
        while time.monotonic() < deadline:
            hint = hook_wait(radio, {"REC_DONE", "REC_FAILED"}, POLL_INTERVAL_S)
            if hint:
                hkind, hpayload = parse_hook(hint)
                if hkind == "REC_FAILED":
                    raise PullError("录制失败 (REC_FAILED) → 不拉取")
                candidate = hpayload.split()[0] if hpayload else ""
                if is_valid_stem(candidate):
                    radio_stem = candidate
                    print("[radio] 快提示 REC_DONE {} (完成判定仍以 SSH 为准)".format(candidate))
            try:
                found = _poll_new_meta(backend, baseline)
            except PullError as exc:
                print("[ssh] 轮询失败: {} → 重连后继续".format(exc))
                time.sleep(1.0)
                continue
            if found:
                stem = found
                break
            if radio_stem:
                try:
                    files = backend.list_runs()
                    if files.get(radio_stem, {}).get("meta") is not None:
                        stem = radio_stem
                        break
                except PullError:
                    pass
        if stem is None:
            raise PullError("等待完成超时 ({}s): 未发现带 meta.json 的新 stem".format(int(eff_duration + 60)))
        meta = pull_stem(backend, stem, args.hangpoint_dir)
        print("[ok] 已拉取 stem={} → {} (rows={}, stop_reason={})".format(
            stem, args.hangpoint_dir, meta.get("rows"), meta.get("stop_reason")))
        return 0
    except (PullError, RadioError) as exc:
        print("[err] {}".format(exc), file=sys.stderr)
        return 1
    finally:
        radio.close()
        if backend is not None:
            backend.close()


def cmd_stop(args):
    backend = None
    radio = RadioClient(args.host, args.port)
    try:
        backend = open_backend(args)
        if not radio.send(build_msg("REC_STOP")):
            raise RadioError("无法连接无线电网关 {}:{}".format(args.host, args.port))
        print("[radio] 已发送: HOOK REC_STOP")
        baseline = {s for s, e in backend.list_runs().items() if e["meta"] is not None}
        deadline = time.monotonic() + STOP_WAIT_S
        stem = None
        radio_stem = None
        while time.monotonic() < deadline:
            hint = hook_wait(radio, {"REC_DONE", "REC_FAILED"}, POLL_INTERVAL_S)
            if hint:
                hkind, hpayload = parse_hook(hint)
                if hkind == "REC_FAILED":
                    raise PullError("录制失败 (REC_FAILED) → 不拉取")
                candidate = hpayload.split()[0] if hpayload else ""
                if is_valid_stem(candidate):
                    radio_stem = candidate
                    print("[radio] 快提示 REC_DONE {} (完成判定仍以 SSH 为准)".format(candidate))
            try:
                found = _poll_new_meta(backend, baseline)
            except PullError as exc:
                print("[ssh] 轮询失败: {} → 重连后继续".format(exc))
                time.sleep(1.0)
                continue
            if found:
                stem = found
                break
            if radio_stem:
                try:
                    files = backend.list_runs()
                    if files.get(radio_stem, {}).get("meta") is not None:
                        stem = radio_stem
                        break
                except PullError:
                    pass
        if stem is None:
            raise PullError("停止等待超时 ({}s): 未发现带 meta.json 的新 stem".format(int(STOP_WAIT_S)))
        meta = pull_stem(backend, stem, args.hangpoint_dir)
        print("[ok] 已拉取 stem={} → {} (rows={}, stop_reason={})".format(
            stem, args.hangpoint_dir, meta.get("rows"), meta.get("stop_reason")))
        return 0
    except (PullError, RadioError) as exc:
        print("[err] {}".format(exc), file=sys.stderr)
        return 1
    finally:
        radio.close()
        if backend is not None:
            backend.close()


def cmd_status(args):
    radio = RadioClient(args.host, args.port)
    try:
        if not radio.send(build_msg("STATUS")):
            raise RadioError("无法连接无线电网关 {}:{}".format(args.host, args.port))
        print("[radio] 已发送: HOOK STATUS")
        got = []
        deadline = time.monotonic() + STATUS_WINDOW_S
        while time.monotonic() < deadline:
            line = radio.wait_raw(min(0.5, deadline - time.monotonic()))
            if line is None:
                continue
            kind, payload = parse_hook(line)
            if kind is not None:
                got.append((kind, payload))
                print("[status] HOOK {} {}".format(kind, payload))
        if not got:
            raise RadioError("{:.0f}s 内未收到状态响应 (对端无监听?)".format(STATUS_WINDOW_S))
        return 0
    except RadioError as exc:
        print("[err] {}".format(exc), file=sys.stderr)
        return 1
    finally:
        radio.close()


def cmd_test(args):
    radio = RadioClient(args.host, args.port)
    try:
        if not radio.send(build_msg("TEST")):
            raise RadioError("无法连接无线电网关 {}:{}".format(args.host, args.port))
        print("[radio] 已发送: HOOK TEST (等待 HOOK ACK ≤60s)")
        line = hook_wait(radio, {"ACK"}, 60.0)
        if line is None:
            print("[err] 60s 内未收到 HOOK ACK", file=sys.stderr)
            return 1
        print("[ok] 收到 {}".format(line.strip()))
        return 0
    except RadioError as exc:
        print("[err] {}".format(exc), file=sys.stderr)
        return 1
    finally:
        radio.close()


def cmd_pull(args):
    backend = None
    try:
        backend = open_backend(args)
        if args.stem:
            stem = args.stem
        else:
            stem = _pick_latest_complete(backend)
            print("[pull] 最新完整对: {}".format(stem))
        meta = pull_stem(backend, stem, args.hangpoint_dir)
        print("[ok] 已拉取 stem={} → {} (rows={}, stop_reason={})".format(
            stem, args.hangpoint_dir, meta.get("rows"), meta.get("stop_reason")))
        return 0
    except PullError as exc:
        print("[err] {}".format(exc), file=sys.stderr)
        return 1
    finally:
        if backend is not None:
            backend.close()


# --------------------------------------------------------------------------
# 自检 (纯离线: 消息构造/解析/改名/拉取逻辑, 不触网)
# --------------------------------------------------------------------------
def run_selftest():
    checks = []

    def check(name, cond, detail=""):
        checks.append((name, bool(cond), detail))
        line = "[PASS] {}" if cond else "[FAIL] {}"
        print(line.format(name) + (" — " + detail if detail else ""))

    # 1. 消息构造
    check("build_msg(REC_START)", build_msg("REC_START") == "HOOK REC_START")
    check("build_msg(REC_START+30)", build_msg("REC_START", "30") == "HOOK REC_START 30")
    check("build_msg(REC_STOP)", build_msg("REC_STOP") == "HOOK REC_STOP")
    check("build_msg(REC_DONE+stem)",
          build_msg("REC_DONE", "20260825-153000") == "HOOK REC_DONE 20260825-153000")

    # 2. 消息解析
    check("parse REC_DONE", parse_hook("HOOK REC_DONE 20260825-153000") == ("REC_DONE", "20260825-153000"))
    check("parse REC_STARTED", parse_hook("HOOK REC_STARTED") == ("REC_STARTED", ""))
    check("parse STATUS", parse_hook("HOOK STATUS recording 20260825-153000")
          == ("STATUS", "recording 20260825-153000"))
    check("parse 非HOOK行忽略", parse_hook("$GPGGA,123519,,,,0,00,,,M,,M,,*47") == (None, None))

    # 3. stem 校验
    check("stem 合法", is_valid_stem("20260825-153000"))
    check("stem 非法(字符)", not is_valid_stem("bad-stem"))
    check("stem 非法(路径穿越)", not is_valid_stem("../../etc/passwd"))
    check("stem 非法(长度)", not is_valid_stem("20260825-15300"))
    check("stem 非法(日期)", not is_valid_stem("20261345-999999"))

    # 4. meta 校验
    check("meta 合法(duration)",
          validate_meta(json.dumps({"rows": 120, "stop_reason": "duration"}).encode("utf-8"), "s")["rows"] == 120)
    check("meta 合法(signal)",
          validate_meta(json.dumps({"rows": 1, "stop_reason": "signal"}).encode("utf-8"), "s")["stop_reason"] == "signal")
    check("meta 合法(signal(15) 规范化→signal)",
          validate_meta(json.dumps({"rows": 1, "stop_reason": "signal(15)"}).encode("utf-8"), "s")["stop_reason"] == "signal")
    check("meta 合法(signal(9) 规范化→signal)",
          validate_meta(json.dumps({"rows": 1, "stop_reason": "signal(9)"}).encode("utf-8"), "s")["stop_reason"] == "signal")
    check("meta rows=0 拒绝",
          _expect_pull_error(lambda: validate_meta(json.dumps({"rows": 0, "stop_reason": "duration"}).encode("utf-8"), "s")))
    check("meta 坏 stop_reason 拒绝",
          _expect_pull_error(lambda: validate_meta(json.dumps({"rows": 5, "stop_reason": "crash"}).encode("utf-8"), "s")))
    check("meta 坏 stop_reason 拒绝(signalX)",
          _expect_pull_error(lambda: validate_meta(json.dumps({"rows": 5, "stop_reason": "signalx"}).encode("utf-8"), "s")))
    check("meta 非法 JSON 拒绝",
          _expect_pull_error(lambda: validate_meta(b"not-json", "s")))
    check("normalize_stop_reason(None)",
          normalize_stop_reason(None) is None and normalize_stop_reason(15) is None)
    check("normalize_stop_reason(空串拒绝)", normalize_stop_reason("") is None)

    # 4b. runs-dir 路径 (缺陷#1: 绝对路径, 不依赖相对路径/符号链接)
    check("runs-dir 默认绝对路径",
          SshBackend([("h", 22, "u")], "p").runs_dir == DEFAULT_RUNS_DIR)
    check("runs-dir 覆盖+去尾斜杠",
          SshBackend([("h", 22, "u")], "p", "/tmp/x/").runs_dir == "/tmp/x")

    # 5. 原子改名逻辑 (.part → os.replace)
    with tempfile.TemporaryDirectory() as td:
        target = os.path.join(td, "x.csv")
        write_atomic(target, b"hello-world")
        ok_content = open(target, "rb").read() == b"hello-world"
        ok_no_part = not any(n.endswith(".part") for n in os.listdir(td))
        write_atomic(target, b"second")  # 覆盖已有文件
        ok_overwrite = open(target, "rb").read() == b"second"
        ok_no_part2 = not any(n.endswith(".part") for n in os.listdir(td))
        check("原子改名+字节一致", ok_content and ok_no_part)
        check("覆盖写入+无 .part 残留", ok_overwrite and ok_no_part2)

    # 6. 不完整对 (csv 有 meta 无) → 报错且不落盘
    with tempfile.TemporaryDirectory() as td:
        with open(os.path.join(td, "20260825-153000.csv"), "wb") as fh:
            fh.write(b"t,x,y\n1,2,3\n")
        out_dir = os.path.join(td, "out")
        raised = _expect_pull_error(lambda: pull_stem(MockBackend(td), "20260825-153000", out_dir))
        nothing = (not os.path.isdir(out_dir)) or os.listdir(out_dir) == []
        check("不完整对→报错不落盘", raised and nothing)

    # 7. 完整 mock 拉取 (端到端, 离线, 字节一致 + 无 .part)
    with tempfile.TemporaryDirectory() as td:
        runs = os.path.join(td, RUNS_REL_DIR)
        os.makedirs(runs)
        stem = "20260825-153000"
        csv_bytes = ("t,x,y\n" + "".join("{},1,2\n".format(i) for i in range(100))).encode("utf-8")
        meta_bytes = json.dumps({"rows": 100, "stop_reason": "duration"}).encode("utf-8")
        with open(os.path.join(runs, stem + ".csv"), "wb") as fh:
            fh.write(csv_bytes)
        with open(os.path.join(runs, stem + ".meta.json"), "wb") as fh:
            fh.write(meta_bytes)
        out_dir = os.path.join(td, "hangpoint")
        meta = pull_stem(MockBackend(td), stem, out_dir)
        ok_csv = open(os.path.join(out_dir, stem + ".csv"), "rb").read() == csv_bytes
        ok_meta = open(os.path.join(out_dir, stem + ".meta.json"), "rb").read() == meta_bytes
        ok_no_part = not any(n.endswith(".part") for n in os.listdir(out_dir))
        check("mock 拉取字节一致+无 .part", ok_csv and ok_meta and ok_no_part and meta["rows"] == 100)

    total = len(checks)
    failed = sum(1 for _, ok, _ in checks if not ok)
    print("\nselftest: {}/{} 通过".format(total - failed, total))
    return 0 if failed == 0 else 1


# --------------------------------------------------------------------------
# 入口
# --------------------------------------------------------------------------
def build_parser():
    # 公共选项同时挂主解析器与各子命令 (argparse 在子命令后不再识别主解析器选项)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--host", default=DEFAULT_GW_HOST,
                        help="无线电网关 IP (默认 %(default)s, Task 1 探测值)")
    common.add_argument("--port", type=int, default=DEFAULT_GW_PORT,
                        help="无线电网关 TCP 端口 (默认 %(default)s)")
    common.add_argument("--user", default=None,
                        help="SSH 用户名 (默认: 环境变量 HANG_PI_USER, 否则 root)")
    common.add_argument("--password", default=None,
                        help="SSH 密码 (默认取环境变量 HANG_PI_PASS; 不硬编码)")
    common.add_argument("--ssh-target", metavar="[USER@]HOST:PORT", default=None,
                        help="SSH 目标, 覆盖默认链 (默认: FRP 139.196.232.191:6006 主 → WLAN 192.168.0.123:22 备)")
    common.add_argument("--hangpoint-dir", default=DEFAULT_HANGPOINT_DIR,
                        help="本地落盘目录 (默认 %(default)s)")
    common.add_argument("--runs-dir", default=DEFAULT_RUNS_DIR,
                        help="Pi 上运行目录绝对路径, SFTP list/get 用此路径 "
                             "(默认 %(default)s, 无符号链接依赖)")
    common.add_argument("--mock-sftp", metavar="DIR", default=None,
                        help="本地伪 SFTP 目标目录 (离线测试, 无真实 SSH; 取 <DIR>/runs/ 或 <DIR> 本身)")
    common.add_argument("--selftest", action="store_true",
                        help="离线自检 (消息构造/解析/改名/拉取逻辑), 退出码 0=通过")

    p = argparse.ArgumentParser(
        prog="hang_ctrl.py",
        parents=[common],
        description="吊物轨迹录制 PC 端控制 CLI (无线电 HOOK 协议 + SSH 轮询权威 + SFTP 拉取)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  python hang_ctrl.py start --duration 30    (30s 录制: 发送 HOOK REC_START 30)
  python hang_ctrl.py start                  (缺省时长: 发送 HOOK REC_START, 守护用默认)
  python hang_ctrl.py stop
  python hang_ctrl.py status
  python hang_ctrl.py pull --stem 20260825-153000
  python hang_ctrl.py pull --latest
  python hang_ctrl.py test
  python hang_ctrl.py --selftest
  python hang_ctrl.py pull --stem S --mock-sftp D:\\fake-pi   (离线测试)

凭据: --user/--password 或环境变量 HANG_PI_USER / HANG_PI_PASS (绝不硬编码)。
SSH 目标: --ssh-target 或环境变量 HANG_PI_SSH_TARGET (逗号分隔多目标依次尝试)。
运行目录: --runs-dir 覆盖 Pi 上运行目录 (默认 /root/trajectory/runs, 绝对路径)。
协议: HOOK REC_START [<seconds>] — 带 <秒> 时守护按该时长录制, 缺省用守护默认。""")

    sub = p.add_subparsers(dest="command")

    sp = sub.add_parser("start", parents=[common],
                        help="开始录制: HOOK REC_START [秒] → 等 REC_STARTED → SSH 权威轮询 → SFTP 拉取")
    sp.add_argument("--duration", type=int, default=None,
                    help="录制时长(秒): 发送 HOOK REC_START <秒> 并用于轮询超时 = duration+60; "
                         "缺省不发送, 守护用自身默认 ({}s)".format(DEFAULT_DURATION))

    sub.add_parser("stop", parents=[common],
                   help="提前结束: HOOK REC_STOP → 等 REC_DONE → SFTP 拉取")

    sub.add_parser("status", parents=[common],
                   help="查询: HOOK STATUS → 打印守护状态")

    sp = sub.add_parser("pull", parents=[common],
                        help="SFTP 拉取 runs/<stem>.csv + .meta.json → hangpoint (完整对才拉)")
    g = sp.add_mutually_exclusive_group(required=True)
    g.add_argument("--stem", help="指定 stem (YYYYMMDD-HHMMSS)")
    g.add_argument("--latest", action="store_true", help="拉最新且有 meta.json 的完整对")

    sub.add_parser("test", parents=[common],
                   help="回环测试: HOOK TEST → 等 HOOK ACK (≤60s)")
    return p


def main(argv=None):
    _fix_stdio()
    args = build_parser().parse_args(argv)
    if args.selftest:
        return run_selftest()
    if args.command == "start":
        return cmd_start(args)
    if args.command == "stop":
        return cmd_stop(args)
    if args.command == "status":
        return cmd_status(args)
    if args.command == "pull":
        return cmd_pull(args)
    if args.command == "test":
        return cmd_test(args)
    build_parser().print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())

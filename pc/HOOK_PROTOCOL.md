# HOOK 协议文档（吊物轨迹录制）

PC 端 `hang_ctrl.py` 与吊物端 Pi 守护 `radio_recorder.py` 之间通过 Ebyte E90-DTU LoRa 数传网关（TCP 8886，文本行协议）交换的指令与应答。本文描述协议格式、行为语义及其与船控协议的隔离，与两侧实现严格一致。

## 1. 消息格式总览

所有消息均为单行 UTF-8 文本，以换行符结尾，格式 `HOOK <命令> [<参数>]`。参数间以单个空格分隔。

### 1.1 PC → Pi（指令）

| 消息 | 说明 |
|------|------|
| `HOOK REC_START [<seconds>]` | 开始录制。`<seconds>` 为正数秒时按该时长录制；**缺省、非法或非正数时用守护默认 1800s**。例：`HOOK REC_START 30`、`HOOK REC_START` |
| `HOOK REC_STOP` | 提前结束当前录制（守护对 recorder 发 SIGTERM，绝不 SIGKILL） |
| `HOOK STATUS` | 查询守护状态 |
| `HOOK TEST` | 回环测试，守护应答 `HOOK ACK` |

### 1.2 Pi → PC（应答）

| 消息 | 说明 |
|------|------|
| `HOOK ACK` | 对 `HOOK TEST` 的回环应答 |
| `HOOK REC_STARTED` | 录制已启动（校准 10s + GPS 定位 ≤30s 后进行采样） |
| `HOOK REC_DONE <stem>` | 录制完成，`<stem>` 为 `YYYYMMDD-HHMMSS`（Pi 本地时间）。**收到后才可用 stem 经 SFTP 拉取** `<stem>.csv` 与 `<stem>.meta.json` |
| `HOOK REC_FAILED` | 录制失败（如无 GPS fix、退出码非 0）。**一旦发出绝不再发 `REC_DONE`** |
| `HOOK REC_BUSY` | 已有录制在进行，拒绝重复 `REC_START` |
| `HOOK STATUS <idle\|recording\|busy> [stem]` | 状态应答。`recording`/`busy` 时带当前 stem，`idle` 不带 |

## 2. 行为语义

### 2.1 REC_START 时长参数

`HOOK REC_START [<seconds>]` 的 `<seconds>` 解析规则（守护 `parse_duration_arg`）：

- 正数（整数或小数，如 `30`、`45.5`）→ 按该秒数录制；
- 缺省、非数字、非正数（`abc`、`-5`、`0`）→ 回退守护默认 **1800.0s**。

`hang_ctrl.py start` 缺省不发送 `<seconds>`（守护用自身默认）；`start --duration N` 则发送 `HOOK REC_START N`。

### 2.2 完成与失败判定

- **exit 0 + meta.json 确认** → `HOOK REC_DONE <stem>`（守护先等 meta.json 落盘，才发 REC_DONE）。
- **exit 0 但 meta.json 未确认** → `HOOK REC_FAILED`（不发 REC_DONE）。
- **exit 2（无 GPS fix 超时）** → `HOOK REC_FAILED`。
- **任何非 0 退出码** → `HOOK REC_FAILED`，绝无 REC_DONE。

### 2.3 REC_STOP 提前停止

`HOOK REC_STOP` → 守护对 recorder 子进程发 SIGTERM（优雅退出，非 SIGKILL）→ 等子进程退出并写 meta.json → 发 `HOOK REC_DONE <stem>`。meta 中 `stop_reason` 落盘为 `signal(15)`，PC 端拉取时规范化为 `signal`。

### 2.4 半双工说明

无线电链路为半双工，PC 端不依赖 REC_DONE 作为完成权威。`hang_ctrl.py` 以 SSH 轮询 Pi 运行目录（默认 `/root/trajectory/runs`）出现 meta.json 为准，`HOOK REC_DONE` 仅作快提示。REC_DONE 的 stem 与 SSH 发现结果不一致时，以 SSH 发现为准。

### 2.5 拉取契约

`HOOK REC_DONE <stem>` 只在 meta.json 落盘确认后才会发出，此时 Pi 侧已存在 `<stem>.csv` + `<stem>.meta.json` 完整对。PC 端 `pull_stem` 要求：JSON 可解析、`rows > 0`、`stop_reason ∈ {duration, signal}`（`signal(N)` 规范化后）。任何一项不满足则报错且不落盘；不完整对（csv 有 meta 无）跳过不落盘。

## 3. 与船控 CMD 前缀的隔离

8886 通路由多套系统共享，为防误触发作以下隔离约定：

- **`HOOK` 前缀专属吊物录制**：守护只解析以 `HOOK` 开头的行。非 `HOOK` 前缀的行（含船控 `CMD`、吊物 `POS`、船端 `USV`）一律忽略并记日志，不触发任何录制行为。
- **大小写敏感**：`hook rec_start` 等小写形式不被识别（selftest 有断言）。
- **船控命令不携带 HOOK 前缀**：船控侧命令（如 `CMD SET_ORIGIN`、`CMD RUN_EXP N`、`CMD STOP`、`CMD REC_START`、`CMD REC_STOP`）由船控侧解析，与吊物 HOOK 命令字面同名也不冲突，因为前缀不同。
- **HOOK 命令的歧义保护**：守护对未知 `HOOK <命令>` 忽略并记日志，不静默执行。

典型误触发场景已由守护 selftest 覆盖：`CMD REC_START` 解析结果为 None（拒绝），`garbage` 与大小写错误同样拒绝。

## 4. 与 PC 端实现的一致性

下表为 `hang_ctrl.py` 中实际构造/解析的协议行（可经 `--selftest` 验证）：

| 场景 | 实际行 |
|------|--------|
| `start --duration 30` 发送 | `HOOK REC_START 30` |
| `start`（缺省）发送 | `HOOK REC_START` |
| `stop` 发送 | `HOOK REC_STOP` |
| `status` 发送 | `HOOK STATUS` |
| `test` 发送 | `HOOK TEST` |
| 等待回环应答 | `HOOK ACK` |
| 等待录制启动 | `HOOK REC_STARTED` / `HOOK REC_BUSY` / `HOOK REC_FAILED` |
| 等待完成快提示 | `HOOK REC_DONE <stem>` / `HOOK REC_FAILED` |
| 状态打印 | `HOOK STATUS <state> [stem]` |

## 5. 吊物端 POS 发送节奏（RADIO_NETWORK_SPEC §4）

吊物端 `gps_tcp_client.py`（`/home/orangepi/Desktop/radio/`）与录制守护同走 8886 通路，把吊物实时位置以 `POS` 行发给船与 UI。发送节奏已按《三端无线电防冲突契约》§4 修改并实机验证，与船侧 `USV`（1Hz、相位 0.0s）错峰，避免两帧 1Hz 报文在同一 GPS 秒边缘同时发射。

### 5.1 报文格式

```
POS <lat> <lon> <alt>
```

实际输出（`position_sender`）：`f"POS {lat:.7f} {lon:.7f} {alt:.1f}\n"`，1Hz。
示例：`POS 30.1234567 120.1234567 3.5`；缺失 GPS 数据以 `nan` 占位，不中断发送。

### 5.2 发送时序（GPS 秒边界 + 0.35s 相位）

- **正常路径**：GPS 时间可用且新鲜（fix 未过期 ≥2s 且 `utc_seconds` 有效）→ 对齐 GPS 秒边界 + 相位 `PHASE_OFFSET=0.35`，即目标发射点为 `ceil(utc/interval)*interval + 0.35`（GPS 秒 `N.350`），1Hz。船侧 `USV` 相位为 `0.0`（`phase_offset_s` 默认），两帧错峰 0.35s。
- **单次 sleep 上界 0.9s**（给系统留余量）；错过目标秒立即发送，不累积、不补发；同秒防重复发射（`last_sent_utc` 去重）。

### 5.3 无 GPS 秒退化

GPS 不可用（fix 过期 >2s 或 `utc_seconds` 无效）→ 退化为墙钟定时器 + 随机抖动：`wait(interval + uniform(-50, +50)ms)`，约 1Hz。命令可靠性由船侧 ACK + 重传兜底。

### 5.4 运行命令

```bash
python3 gps_tcp_client.py --serial /dev/ttyACM0 --baud 115200 --host <网关IP> --port 8886 --interval 1.0
```

- `--interval 1.0` 为默认值（GPS 秒边界 + 0.35s 相位错峰）。
- `<网关IP>` 指向吊物侧数传电台（实机 `192.168.0.127`，端口 `8886`）。

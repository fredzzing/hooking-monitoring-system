# hang_ctrl.py 使用说明（吊物轨迹录制）

吊物轨迹录制系统的 PC 端命令行控制工具。通过 LoRa 数传电台（TCP 8886）向吊物端 OrangePi 守护发送 HOOK 指令启动/停止录制，再经 SSH/SFTP 把录制的 CSV 数据回传到 PC。

本文件仅描述 `hang_ctrl.py` 这个独立 CLI 工具。船控（USV GCS 桌面应用）见 `README.usv-gcs.md`，湖试操作见 `LAKE_TRIAL_MANUAL.md`。

## 安装

依赖：Python 3 + `paramiko`。

```bash
pip install paramiko
```

`hang_ctrl.py` 自包含，只依赖 Python 标准库加 paramiko。没有 paramiko 也能运行 `--selftest` 和 `--mock-sftp`（离线测试），但真实 SSH 拉取必须装。

运行前提（三者缺一不可）：

1. 吊物端 LoRa 电台上电，网关 `192.168.0.127:8886` 可达（PC 与 Pi 双端共享该网段）。
2. 吊物端 Pi 守护 `radio-recorder.service` 处于 active（systemd 开机自启）。
3. SSH 通道可达：默认走 FRP `139.196.232.191:6006` 主路径，备选 WLAN `192.168.0.123:22`。

## 使用

```
python hang_ctrl.py start [--duration N]     # 开始录制（N 秒，缺省守护默认 1800s）
python hang_ctrl.py stop                     # 提前结束录制
python hang_ctrl.py status                   # 查询守护状态
python hang_ctrl.py pull --stem 20260825-112158   # 按 stem 拉取指定录制
python hang_ctrl.py pull --latest            # 拉取最新完成的录制
python hang_ctrl.py test                     # 无线电回环测试（HOOK TEST → HOOK ACK）
python hang_ctrl.py --selftest               # 离线自检（不触网）
```

示例：

```bash
# 30 秒实机录制
python hang_ctrl.py start --duration 30

# 录制中途提前停止
python hang_ctrl.py stop

# 拉取指定录制到本地 hangpoint 目录
python hang_ctrl.py pull --stem 20260825-112158

# 回环测试（验证电台链路）
python hang_ctrl.py test
```

`start` 和 `stop` 成功后会自动拉取：数据落到本地 `D:\Simulate\ROS2USV\hangpoint\<stem>.csv` 和 `<stem>.meta.json`（Pi 端原件保留）。`pull` 只拉 csv 与 meta.json 都齐全的完整对，不完整对直接跳过不落盘。

## 配置

所有参数都有默认值，可用命令行覆盖。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--host` / `--port` | `192.168.0.127` / `8886` | LoRa 网关地址（Ebyte E90-DTU，WLAN 段） |
| `--duration` | 缺省不发送，守护用自身默认 1800s | `start` 的录制时长；带 N 时发送 `HOOK REC_START N` |
| SSH 目标 | FRP 主 `root@139.196.232.191:6006` → WLAN 备 `root@192.168.0.123:22` | 依序尝试，首个连上的生效；`--ssh-target` 或环境变量 `HANG_PI_SSH_TARGET` 可覆盖（逗号分隔多目标） |
| `--user` | 环境变量 `HANG_PI_USER`，否则 `root` | SSH 用户名 |
| `--password` | 环境变量 `HANG_PI_PASS` | SSH 密码；缺省无密码会直接报错（凭据不硬编码） |
| `--runs-dir` | `/root/trajectory/runs` | Pi 上录制数据的绝对路径 |
| `--hangpoint-dir` | `..\hangpoint` | PC 本地落盘目录 |
| `--mock-sftp DIR` | 无 | 本地伪 SFTP 目录，离线测试用，不连真实 SSH |

凭据推荐用环境变量：

```bash
# PowerShell
$env:HANG_PI_USER = "root"
$env:HANG_PI_PASS = "你的密码"
```

## 操作提示

- **录制启动后吊钩保持静止**：守护会先做约 10 秒校准，再等 GPS 定位（最多约 30 秒），之后才正式采样。此期间吊钩移动会影响数据质量。
- **总耗时估算**：校准 10s + GPS 等待（最多 30s）+ 录制时长 + SSH 拉取时间。20MB 级大文件经 FRP 拉取约需 75 到 225 秒，请耐心等待，勿提前中断。
- **无线电是半双工**：指令可能丢，但 PC 端有 SSH 轮询兜底。完成判定以 SSH 侧 meta.json 出现为准，`HOOK REC_DONE` 只是快提示，不代表权威结果。
- `stop` 只发一条 `HOOK REC_STOP`，随后转 SSH 轮询等待录制结束并拉取；中途任何一步失败会以非零退出码报错，不会留下半截文件。

## 已知限制（如实说明）

- **无线电链路不可靠**：半双工，命令可能重发或丢失。PC 端不能保证每条无线电消息都被对端收到，完成判定依赖 SSH 轮询（非无线电）。`test` 的 `HOOK ACK` 有 60 秒超时，超时即失败。
- **单次录制约 20MB**（30 分钟、约 50Hz 采样）。长时间录制前请确认 Pi 磁盘余量。
- **Pi 无硬件 RTC**：时间靠 NTP 校时，断网时时钟可能漂移（曾实测偏差 +34s）。建议在 Pi 上配置 NTP/chrony，让录制文件名时间戳更准确。
- **GPS 会挂起**：GPS 无 fix 超过 30 秒时录制直接以 `REC_FAILED` 结束（守护 exit 2，绝不发 `REC_DONE`）。GPS 挂起可通过 USB 重枚举恢复。
- **stop_reason 规范化**：Pi 落盘写的是 `signal(15)`，PC 端拉取时统一规范化为 `signal`。`duration` 保持原样。
- **无内置大文件超时**：SFTP 拉取大文件时外部调用脚本需给足时间（建议 ≥300s）。

## 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `REC_FAILED` | GPS 无 fix（超 30s）、录制异常 | 检查 GPS 信号，必要时 USB 重枚举；重试 |
| `REC_BUSY` | 已有录制在运行（重复 start） | 等当前录制结束，或先 `stop` |
| `test` 收不到 `HOOK ACK`（60s 超时） | 电台未上电、网关地址错、对端无监听 | 确认电台与网关 `192.168.0.127:8886` 可达 |
| `SSH 连接失败` / SFTP 认证失败 | FRP/WLAN 都不可达、凭据错 | 核对 `--user/--password` 或环境变量；检查网络路径 |
| start/stop 等待超时 | 无线电丢指令、SSH 轮询也失败 | 重试命令；确认守护 active 且 SSH 可达 |
| 拉取报「不完整对」 | 录制中断，csv 与 meta 不全 | 等录制正常完成再 pull；已损坏的不落盘属预期 |

## 数据格式

拉取结果位于 `hangpoint\` 目录：

- `<stem>.csv`：采样数据（约 50Hz）。stem 格式 `YYYYMMDD-HHMMSS`（Pi 本地时间）。
- `<stem>.meta.json`：元数据，含 `rows`（行数）与 `stop_reason`（`duration` 或 `signal`）。

meta.json 校验通过才会落盘：必须 JSON 可解析、`rows > 0`、`stop_reason` 合法。任何一条不满足都会报错且不落盘。

## 离线测试

无电台、无 SSH 时可用 `--mock-sftp` 验证拉取逻辑：

```bash
python hang_ctrl.py pull --stem 20260825-112158 --mock-sftp D:\fake-pi
python hang_ctrl.py --selftest
```

`--selftest` 不触网，覆盖消息构造/解析、stem 与 meta 校验、原子改名、不完整对拒绝等逻辑，退出码 0 表示全部通过。

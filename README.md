# 吊物轨迹监测系统 (hook-monitoring-system)

吊物轨迹监测系统：Orange Pi 对吊物进行 GPS+IMU 融合轨迹录制，PC 经 LoRa 无线电网关远程控制录制并通过 SSH 回传数据。

## 目录结构

```
├── pc/                        # PC 端
│   ├── hang_ctrl.py           # 无线电控制 + SSH 数据回传 CLI（自包含，仅 stdlib+paramiko）
│   ├── README.md              # PC 端 CLI 快速说明
│   ├── 使用说明.md            # 详细使用说明
│   ├── HOOK_PROTOCOL.md       # HOOK 无线电控制协议
│   └── CSV列说明.md           # 录制 CSV 字段说明
└── orange_pi/                 # 香橙派端
    ├── trajectory/            # 轨迹录制（板载 /root/trajectory/）
    │   ├── recorder.py        # 录制主程序（GPS+IMU → CSV+meta）
    │   ├── radio_recorder.py  # 无线电守护（HOOK 指令 → 驱动录制，systemd 常驻）
    │   ├── gps_reader.py      # GPS NMEA 读取
    │   ├── imu_reader.py      # IMU 读取
    │   ├── attitude.py        # 姿态解算
    │   ├── geo.py             # 地理坐标工具
    │   ├── ekf.py             # 扩展卡尔曼滤波融合
    │   ├── validate_csv.py    # CSV 数据校验
    │   ├── analyze.py         # 数据分析
    │   ├── CSV_SCHEMA.md      # CSV 字段规范
    │   ├── README.md          # 轨迹录制模块说明
    │   └── TEST-PROCEDURE.md  # 板上测试流程
    └── radio/
        └── gps_tcp_client.py  # 吊物 POS 位置转发（GPS 秒边界 +0.35s 错峰）
```

## 三端架构

- **吊物端**：GPS+IMU 传感器 + 无线电链路；位置 POS 报文 1Hz 上报（GPS 秒边界 +0.35s 与船控 USV 错峰）
- **香橙派端（Orange Pi）**：轨迹录制（GPS+IMU 经 EKF 融合）；radio_recorder 守护监听 HOOK 控制指令驱动录制/停止，录制完成后经无线电回执 REC_DONE/REC_FAILED；gps_tcp_client 转发吊物位置
- **PC 端**：hang_ctrl.py 经 LoRa 无线电网关（192.168.0.127:8886）发送 HOOK 指令（start/stop/status/test），经 SSH 拉取录制数据（主 FRP 139.196.232.191:6006，备 WLAN 192.168.0.123:22）

## 关键文档

- PC CLI 快速说明：[pc/README.md](pc/README.md)
- 详细使用说明：[pc/使用说明.md](pc/使用说明.md)
- 无线电控制协议：[pc/HOOK_PROTOCOL.md](pc/HOOK_PROTOCOL.md)
- CSV 字段说明：[pc/CSV列说明.md](pc/CSV列说明.md)
- 板上录制模块说明：[orange_pi/trajectory/README.md](orange_pi/trajectory/README.md)
- 板上测试流程：[orange_pi/trajectory/TEST-PROCEDURE.md](orange_pi/trajectory/TEST-PROCEDURE.md)
- CSV 字段规范：[orange_pi/trajectory/CSV_SCHEMA.md](orange_pi/trajectory/CSV_SCHEMA.md)

## 运行前提

- **PC**：Windows + Python 3.11（依赖 paramiko 5.0.0，其余仅 stdlib）；需可达 LoRa 网关 192.168.0.127:8886 与 Pi SSH（FRP 主 / WLAN 备）
- **香橙派**：Python 3（仅 stdlib）；radio-recorder 已配置 systemd 开机自启；录制需 GPS fix
- **凭据**：仅通过 --user/--password 参数或环境变量传入；本仓库不含任何凭据

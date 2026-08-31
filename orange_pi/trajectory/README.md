# 吊钩轨迹记录系统使用说明（GPS + IMU 融合）

> 平台：Orange Pi 3B（aarch64）+ u-blox GPS（1 Hz）+ WitMotion WT901SDCL（USB 串口，50 Hz）
> 目录：`/root/trajectory`（所有模块、脚本、测试数据）
> 坐标系：ENU（东北上），原点为录制开始时 GPS survey-in 中位数（见 meta.json 的 origin）
> 本说明所有参数均来自实机实测（2026-08-24），不夸大精度，未实现的功能不承诺。

---

## 1. 安装

系统为 Python 3.10.12。模块均为纯 Python 脚本，除 numpy 外无新增依赖。

```bash
# numpy（EKF/姿态模块可选加速；recorder 启动时自动检测）
pip3 install numpy
# 若 pip 在 aarch64 上源码编译卡死，改用 apt（实测秒装，版本 1.21.5）
apt-get install -y python3-numpy
```

已有依赖（无需安装）：

- **pyserial**（import 名为 `serial`，实测 3.5）：GPS 串口读取 + WitMotion IMU 串口读取

### 模块清单与职责

| 文件 | 职责 |
|---|---|
| `gps_reader.py` | GPS 读取。pyserial + NMEA 解析，`/dev/ttyACM0` @ 115200，1 Hz；门控 fix≥1 且 HDOP<2.0 |
| `imu_reader.py` | IMU 读取。WitMotion WT901SDCL 串口读取（pyserial, 0x55 协议, 50Hz, ±16g/±2000dps, 自动探测+配置, 启动 10s 校准, 串口积压时间戳回推） |
| `attitude.py` | 姿态估计。Madgwick 自适应互补滤波，roll/pitch/yaw（ZYX）；无磁力计，yaw 为纯陀螺积分 |
| `ekf.py` | 9 状态扩展卡尔曼滤波（ENU 位置/速度 + 加速度零偏），GPS 位置/速度 + ZUPT 更新，chi-square 野值拒绝 |
| `geo.py` | WGS84 ↔ ENU 坐标转换；survey-in 原点计算 |
| `recorder.py` | 主控。3 线程（GPS / IMU / 融合写盘），28 列 CSV + `.meta.json` 输出 |
| `validate_csv.py` | CSV 结构校验（28 列/行数/采样率/单调性/无 NaN/zupt 枚举） |
| `analyze.py` | 物理量断言（5 个指标，见第 4 节） |

---

## 2. 校准与运行

### 启动前必读

**启动后吊钩必须保持完全静止**。程序自动完成以下流程后才会开始正式记录：

1. IMU 校准 10 s（静止，采集陀螺/加速度零偏）
2. 等待首个 GPS fix（最长 30 s，冷启动可能较慢）
3. GPS survey-in 15 s（≥10 个合格 fix，取逐分量中位数作为原点）
4. 终端输出 `recording ->` 后才正式开始记录

全程约 25~55 s。**在出现 `recording ->` 之前不要移动吊钩**，否则原点标定与零偏校准会失真。

### 默认运行（30 分钟）

```bash
cd /root/trajectory
python3 recorder.py --duration 1800 --out runs/
```

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--duration` | 1800 | 计划录制时长（秒），即默认 30 分钟 |
| `--rate` | 50 | 标称 IMU 采样率（50Hz） |
| `--out` | `/root/trajectory/runs/` | 输出目录（CSV + meta.json） |

### 停止

- **Ctrl+C**：优雅停止（SIGINT，约 2 s 内干净退出，自动做 CSV 尾部完整性检查）。实测 10 s 中断测试通过。
- 正常跑满 `--duration` 也会自动结束。
- 不建议 kill -9：会跳过尾部完整性检查与 meta 收尾。

### 串口缓冲与零丢弃说明

- 校准完成后 IMU 线程**立即启动并持续读取**，首个 fix 等待与 survey-in 期间持续 drain 串口缓冲（内核缓冲约 4 KB，50Hz × 22 字节 ≈ 1.1 KB/s，正常不丢数），防止缓冲积压导致读取时间戳延迟。
- 正常录制 `meta.json` 中 `dropped.imu` / `dropped.gps` 应为 0。实测 60 s 与 300 s 长稳录制均为 0。
- 串口读取偶发停顿（GPS 串口并发时单次读取 20~43 ms）不丢样本：程序按串口 RX 积压字节数把读取时间戳回推到生产时刻（见 meta `backdated` 字段）。

---

## 3. 输出

每次录制在 `runs/` 下生成一对文件（同名）：

- `runs/<时间戳>.csv`：28 列轨迹数据
- `runs/<时间戳>.meta.json`：本次录制元数据

### CSV 28 列

| # | 列名 | 单位 | 说明 |
|---|---|---|---|
| 0 | t_mono | s | 单调时间戳 |
| 1-3 | gps_lat / gps_lon / gps_alt | deg / m | WGS84（coast 行为 NaN） |
| 4 | gps_fix | - | 0=无 fix，1/2=有 fix |
| 5 | gps_hdop | - | 水平精度因子（coast 为 NaN） |
| 6-7 | gps_speed_ms / gps_course_deg | m/s / deg | 地速/航向（coast 为 NaN） |
| 8-10 | imu_ax / imu_ay / imu_az | m/s² | 加速度（体坐标，已扣零偏） |
| 11-13 | imu_gx / imu_gy / imu_gz | rad/s | 角速度（体坐标，已扣零偏） |
| 14 | imu_sat | - | 卫星数（当前 GPS 模块无此字段，恒 0） |
| 15-17 | att_roll / att_pitch / att_yaw | rad | **安装相对**姿态（ZYX，见已知限制） |
| 18-20 | fused_n / fused_e / fused_u | m | EKF 位置（ENU，相对原点） |
| 21-23 | fused_vn / fused_ve / fused_vu | m/s | EKF 速度（ENU） |
| 24-26 | fused_lat / fused_lon / fused_alt | deg / m | 融合后 WGS84 坐标 |
| 27 | zupt | - | 0=运动，1=ZUPT 保持，2=coast（GPS 失效，纯 IMU 积分） |

完整规范见同目录 `CSV_SCHEMA.md`（冻结版本，三处脚本共用）。

### .meta.json 关键字段

| 字段 | 说明 |
|---|---|
| `origin` | 原点（survey-in 逐分量中位数，lat/lon/alt） |
| `survey_in` | `{n_fixes, duration_s, spread_h_m, origin_method}`：fix 数、窗口时长、水平散布、中位数法 |
| `gyro_bias` / `accel_bias` | 启动 10 s 校准的陀螺/加速度零偏（已扣除） |
| `mount_gravity` | 安装重力向量（用于姿态预对齐与安装相对姿态还原） |
| `calibration` | `{n_samples, duration_s}`：校准样本数与时长 |
| `sample_rate` / `duration` | 实测采样率（Hz）与实际跨度（s），会覆盖 validate 的 CLI 默认 |
| `planned_duration` / `nominal_rate` | 计划时长与标称采样率 |
| `imu` / `gps` | 硬件配置（型号、接口、量程、输出率；串口、波特率、fix/HDOP 门限） |
| `start_time` / `stop_time` | 起止时刻（板时区 UTC，ISO 8601） |
| `rows` | 实际行数 |
| `dropped` | `{imu, gps}` 丢弃计数（0=零丢弃） |
| `coast` | `{rows, seconds, gps_rejected_events}`：GPS 失效段统计 |
| `zupt` | `{active_rows}`：ZUPT 生效行数 |
| `backdated` | `{samples, max_backdate_ms, dt_ema_ms}`：串口积压时间戳回推统计 |
| `rate_warnings` / `numpy` / `stop_reason` | 采样率告警数、numpy 是否可用、结束原因（如 signal(15)=Ctrl+C） |

---

## 4. 分析用法

```bash
cd /root/trajectory
CSV=$(ls -t runs/*.csv | head -1)

# 结构校验（可选，录制自检用）
python3 validate_csv.py "$CSV" --rate 50 --tolerance_pct 1 ; echo "VALIDATE_EXIT=$?"

# 统计报告：丢样本数、coast 段、fused_* 范围
python3 analyze.py "$CSV" --metric report

# 静态漂移断言：水平<5m 垂直<8m 末速<0.1m/s
python3 analyze.py "$CSV" --metric static_drift

# 姿态稳定性断言：静止段 mean|roll|、mean|pitch| < 1°（安装相对姿态）
python3 analyze.py "$CSV" --metric attitude_static

# 摆动幅值（informational，恒 PASS，用于 3 次复现对比）
python3 analyze.py "$CSV" --metric swing_amplitude

# 升降位移断言：|delta − expect_delta| ≤ tolerance（默认 1.5 m）
python3 analyze.py "$CSV" --metric hoist_delta --expect_delta D --tolerance 1.5
```

`--metric` 可选值：`report` / `static_drift` / `attitude_static` / `swing_amplitude` / `hoist_delta`。

**断言语义**：越界即如实报 FAIL 并返回退出码 1，无好样本挑选、无后处理。

---

## 5. 物理测试流程

完整脚本化测试流程见同目录 **`TEST-PROCEDURE.md`**。所有判定均为脚本断言（退出码 0=PASS / 1=FAIL），**不是目测**。三个场景：

### 5.1 升降测试（垂直位移准确性）

1. 吊钩静止启动录制（等待 `recording ->`），录 120 s
2. 以 **≥0.5 m/s**（建议约 1 m/s）稳定速度升降已知距离 D，到位后静止 15 s
3. 结束后运行：
   ```bash
   python3 analyze.py $CSV --metric hoist_delta --expect_delta D --tolerance 1.5
   ```
   要求 `|delta − D| ≤ 1.5 m`。建议 D ≥ 5 m（远大于 GPS 高程噪声 2~3 m）。

### 5.2 摆动测试（摆动可复现性）

静止 10 s → 手动拨动吊钩自由摆动自然衰减 20~30 s → 完全静止 10 s，**重复 3 次**，每次独立录制，对比 `swing_amplitude` 的 accel 幅值/rms/周期/角速度峰值/水平速度峰值（±30% 内视为可复现）。该指标为 informational，恒 PASS，用于横向对比而非单次判定。

### 5.3 静止自检（推荐每次测试前执行）

```bash
python3 validate_csv.py $CSV --rate 50 --tolerance_pct 1   # 结构
python3 analyze.py $CSV --metric static_drift              # 漂移门限
python3 analyze.py $CSV --metric attitude_static           # 姿态门限
```

任一 FAIL 说明系统异常，先排查再测（常见原因见第 7 节）。

---

## 6. 已知限制（实机实测，如实说明）

- **垂直精度约 1-2m**：无气压计，垂直方向依赖 GPS 高程与 IMU 积分，量级为米级。升降断言用 1.5 m 容差并建议升距 ≥5 m。
- **yaw 静止/纯摆动会漂移**：无磁力计，yaw 为纯陀螺积分，长时间静止或纯摆动（加速度无法观测量测方位）时缓慢漂移。roll/pitch 由加速度+陀螺融合，静止时稳定（实测 0.05~0.08°）。
- **绝对坐标受 GPS 限制**：标称 1~3 m 水平 / 3~6 m 垂直。**本板接收机在多径环境下静止位置会在约 10 m 包络内游走，相邻 fix 可跳 3~4 m**，HDOP≈1 也不完全可信。系统通过 ZUPT 位置保持抑制其对相对轨迹的影响，但绝对位置的长期稳定性由 GPS 决定。
- **相对轨迹形状为 cm~dm 级**：位移形状、升降方向、摆动幅度等相对量可靠；绝对位置（fused_lat/lon/alt 与原点偏移）不承诺厘米级。
- **60 s 静态漂移实测 1.7~2.8 m（水平）**：ZUPT 生效下融合位置的静态漂移量级（垂直 0.56~1.95 m）。这是 GPS 游走渗入保持后的实际表现，门限为水平 <5 m / 垂直 <8 m。
- **姿态列为安装相对姿态**：本板安装倾角约 153°，绝对姿态由 meta `mount_gravity` 还原（`q_abs = q_mount ⊗ q_rel`）。CSV 中 att_* 静止时约为 0°，直接反映吊钩运动。
- **采样率为实测 ~50.6 Hz**（标称 50Hz），非固定精确 50 Hz；时间戳经回推修正，均用于融合与校验。

---

## 7. 常见问题（FAQ）

**Q1. GPS 无 fix，程序卡在等待。**
遮挡（室内/高墙）或冷启动都会导致无 fix。确认天线朝天、`/dev/ttyACM0` 存在、HDOP < 2.0。冷启动最长等 30 s；仍无则检查串口接线与供电。

**Q2. IMU 串口读取卡顿 / 数据错乱。**
系统已内置启动期持续 drain 与防积压处理，正常不会发生。若 `dropped.imu > 0` 或 a_mag 异常（静止时远小于 9.81），检查 USB 数据线、供电，并避免录制期间其他进程并发占用串口设备。

**Q3. 采样率不足 / validate 报 rate FAIL。**
实测 ~50.6 Hz。若均值偏离 50 ±2 Hz 或 p95 jitter ≥5 ms，多为 USB 串口占用或供电不足；查看 meta `backdated.max_backdate_ms`，停顿过大说明串口读取受干扰。校准阶段重跑一次可复现即可定位。

**Q4. HDOP 劣化怎么办。**
HDOP ≥ 2.0 时该 GPS 观测被门控拒绝，行进入 coast（zupt=2），轨迹退化为纯 IMU 积分，位置随积分时间漂移。改善天线环境后重录；分析时注意 coast 段精度下降。

**Q5. 30 分钟长录制要注意什么。**
默认 `--duration 1800`。系统已验证 300 s 无累积漂移、零丢弃；长时间录制建议定期查看终端 status（dropped 应保持 0）。30 分钟测试保持吊钩静止同样有效（ZUPT 生效）。

**Q6. 分析脚本报 FAIL 是脚本坏了吗？**
不是。analyze.py 越界如实报 FAIL（退出码 1），这是设计行为。先看具体数字：static_drift 水平 <5 m、垂直 <8 m、末速 <0.1 m/s；attitude_static <1°；hoist_delta ≤1.5 m。若多次 FAIL 先做 5.3 静止自检排查系统。

---

## 附：测试环境实机参数（2026-08-24 记录）

| 项目 | 实测值 |
|---|---|
| GPS 端口/波特率 | /dev/ttyACM0 @ 115200，1 Hz，HDOP 0.83~1.59，12 星 |
| IMU | WitMotion WT901SDCL，USB 串口，±2000 dps / ±16 g，50 Hz |
| 采样率 | ~50.6 Hz（标称 50Hz） |
| 60 s 静态 validate | 8/8 PASS，dropped=0 |
| 静态漂移 | 水平 1.69~2.85 m / 垂直 0.56~1.95 m，末速 0.000 m/s |
| 姿态静态 | mean|roll|/|pitch| 0.05~0.08°（安装相对） |
| 300 s 长稳 | 15184 行，50.61 Hz，dropped 0 |

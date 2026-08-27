# CSV & Metadata Schema (FROZEN, project-wide)

> Source of truth for the trajectory recording format. Any column / field change
> MUST be synced across: `validate_csv.py` (validator), `recorder.py` (writer),
> `analyze.py` (reader). See `validate_csv.py` docstring for the authoritative spec.

## CSV columns (28, frozen)

| # | column        | unit        | description                              |
|---|---------------|-------------|------------------------------------------|
| 0 | t_mono        | s           | time.monotonic() timestamp, strictly increasing |
| 1 | gps_lat       | deg         | WGS84 latitude (NaN if coast)            |
| 2 | gps_lon       | deg         | WGS84 longitude (NaN if coast)           |
| 3 | gps_alt       | m           | GPS altitude AMSL (NaN if coast)         |
| 4 | gps_fix       | int         | fix quality 0=no fix, 1/2=fix           |
| 5 | gps_hdop       | -           | horizontal dilution of precision (NaN if coast) |
| 6 | gps_speed_ms  | m/s         | ground speed (NaN if coast)              |
| 7 | gps_course_deg| deg         | ground course (NaN if coast)             |
| 8 | imu_ax        | m/s^2       | accel X body                             |
| 9 | imu_ay        | m/s^2       | accel Y body                             |
|10 | imu_az        | m/s^2       | accel Z body                             |
|11 | imu_gx        | rad/s       | gyro X body                              |
|12 | imu_gy        | rad/s       | gyro Y body                              |
|13 | imu_gz        | rad/s       | gyro Z body                              |
|14 | imu_sat       | int         | satellite count                          |
|15 | att_roll      | rad         | roll  MOUNT-RELATIVE (Madgwick, ZYX)     |
|16 | att_pitch     | rad         | pitch MOUNT-RELATIVE (Madgwick, ZYX)     |
|17 | att_yaw       | rad         | yaw   MOUNT-RELATIVE (Madgwick, ZYX)     |
|18 | fused_n       | m           | EKF position north (ENU)                 |
|19 | fused_e       | m           | EKF position east (ENU)                  |
|20 | fused_u       | m           | EKF position up (ENU)                    |
|21 | fused_vn      | m/s         | EKF velocity north                       |
|22 | fused_ve      | m/s         | EKF velocity east                        |
|23 | fused_vu      | m/s         | EKF velocity up                          |
|24 | fused_lat     | deg         | fused position latitude                  |
|25 | fused_lon     | deg         | fused position longitude                 |
|26 | fused_alt     | m           | fused position altitude                  |
|27 | zupt          | int         | 0=moving, 1=ZUPT active, 2=coast (GPS invalid) |

Rules:
- `t_mono` strictly increasing; nominal sample rate 50 Hz.
- **NaN policy**: only `gps_lat/lon/alt/hdop/speed_ms/course_deg` may be empty/NaN,
  and only on rows where `zupt == 2` (coast). All other cells must be finite numbers.
- `zupt` must be integer in {0,1,2}.

### att_* 参考系（Task 9 变更，2026-08-24）

`att_roll/att_pitch/att_yaw` 是吊钩组件相对**标定安装姿态**的姿态
（q_rel = q_mount^-1 x q_att；q_mount 将体轴重力方向映射到地轴 +z，零 yaw）。
板子以大倾角固定安装（实测 mount_gravity ≈ 0.514, 0.390, -0.764，绝对
roll ≈ 153°），绝对姿态静止时恒为 ~153°，`attitude_static` 指标
（mean|roll|/|pitch| < 1°）在本硬件上无法满足。安装相对姿态静止时读出
~0°，真实反映吊钩运动。板子绝对姿态可由 meta 的 `mount_gravity`（q_mount）
还原。

## Sibling `.meta.json` (same directory, stem = CSV filename)

```json
{
  "origin":        {"lat": 31.2304, "lon": 121.4737, "alt": 50.0},
  "survey_in":     {"n_fixes": 16, "duration_s": 15.0, "spread_h_m": 3.0,
                    "origin_method": "median"},
  "gyro_bias":     {"x": 0.0, "y": 0.0, "z": 0.0},
  "accel_bias":    {"x": 0.0, "y": 0.0, "z": 0.0},
  "mount_gravity": {"x": 0.0, "y": 0.0, "z": -9.8},
  "sample_rate":   50.0,
  "duration":      1800.0,
  "planned_duration": 1800.0,
  "multipath_rejected": 0,
  "nominal_rate":  50.0,
  "imu":           {"accel_range": "4g", "gyro_range": "500dps",
                    "dlpf_cfg": 4, "i2c_bus": 2, "i2c_addr": 104},
  "gps":           {"port": "/dev/ttyACM0", "baud": 115200,
                    "fix_min": 1, "hdop_max": 2.0},
  "start_time":    "2026-08-24T12:00:00.000+08:00"
}
```

- `origin`: **survey-in 中位值**（Task 9 变更；原为：首个合格 GPS fix）。
  接收机冷启动后前 ~10-20 s 解算仍在漂移（板上实测：~8 s 内东移 +8 m /
  上移 +3.6 m 后才稳定），首个 fix 可能是早期收敛野值。现为 15 s
  survey-in 窗口内 >= 10 个合格 fix 的逐分量中位数（`survey_in`）。
  录制中途从不重算。
- `multipath_rejected`: 静止位置保持期（hold）被多径守卫拒绝的水平跳变
  fix 数（F3 修复新增字段）。判据：水平 innovation √(ΔN²+ΔE²) > 2.5 m 且
  hold 激活 → 水平通道跳过该 fix（位置不跟随），高度通道按保持权重照常
  更新、速度观测不受影响；仅当连续 >= 3 个相互 < 2.5 m 的一致 fix 出现才
  恢复正常位置权重。配合：hold 期位置 R 缩放 25×（ZUPT_R_SCALE）、垂直
  通道额外 ×100 降权（GPS 高度本就弱）、hold 期零速约束（速度通道，独立
  于冻结的 0.5 m/s ZUPT 门）、hold 速度门取最近 5 个 fix 速度中位数
  （抗静态速度噪声尖峰），共同抑制夜间多径窗口的静止位置游走。
- `gyro_bias` / `accel_bias` / `mount_gravity`: startup 10 s static calibration.
- `sample_rate` / `duration`: override CLI defaults in `validate_csv.py` when present.
- `start_time`: ISO 8601 with timezone.

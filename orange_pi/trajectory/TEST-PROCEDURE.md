# 吊钩轨迹物理测试脚本（TEST-PROCEDURE）

> 适用：Orange Pi 3B + u-blox GPS（1Hz）+ MPU6050（50Hz）融合系统
> 部署目录：`/root/trajectory`（recorder.py / analyze.py / validate_csv.py）
> **所有判定均为脚本化断言（退出码 0=PASS、1=FAIL、2=用法/IO 错误），不做目测。**

---

## 0. 测试前准备（每次录制必读）

1. 确认 GPS 天线与 IMU 接线完好，板上电，GPS 已定位（`/dev/ttyACM0` 存在）。
2. **启动后保持吊钩完全静止**：录制启动流程为
   「IMU 校准 10s → 等待首个 GPS fix（最长 30s）→ GPS survey-in 15s（原点收敛）」，
   全程约 25~55 秒。**只有终端出现 `recording ->` 之后才能开始动作**；
   在此之前吊钩必须静止，否则原点标定与零偏校准作废。
3. 用卷尺 / 激光测距预先测量升降距离 D（米），建议 5~10 m。
4. 所有测试在开阔天空下执行（GPS 多径会显著降低精度；本板 GPS 静止时
   位置在 ~10 m 包络内漂移、相邻 fix 可达 3~4 m 跳变，hdop≈1 不可全信）。

---

## 1. 升降测试（垂直位移精度断言）

**目的**：验证吊钩提升已知距离 D 时，融合垂直位移 `fused_u` 与真值的偏差 ≤ 1.5 m。

**步骤**：

```bash
cd /root/trajectory

# 1) 吊钩静止，启动录制（录制 120 s）
python3 recorder.py --duration 120 --out runs/

# 2) 等待终端出现 "recording ->"（约 25~55 s，期间吊钩保持静止）

# 3) 以 >= 0.5 m/s（推荐 ~1 m/s）的稳定速度将吊钩匀速提升 D 米
#    （D 为卷尺/激光测距实测值），到位后保持静止 15 s 以上，
#    等待录制自动结束（或 Ctrl-C 提前结束，CSV 仍完整可用）

# 4) 录制结束后，对最新 CSV 执行脚本化断言（把 D 换成实测米数）：
CSV=$(ls -t runs/*.csv | head -1)
python3 analyze.py $CSV --metric hoist_delta --expect_delta D --tolerance 1.5
echo "HOIST_EXIT=$?"
```

**判定**（脚本输出，非目测）：
- `PASS: hoist_delta delta=...m (expect_delta=D tolerance=1.5m, err=...)` 且退出码 0 → 通过；
- `FAIL: ...` 或退出码 1 → 不通过，如实记录 delta 与 err。

**注意事项**：
- 提升速度必须 ≥ 0.5 m/s：低于该速度时 GPS 速度噪声会把系统误判为静止
  （ZUPT/位置保持生效），垂直位移会被冻结，hoist_delta 将低估。
- 提升过程中避免遮挡 GPS 天线；到位后静止期间系统进入位置保持，等待
  录制结束即可。

---

## 2. 摆动测试（摆动可复现性）

**目的**：验证「静止 → 摆动 → 静止」的摆动幅度特征可复现。
`swing_amplitude` 为 informational 指标（恒 PASS，无硬门限），
以脚本输出的物理量（加速度幅值 / 周期 / 角速度峰值 / 水平速度峰值）做
三次重复对比，量级一致即视为可复现。

**步骤**（重复 3 次，每次一段新录制）：

```bash
cd /root/trajectory

# 1) 吊钩静止，启动录制
python3 recorder.py --duration 90 --out runs/

# 2) 等待 "recording ->" 后：静止 10 s -> 手动推摆吊钩，让其自由摆动
#    并自然衰减 20~30 s -> 完全静止 10 s -> 等待录制结束

# 3) 对最新 CSV 输出摆动物理量（informational，恒 PASS）：
CSV=$(ls -t runs/*.csv | head -1)
python3 analyze.py $CSV --metric swing_amplitude
```

**复现性判定**（脚本输出对比，非目测）：
- 3 次录制的 `accel_amp / accel_rms / period_est / gyro_peak /
  horiz_speed_peak` 量级一致（允许 ±30% 波动，手动推摆力度差异所致）→ 可复现；
- 若某次数值量级与其他两次相差数倍以上 → 记为该次测试异常，排查
  推摆方式或 GPS/IMU 状态后重测。

---

## 3. 静止自检（安装/系统健康检查，可选但推荐每次测试前执行）

```bash
cd /root/trajectory
python3 recorder.py --duration 60 --out runs/    # 全程静止，等待 recording -> 后仍保持静止
CSV=$(ls -t runs/*.csv | head -1)
python3 validate_csv.py $CSV --rate 50 --tolerance_pct 1 ; echo "VALIDATE_EXIT=$?"
python3 analyze.py $CSV --metric static_drift ; echo "DRIFT_EXIT=$?"
python3 analyze.py $CSV --metric attitude_static ; echo "ATT_EXIT=$?"
```

**判定**（全部退出码 0）：
- validate：CSV 完整性（行数/采样率/时序/无 NaN）
- static_drift：水平漂移 < 5 m、垂直 < 8 m、末速 < 0.1 m/s
- attitude_static：静止段 mean|roll|、mean|pitch| < 1°（相对安装姿态）

任一 FAIL → 系统异常（参考 learnings：GPS 多径 / FIFO 溢出 / 姿态收敛），
排除后重测。

---

## 4. 重要说明（勿误读）

- **60s 静态测试通过 ≠ 实机运动精度保证**。静止精度由 ZUPT 位置保持
  达成；运动精度（升降/摆动）必须按本脚本第 1/2 节实测。
- 垂直轴最弱：GPS 高程噪声 ±2~3 m，hoist_delta 容差 1.5 m 依赖升降
  距离足够大（推荐 ≥ 5 m）与稳定的 GPS 高程收敛。
- 30 分钟长录制：`python3 recorder.py --duration 1800 --out runs/`，
  系统已验证 300 s 无丢样本累积；长时间录制请定期查看终端 status 行
  （dropped 计数应保持 0）。
- CSV 每行 28 列冻结格式见 `/root/trajectory/CSV_SCHEMA.md`；
  姿态列为**安装相对姿态**（板子倾斜安装，绝对 roll ≈ 153° 属正常）。

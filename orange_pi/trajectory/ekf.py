#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ekf.py — 9 状态松耦合扩展卡尔曼滤波器（位置/速度/加速度零偏 + ZUPT + 守卫）
          （gps-imu-fusion 项目，全项目数学核心）

用途:
  - 融合 IMU 导航系加速度（50Hz 预测）与 GPS ENU 位置（1Hz 更新）
  - 静止时 ZUPT 零速约束抑制漂移
  - 野值创新门限 + 协方差钳制 + NaN/Inf 守卫

状态（9 维，ENU）:
    x = [p_n, p_e, p_u, v_n, v_e, v_u, b_an, b_ae, b_au]
        p_* : 位置 [m]（ENU，n 北 / e 东 / u 上，z 向上）
        v_* : 速度 [m/s]
        b_a*: 导航系加速度零偏 [m/s²]（随机游走，Q 驱动）

坐标/符号约定（重要，与全项目一致）:
  - ENU 右手系，z 向上。重力方向为 -z，即 [0, 0, -9.80665]。
  - 本模块接收的 a_nav 是「已扣除重力的导航系加速度」:
        a_nav = R(attitude)·(a_body − b_a_body) − [0, 0, g]
    重力符号与旋转由调用方（recorder.py 结合 attitude.py）保证。
    本模块不重复扣重力 —— 传入 a_nav 意味着调用方已完成补偿。
  - 静止时 a_nav ≈ (0, 0, 0)；自由落体 a_nav ≈ (0, 0, -g)。

预测模型（松耦合、线性化）:
    p' = v
    v' = a_nav − b_a
    b_a' = 0（随机游走，由过程噪声 Q 驱动）
  离散化（半隐式欧拉，p 用旧 v）:
    p += v*dt
    v += (a_nav − b_a)*dt
  F = ∂(离散状态转移)/∂x，含 dp/dv = dt、dv/db_a = −dt。

更新模型:
  - update_gps(n, e, u, r_scale=1.0, hold=False): H 取 p 分量，
    R = diag([3m]², [3m]², [4m]²)（z 轴最弱）× r_scale。
    hold=True（静止位置保持）时：垂直通道额外 ×MP_V_HOLD_DOWNWEIGHT
    降权（GPS 高度本就弱）；水平多径守卫（以 hold 锚点为参照，见 MP_*
    常数）：fix 与锚点水平距离 > 2.5 m 判多径候选 → 水平通道跳过该 fix、
    高度通道按保持权重照常更新；连续 ≥3 个相互 <2.5 m 的一致 fix 后退出
    锁定并按锚点重新判定。被拒数计入 self.multipath_rejected。
  - update_vel(vn, ve, vu): 可选 GPS 速度观测，H 取 v 分量
  - zupt(): 零速伪观测 v = 0，H 取 v 分量，R 紧（默认 0.05 m/s）

守卫:
  - 创新门限: 归一化创新平方 d² = yᵀ S⁻¹ y 与 chi-square 95% 分位比较（3-DOF=7.815），
    超过即判野值拒绝（不更新）。
  - 协方差对角钳制: [1e-6, 1e8]，防下溢/上溢。
  - NaN/Inf 检测: 状态或协方差出现非有限值 → reset() 到初始状态。
    预测输入非法（dt<=0 / a_nav 含 NaN）→ reset()；观测含 NaN → 忽略该次观测。

注意: 本模块纯 stdlib 实现（无 numpy），矩阵为 9x9 小规模，直接用 list 运算。
"""

import math
import sys

# ---- 常数 ----
GRAVITY = 9.80665  # 标准重力 [m/s²]（文档约定用，模块内不参与计算）

# chi-square 95% 分位表（自由度 1..10）
_CHI2_95 = {
    1: 3.8415, 2: 5.9915, 3: 7.8147, 4: 9.4877, 5: 11.0705,
    6: 12.5916, 7: 14.0671, 8: 15.5073, 9: 16.9190, 10: 18.3070,
}

# ---- 静止保持期多径守卫参数（hold=True 时生效） ----
# 水平判定以「hold 锚点」为参照：进入保持时的位置估计。任何把融合位置
# 拉离锚点超过 MP_HOLD_DIST 的 fix 都判多径候选（无论当前估计已漂到哪），
# 从而把整个保持期的融合位置锚定在 ±MP_HOLD_DIST 圆内——实测夜间多径既有
# 3-4m 跳变式也有 ~0.16m/fix 的平滑漂移式，后者的 fix 相互一致（相邻
# <2.5m），若以当前估计为参照会经「连续一致恢复」通道被持续跟随。
MP_HOLD_DIST = 2.5           # m: fix 与 hold 锚点的水平距离上限，超过判多径候选
MP_CONSIST_DIST = 2.5        # m: 相邻 fix 水平距上限，低于视为「相互一致」
MP_RECOVER_CONSECUTIVE = 3   # 连续一致 fix 数达到该值才退出多径锁定
MP_V_HOLD_DOWNWEIGHT = 100   # hold 期垂直通道 R 的额外降权倍数：GPS 高度本就弱
                             # （夜间高程游走实测可达 40m），普通保持权重（增益
                             # ~0.1/fix）仍会跟随高程噪声；额外 ×100 后增益
                             # ~0.002/fix，静态垂直跨度压到米级。


# --------------------------------------------------------------------------
# 线性代数助手（纯 stdlib，9x9 / 3x3 小矩阵）
# --------------------------------------------------------------------------

def _zeros(n, m=None):
    m = n if m is None else m
    return [[0.0] * m for _ in range(n)]


def _eye(n):
    M = _zeros(n)
    for i in range(n):
        M[i][i] = 1.0
    return M


def _diag3(a, b, c):
    return [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]]


def _mat_mul(A, B):
    """A (n×k) · B (k×m) -> (n×m)。"""
    n, k, m = len(A), len(B), len(B[0])
    C = _zeros(n, m)
    for i in range(n):
        Ai = A[i]
        Ci = C[i]
        for j in range(m):
            s = 0.0
            for t in range(k):
                s += Ai[t] * B[t][j]
            Ci[j] = s
    return C


def _mat_T(A):
    n, m = len(A), len(A[0])
    return [[A[i][j] for i in range(n)] for j in range(m)]


def _mat_add(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def _mat_sub(A, B):
    return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def _mat_vec(A, v):
    """A (n×k) · v (k) -> (n)。"""
    return [sum(A[i][j] * v[j] for j in range(len(v))) for i in range(len(A))]


def _vec_sub(a, b):
    return [a[i] - b[i] for i in range(len(a))]


def _inv3(M):
    """3x3 逆（余子式/伴随法）。奇异时抛 ZeroDivisionError。"""
    a, b, c = M[0]
    d, e, f = M[1]
    g, h, i = M[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-15:
        raise ZeroDivisionError("singular 3x3 matrix")
    inv = [
        [(e * i - f * h), (c * h - b * i), (b * f - c * e)],
        [(f * g - d * i), (a * i - c * g), (c * d - a * f)],
        [(d * h - e * g), (b * g - a * h), (a * e - b * d)],
    ]
    return _mat_scale(inv, 1.0 / det)


def _invk(S):
    """小矩阵逆：1x1 或 3x3（3x3 走 _inv3 伴随法，行为与既有路径一致）。"""
    n = len(S)
    if n == 1:
        s = S[0][0]
        if abs(s) < 1e-15:
            raise ZeroDivisionError("singular matrix")
        return [[1.0 / s]]
    return _inv3(S)


def _mat_scale(A, s):
    return [[A[i][j] * s for j in range(len(A[0]))] for i in range(len(A))]


def _all_finite(seq):
    return all(math.isfinite(v) for v in seq)


# --------------------------------------------------------------------------
# EKF9
# --------------------------------------------------------------------------

class EKF9:
    """9 状态松耦合 EKF（位置/速度/加速度零偏）。

    用法:
        e = EKF9()
        e.predict(dt, a_nav)          # 每个 IMU 样本（实测 dt，导航系加速度）
        e.update_gps(n, e, u,
                     r_scale=1.0,
                     hold=False)      # 每个合格 GPS fix（ENU 位置；
                                      # hold=True 启用静止多径守卫）
        e.update_vel(vn, ve, vu)      # 可选 GPS 速度
        if e.zupt_detect(v_h, a_mag, w_mag):
            e.zupt()                  # 静止零速约束
        p, v = e.state()[:3], e.state()[3:6]
    """

    STATE_DIM = 9

    def __init__(self,
                 r_pos=(3.0, 3.0, 4.0),       # GPS 位置观测噪声 std [m]（水平 3，垂直 4）
                 r_vel=(0.5, 0.5, 0.5),       # GPS 速度观测噪声 std [m/s]
                 r_zupt=(0.05, 0.05, 0.05),   # ZUPT 零速伪观测噪声 std [m/s]
                 q_acc=(0.5, 0.5, 1.5),       # 加速度过程噪声 std [m/s²]（z 轴按姿态误差放大）
                 q_bias=(0.01, 0.01, 0.01),   # 加速度零偏随机游走 std [m/s²/√s]
                 p0_pos=(10.0, 10.0, 15.0),   # 初始位置不确定度 std [m]
                 p0_vel=(10.0, 10.0, 10.0),   # 初始速度不确定度 std [m/s]
                 p0_bias=(0.1, 0.1, 0.1)):    # 初始零偏不确定度 std [m/s²]
        self._r_pos = _diag3(r_pos[0] ** 2, r_pos[1] ** 2, r_pos[2] ** 2)
        self._r_vel = _diag3(r_vel[0] ** 2, r_vel[1] ** 2, r_vel[2] ** 2)
        self._r_zupt = _diag3(r_zupt[0] ** 2, r_zupt[1] ** 2, r_zupt[2] ** 2)
        self._q_acc = tuple(float(v) for v in q_acc)
        self._q_bias = tuple(float(v) for v in q_bias)
        self._p0_pos = tuple(float(v) for v in p0_pos)
        self._p0_vel = tuple(float(v) for v in p0_vel)
        self._p0_bias = tuple(float(v) for v in p0_bias)
        self._cov_min = 1e-6
        self._cov_max = 1e8
        self.multipath_rejected = 0  # hold 期多径跳变 fix 拒绝计数（诊断）
        self.reset()

    # -- 状态访问 --
    def state(self):
        """返回当前 9 维状态副本 [p_n,p_e,p_u,v_n,v_e,v_u,b_an,b_ae,b_au]。"""
        return list(self._x)

    def cov(self):
        """返回当前 9x9 协方差副本（诊断用）。"""
        return [list(row) for row in self._P]

    # -- 重置 --
    def reset(self):
        """重置状态到初始（零）与初始协方差。"""
        self._x = [0.0] * 9
        self._P = _eye(9)
        for i in range(3):
            self._P[i][i] = self._p0_pos[i] ** 2
            self._P[3 + i][3 + i] = self._p0_vel[i] ** 2
            self._P[6 + i][6 + i] = self._p0_bias[i] ** 2
        # 多径守卫状态机复位
        self.multipath_rejected = 0
        self._mp_lockout = False    # 多径锁定：水平通道抑制中
        self._mp_consistent = 0     # 锁定期间连续一致 fix 计数
        self._last_fix_he = None    # 上一 fix 水平位置 (n,e)，一致性参照
        self._anchor_he = None      # hold 锚点（进入保持时的位置估计）
        self._prev_hold = False     # 上一 fix 的 hold 状态（锚点重设判定）

    # -- 内部构造 --
    def _build_F(self, dt):
        F = _eye(9)
        for i in range(3):
            F[i][3 + i] = dt        # dp/dv
            F[3 + i][6 + i] = -dt   # dv/db_a
        return F

    def _build_Q(self, dt):
        Q = _zeros(9)
        dt2 = dt * dt
        for i in range(3):
            qa = self._q_acc[i] ** 2
            qb = self._q_bias[i] ** 2
            Q[i][i] += qa * (dt2 * dt2) / 4.0        # 位置（加速度二重积分）
            Q[i][3 + i] += qa * (dt2 * dt) / 2.0     # p-v 交叉
            Q[3 + i][i] += qa * (dt2 * dt) / 2.0
            Q[3 + i][3 + i] += qa * dt2              # 速度（加速度一重积分）
            Q[6 + i][6 + i] += qb * dt               # 零偏随机游走
        return Q

    @staticmethod
    def _H_pos():
        H = _zeros(3, 9)
        H[0][0] = H[1][1] = H[2][2] = 1.0
        return H

    @staticmethod
    def _H_u():
        """高度通道观测矩阵（1×9，仅取 p_u）。"""
        H = _zeros(1, 9)
        H[0][2] = 1.0
        return H

    @staticmethod
    def _H_vel():
        H = _zeros(3, 9)
        H[0][3] = H[1][4] = H[2][5] = 1.0
        return H

    @staticmethod
    def _meas3(a, b, c):
        """把 3 个观测转成有限 float 列表；任一非法 → None。"""
        try:
            z = [float(a), float(b), float(c)]
        except (TypeError, ValueError):
            return None
        if not _all_finite(z):
            return None
        return z

    # -- 守卫 --
    def _clamp_cov(self):
        P = self._P
        for i in range(self.STATE_DIM):
            d = P[i][i]
            if d < self._cov_min:
                P[i][i] = self._cov_min
            elif d > self._cov_max:
                P[i][i] = self._cov_max

    def _guard_state(self):
        """状态/协方差出现 NaN/Inf → reset()，返回是否发生重置。"""
        bad = (not _all_finite(self._x)) or \
              any(not _all_finite(row) for row in self._P)
        if bad:
            self.reset()
            return True
        return False

    # -- 预测 --
    def predict(self, dt, a_nav):
        """预测一步。dt 实测秒（>0），a_nav 为导航系加速度 (m/s²，已扣重力)。

        返回:
          True  - 正常
          False - 输入非法（dt<=0 / NaN），已 reset()
        """
        if not (math.isfinite(dt) and dt > 0.0):
            self.reset()
            return False
        if len(a_nav) != 3 or not _all_finite(a_nav):
            self.reset()
            return False

        x = self._x
        a = a_nav
        # 半隐式欧拉：p 用旧 v，v 用 a − b_a
        new_v = [x[3 + i] + (a[i] - x[6 + i]) * dt for i in range(3)]
        new_p = [x[i] + x[3 + i] * dt for i in range(3)]
        self._x = new_p + new_v + x[6:9]

        F = self._build_F(dt)
        FP = _mat_mul(F, self._P)
        self._P = _mat_add(_mat_mul(FP, _mat_T(F)), self._build_Q(dt))

        self._clamp_cov()
        self._guard_state()
        return True

    # -- 更新（通用卡尔曼，观测行数 k=1 或 3） --
    def _update(self, z, H, R, dof):
        x = self._x
        k = len(z)                                       # 观测行数（1 或 3）
        y = _vec_sub(z, _mat_vec(H, x))                  # 创新 (k,)
        HP = _mat_mul(H, self._P)                        # (k,9)
        S = _mat_add(_mat_mul(HP, _mat_T(H)), R)         # (k,k)
        try:
            Si = _invk(S)
        except ZeroDivisionError:
            return False

        # chi-square 创新门限：d² = yᵀ S⁻¹ y
        d2 = sum(y[i] * sum(Si[i][j] * y[j] for j in range(k)) for i in range(k))
        gate = _CHI2_95.get(dof, _CHI2_95[3])
        if d2 > gate:
            return False                                # 野值，拒绝

        K = _mat_mul(_mat_mul(self._P, _mat_T(H)), Si)  # 卡尔曼增益 (9,k)
        Ky = _mat_vec(K, y)
        self._x = [x[i] + Ky[i] for i in range(self.STATE_DIM)]

        KH = _mat_mul(K, H)
        IKH = _mat_sub(_eye(self.STATE_DIM), KH)
        self._P = _mat_mul(IKH, self._P)                # Joseph 简化形式

        self._clamp_cov()
        self._guard_state()
        return True

    # -- GPS 位置更新 --
    def update_gps(self, n, e, u, r_scale=1.0, hold=False):
        """GPS ENU 位置观测更新（H 取 p 分量，R 水平 3m/垂直 4m）。

        r_scale: R 缩放因子（默认 1.0）。调用方在静止位置保持期传入较大
        r_scale 以降低 GPS 位置权重——静止时 GPS 位置噪声（实测 ~10m 包络、
        相邻 fix 3-4m 跳变的多径漂移）不应拉动已知静止的位置估计。

        hold: 静止位置保持标志（默认 False，向后兼容）。hold=True 时启用:
          - 垂直通道额外降权（R_z × MP_V_HOLD_DOWNWEIGHT）：GPS 高度本就弱，
            hold 期垂直增益压到 ~0.002 量级，静态垂直跨度保持米级；垂直
            不参与多径判定。
          - 水平多径守卫（以 hold 锚点为参照）：进入保持时记录锚点（当前
            位置估计）；hold 期间 fix 与锚点的水平距离 > MP_HOLD_DIST
            （2.5m）判多径候选 → 进入锁定：水平通道跳过该 fix（位置不
            跟随，整段保持期位置被锚定在 ±2.5m 圆内），高度通道按保持
            权重照常更新，速度观测不受影响（由调用方单独 update_vel）；
            锁定期间仅当连续 MP_RECOVER_CONSECUTIVE（3）个相互
            < MP_CONSIST_DIST（2.5m）的一致 fix 出现，才退出锁定、重新
            按锚点判定（GPS 一致地回到锚点附近后恢复正常位置权重）。
            被拒 fix 计入 self.multipath_rejected。

        返回 True 表示观测（或其高度分量）被采纳；多径水平拒绝返回 False
        （此时高度分量仍可能已按保持权重更新），非法观测返回 False。
        """
        z = self._meas3(n, e, u)
        if z is None:
            return False
        if r_scale == 1.0:
            R = self._r_pos
        else:
            s = float(r_scale)
            R = [[self._r_pos[i][j] * s for j in range(3)] for i in range(3)]

        if not hold:
            # 运动期：沿用既有行为（r_scale 全通道），并复位多径状态机与
            # 锚点，使下一次保持期从干净状态开始。
            self._mp_lockout = False
            self._mp_consistent = 0
            self._last_fix_he = None
            self._anchor_he = None
            self._prev_hold = False
            return self._update(z, self._H_pos(), R, dof=3)

        # ---- 静止保持期：锚点 + 垂直通道额外降权 + 水平多径守卫 ----
        if not self._prev_hold or self._anchor_he is None:
            # 进入保持（或锚点失效）：以当前估计为锚点
            self._anchor_he = (self._x[0], self._x[1])
        self._prev_hold = True
        R_h = [[R[0][0], 0.0, 0.0],
               [0.0, R[1][1], 0.0],
               [0.0, 0.0, R[2][2] * MP_V_HOLD_DOWNWEIGHT]]
        R_u = [[R_h[2][2]]]

        # fix 与锚点的水平距离（判据参照锚点而非当前估计：当前估计可能已
        # 被小 fix 缓慢拉动，以它为参照会放跑平滑漂移型多径）
        dh_a = math.hypot(z[0] - self._anchor_he[0],
                          z[1] - self._anchor_he[1])
        if self._last_fix_he is not None:
            dfix = math.hypot(z[0] - self._last_fix_he[0],
                              z[1] - self._last_fix_he[1])
            if dfix < MP_CONSIST_DIST:
                self._mp_consistent += 1
            else:
                self._mp_consistent = 0
        self._last_fix_he = (z[0], z[1])

        if self._mp_lockout:
            if self._mp_consistent >= MP_RECOVER_CONSECUTIVE:
                self._mp_lockout = False
                self._mp_consistent = 0    # 退出锁定，下方按锚点重新判定
            else:
                self.multipath_rejected += 1
                # 高度通道按保持权重照常更新（垂直不参与多径判定），
                # 水平跳过；返回 False 表示该 fix 的水平位置未被采纳。
                self._update([z[2]], self._H_u(), R_u, dof=1)
                return False
        if dh_a > MP_HOLD_DIST:
            # 多径候选（含刚退出锁定的 fix 距锚点仍超限的情形）：重新锁定
            self._mp_lockout = True
            self._mp_consistent = 0
            self.multipath_rejected += 1
            self._update([z[2]], self._H_u(), R_u, dof=1)
            return False

        return self._update(z, self._H_pos(), R_h, dof=3)

    # -- GPS 速度更新（可选） --
    def update_vel(self, vn, ve, vu):
        """GPS 速度观测更新（H 取 v 分量）。可选第二观测。"""
        z = self._meas3(vn, ve, vu)
        if z is None:
            return False
        return self._update(z, self._H_vel(), self._r_vel, dof=3)

    # -- ZUPT --
    def zupt(self):
        """零速伪观测（v = 0）更新，抑制静止漂移。返回是否被采纳。"""
        return self._update([0.0, 0.0, 0.0], self._H_vel(), self._r_zupt, dof=3)

    def zupt_detect(self, v_gps_h, a_mag, w_mag):
        """静止检测：|v_gps_h|<0.5 且 ‖a‖∈[9.5,10.1] 且 ‖ω‖<0.1 rad/s。

        约定:
          - v_gps_h: GPS 水平速度模 [m/s]（无 GPS 时传 None，视为不否决）
          - a_mag:   ‖a_body‖ [m/s²]（未扣重力的原始加速度模，静止≈g）
          - w_mag:   ‖ω‖ [rad/s]（陀螺角速度模）
        """
        ok_gps = (v_gps_h is None) or \
                 (math.isfinite(v_gps_h) and abs(v_gps_h) < 0.5)
        ok_acc = math.isfinite(a_mag) and 9.5 <= a_mag <= 10.1
        ok_gyro = math.isfinite(w_mag) and w_mag < 0.1
        return ok_gps and ok_acc and ok_gyro


# --------------------------------------------------------------------------
# 内置合成测试（python3 ekf.py --test）
# --------------------------------------------------------------------------

def run_tests():
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(("PASS" if ok else "FAIL") + " - " + name + ((" : " + detail) if detail else ""))

    # -- 1. 静止漂移：零输入 60 步（dt=0.02，1.2s） --
    e = EKF9()
    for _ in range(60):
        e.predict(0.02, (0.0, 0.0, 0.0))
    s = e.state()
    drift = math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2)
    vspeed = math.sqrt(s[3] ** 2 + s[4] ** 2 + s[5] ** 2)
    check("static-drift<0.05", drift < 0.05, "drift=%.3g m" % drift)
    check("static-vel<0.01", vspeed < 0.01, "vel=%.3g m/s" % vspeed)

    # -- 2. GPS 收敛：50 步预测 + 5 次位置更新(5,0,0) --
    e = EKF9()
    for _ in range(50):
        e.predict(0.02, (0.0, 0.0, 0.0))
    for _ in range(5):
        e.update_gps(5.0, 0.0, 0.0)
    s = e.state()
    err = math.sqrt((s[0] - 5.0) ** 2 + s[1] ** 2 + s[2] ** 2)
    check("gps-converge<0.5", err < 0.5, "pos=%r err=%.3f m" % (s[:3], err))

    # -- 3. NaN 守卫：预测非法 dt → reset 不崩溃 --
    e = EKF9()
    ret = e.predict(float("nan"), (0.0, 0.0, 0.0))
    s = e.state()
    check("nan-predict-reset", ret is False, "predict(nan) returns %r" % ret)
    check("nan-state-finite-zero", _all_finite(s) and all(abs(v) < 1e-12 for v in s),
          "state=%r" % (s,))

    # -- 4. NaN 观测：非法 GPS 观测被忽略不崩溃 --
    e = EKF9()
    for _ in range(10):
        e.predict(0.02, (0.0, 0.0, 0.0))
    r1 = e.update_gps(float("nan"), 0.0, 0.0)
    r2 = e.update_gps(1.0, float("inf"), 0.0)
    s = e.state()
    check("nan-gps-ignored", r1 is False and r2 is False and _all_finite(s),
          "r1=%r r2=%r pos=%r" % (r1, r2, s[:3]))

    # -- 5. 野值门限：100m 阶跃应被 chi-square 拒绝 --
    e = EKF9()
    for _ in range(50):
        e.predict(0.02, (0.0, 0.0, 0.0))
    rej = e.update_gps(100.0, 0.0, 0.0)
    check("outlier-reject", rej is False, "100m outlier rejected=%r" % rej)

    # -- 6. ZUPT 检测边界 --
    e = EKF9()
    check("zupt-static", e.zupt_detect(0.0, 9.81, 0.01) is True, "静止 -> True")
    check("zupt-moving", e.zupt_detect(2.0, 9.81, 0.01) is False, "速度 2m/s -> False")
    check("zupt-swing", e.zupt_detect(0.0, 8.0, 0.01) is False, "加速度 8 -> False")
    check("zupt-rotate", e.zupt_detect(0.0, 9.81, 0.5) is False, "角速度 0.5 -> False")
    check("zupt-no-gps", e.zupt_detect(None, 9.81, 0.01) is True, "无 GPS 速度 -> True(仅 IMU 静止)")

    # -- 7. ZUPT 效果：含速度后 zupt 拉回零 --
    e = EKF9()
    e.predict(1.0, (0.5, 0.0, 0.0))       # 1s 加速到 ~0.5 m/s
    v_before = math.sqrt(sum(v * v for v in e.state()[3:6]))
    e.zupt()
    v_after = math.sqrt(sum(v * v for v in e.state()[3:6]))
    check("zupt-pull-zero", v_after < v_before, "v %.3f -> %.3f m/s" % (v_before, v_after))

    # -- 8. hold 多径拒绝：hold 下 6m 水平跳变不跟随（高度通道仍更新但钉住）--
    e = EKF9()
    for _ in range(50):
        e.predict(0.02, (0.0, 0.0, 0.0))
    for _ in range(10):
        e.update_gps(0.0, 0.0, 0.0)       # 无 hold 收敛到 0
    p0 = e.state()[:2]
    u0 = e.state()[2]
    r = e.update_gps(6.0, 0.0, 5.0, r_scale=25.0, hold=True)
    s = e.state()
    move = math.hypot(s[0] - p0[0], s[1] - p0[1])
    check("hold-multipath-reject",
          r is False and move < 0.5 and e.multipath_rejected >= 1
          and u0 < s[2] < u0 + 1.0,
          "6m jump: ret=%r h_move=%.4f u=%.4f rejected=%d"
          % (r, move, s[2], e.multipath_rejected))

    # -- 9. hold 小 fix（<2.5m）正常融合，不误拒 --
    e = EKF9()
    for _ in range(50):
        e.predict(0.02, (0.0, 0.0, 0.0))
    for _ in range(10):
        e.update_gps(0.0, 0.0, 0.0)
    r = e.update_gps(1.0, 0.0, 0.0, r_scale=25.0, hold=True)
    s = e.state()
    moved = math.hypot(s[0], s[1])
    check("hold-small-fix-fuses",
          r is True and e.multipath_rejected == 0 and 0.0 < moved < 1.0,
          "1m fix: ret=%r moved=%.4f rejected=%d"
          % (r, moved, e.multipath_rejected))

    # -- 10. hold 平滑漂移封顶：慢速一致漂移不把位置拉离锚点 --
    e = EKF9()
    for _ in range(50):
        e.predict(0.02, (0.0, 0.0, 0.0))
    for _ in range(10):
        e.update_gps(0.0, 0.0, 0.0)
    for d in (0.5, 1.0, 1.5, 2.0, 2.5):       # 锚点圆内：正常融合
        e.update_gps(d, 0.0, 0.0, r_scale=25.0, hold=True)
    r0 = e.update_gps(3.0, 0.0, 0.0, r_scale=25.0, hold=True)   # 出圆 -> 锁定
    r1 = e.update_gps(3.4, 0.0, 0.0, r_scale=25.0, hold=True)   # 一致 1
    r2 = e.update_gps(3.8, 0.0, 0.0, r_scale=25.0, hold=True)   # 一致 2
    r3 = e.update_gps(4.2, 0.0, 0.0, r_scale=25.0, hold=True)   # 一致 3 -> 退出锁定，
                                                                # 仍超锚点 -> 再锁定
    s = e.state()
    off_anchor = math.hypot(s[0], s[1])
    check("hold-smooth-drift-capped",
          r0 is False and r1 is False and r2 is False and r3 is False
          and off_anchor < MP_HOLD_DIST + 0.1 and e.multipath_rejected >= 2,
          "r=%r/%r/%r/%r off_anchor=%.3f rejected=%d"
          % (r0, r1, r2, r3, off_anchor, e.multipath_rejected))

    # -- 11. 多径恢复：GPS 一致地回到锚点附近后恢复正常位置权重 --
    e = EKF9()
    for _ in range(50):
        e.predict(0.02, (0.0, 0.0, 0.0))
    for _ in range(10):
        e.update_gps(0.0, 0.0, 0.0)
    e.update_gps(6.0, 0.0, 0.0, r_scale=25.0, hold=True)    # 触发锁定
    r1 = e.update_gps(4.0, 0.0, 0.0, r_scale=25.0, hold=True)   # 一致 1
    r2 = e.update_gps(2.5, 0.0, 0.0, r_scale=25.0, hold=True)   # 一致 2
    r3 = e.update_gps(1.5, 0.0, 0.0, r_scale=25.0, hold=True)   # 一致 3 -> 恢复
    s = e.state()
    check("hold-multipath-recover",
          r1 is False and r2 is False and r3 is True
          and e.multipath_rejected == 3 and s[0] > 0.0,
          "r1=%r r2=%r r3=%r rejected=%d pos_n=%.4f"
          % (r1, r2, r3, e.multipath_rejected, s[0]))

    # -- 12. 向后兼容：非 hold 下 6m 跳变照常融合（运动期跟随 GPS）--
    e = EKF9()
    for _ in range(50):
        e.predict(0.02, (0.0, 0.0, 0.0))
    for _ in range(10):
        e.update_gps(0.0, 0.0, 0.0)
    r = e.update_gps(6.0, 0.0, 0.0)       # hold 默认 False
    s = e.state()
    check("nohold-jump-follows",
          r is True and s[0] > 0.5 and e.multipath_rejected == 0,
          "ret=%r pos_n=%.3f rejected=%d" % (r, s[0], e.multipath_rejected))

    n_fail = sum(1 for _, ok in results if not ok)
    print("-" * 60)
    print("TOTAL: %d tests, %d FAIL" % (len(results), n_fail))
    return 0 if n_fail == 0 else 1


def main(argv):
    if len(argv) >= 2 and argv[1] == "--test":
        return run_tests()
    print(__doc__)
    print("用法: python3 ekf.py --test   # 运行内置合成测试")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

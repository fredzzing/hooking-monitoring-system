#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""attitude.py - Madgwick complementary attitude filter (AHRS, accel + gyro only).

Project-wide conventions (orangepi-gps-imu-fusion):
  * Hamilton quaternions q = (w, x, y, z), rotation matrix R(q) maps BODY -> EARTH
    (v_earth = R(q) @ v_body). NO JPL / scalar-last quaternions anywhere.
  * Inputs: gyro in rad/s (body frame; imu_reader output is already
    bias-corrected by its startup calibration), accel in m/s^2 (body frame).
  * Outputs: roll/pitch/yaw in rad (ZYX Euler of R), quaternion (w,x,y,z).
  * No magnetometer fitted on this board -> yaw is gyro-dead-reckoned;
    the z-axis gyro bias is unobservable from accel alone and stays near seed.

Features:
  * Adaptive gain: err = | ||a|| - g |. When err grows (hook swinging) the accel
    correction weight beta is ramped smoothly from beta_high (0.05) down to
    beta_low (0.01), so swing accelerations do not corrupt the gravity reference.
  * Online gyro bias: the Madgwick gradient step feeds an integral estimator of
    the gyro bias (sensor frame). Seeded from the startup calibration value
    (imu_reader calibrate() gyro_bias) via init_bias=.

Reference: S. Madgwick, "An efficient orientation filter for inertial and
inertial/magnetic sensor arrays", 2011 (gradient descent step eq. 21-23;
gyro-bias integral feedback section 3.4).
"""

import math

G_STD = 9.80665  # standard gravity [m/s^2]

DEFAULT_BETA_HIGH = 0.05        # normal accel correction gain
DEFAULT_BETA_LOW = 0.01         # swing gain (accel down-weighted)
DEFAULT_SWING_ERR = 0.3 * G_STD  # | ||a|| - g | above this -> swing regime
DEFAULT_RAMP_START = 0.2 * G_STD  # smooth transition begins here
DEFAULT_BETA_TAU = 0.05         # first-order smoothing time constant [s]
# tau=0.05 s: beta tracks swing onset within ~50 ms (hook swing period ~1 s),
# while still smoothing the gain (no step jumps). tau=0.1 s left beta lagging
# so much that a +-3 m/s^2 swing only reached beta~0.021 (self-test catch).
DEFAULT_ZETA = 0.1              # gyro-bias integral gain [1/s]
DEFAULT_BIAS_MAX = 0.3          # per-axis bias clamp [rad/s]


def _clamp(v, m):
    return max(-m, min(m, v))


def quat_from_rpy(roll, pitch, yaw):
    """ZYX Euler angles (rad) -> Hamilton quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def _qmul(a, b):
    """Hamilton product (a1,a2,a3,a4) x (b1,b2,b3,b4)."""
    a1, a2, a3, a4 = a
    b1, b2, b3, b4 = b
    return (a1 * b1 - a2 * b2 - a3 * b3 - a4 * b4,
            a1 * b2 + a2 * b1 + a3 * b4 - a4 * b3,
            a1 * b3 - a2 * b4 + a3 * b1 + a4 * b2,
            a1 * b4 + a2 * b3 - a3 * b2 + a4 * b1)


class Attitude(object):
    """Madgwick AHRS with adaptive accel gain and online gyro-bias estimate."""

    def __init__(self, q0=(1.0, 0.0, 0.0, 0.0),
                 beta_high=DEFAULT_BETA_HIGH, beta_low=DEFAULT_BETA_LOW,
                 swing_err=DEFAULT_SWING_ERR, ramp_start=DEFAULT_RAMP_START,
                 beta_tau=DEFAULT_BETA_TAU, zeta=DEFAULT_ZETA,
                 init_bias=(0.0, 0.0, 0.0), bias_max=DEFAULT_BIAS_MAX):
        n = math.sqrt(sum(v * v for v in q0))
        if not (n > 0.0 and math.isfinite(n)):
            raise ValueError("q0 must be a non-zero finite quaternion")
        self.q = [q0[0] / n, q0[1] / n, q0[2] / n, q0[3] / n]
        self.beta_high = float(beta_high)
        self.beta_low = float(beta_low)
        self.swing_err = float(swing_err)
        self.ramp_start = float(ramp_start)
        self.beta_tau = float(beta_tau)
        self.zeta = float(zeta)
        self.bias_max = float(bias_max)
        self.bias = [float(init_bias[0]), float(init_bias[1]), float(init_bias[2])]
        self.beta = self.beta_high   # current adaptive gain (public)
        self.accel_err = 0.0         # last | ||a|| - g | [m/s^2]
        self.n_updates = 0

    # ------------------------------------------------------------------ API
    def update(self, gyro_rad, accel_ms2, dt):
        """Fuse one IMU sample.

        gyro_rad:  (gx, gy, gz) rad/s, body frame (already bias-corrected).
        accel_ms2: (ax, ay, az) m/s^2, body frame.
        dt:        elapsed time since previous sample [s].
        """
        gx, gy, gz = gyro_rad
        ax, ay, az = accel_ms2
        if not (math.isfinite(gx) and math.isfinite(gy) and math.isfinite(gz)
                and math.isfinite(ax) and math.isfinite(ay) and math.isfinite(az)
                and math.isfinite(dt) and dt > 0.0):
            return  # reject non-finite / bad-timestamp input
        if dt > 0.5:   # clock glitch guard
            dt = 0.5
        if dt < 1e-4:
            dt = 1e-4

        # remove estimated gyro bias (sensor frame)
        wx = gx - self.bias[0]
        wy = gy - self.bias[1]
        wz = gz - self.bias[2]

        # ---- adaptive gain: swing detection via | ||a|| - g | ---------------
        an = math.sqrt(ax * ax + ay * ay + az * az)
        self.accel_err = abs(an - G_STD)
        if self.accel_err <= self.ramp_start:
            target = self.beta_high
        elif self.accel_err >= self.swing_err:
            target = self.beta_low
        else:  # smooth linear ramp between the two regimes
            f = ((self.accel_err - self.ramp_start)
                 / (self.swing_err - self.ramp_start))
            target = self.beta_high + (self.beta_low - self.beta_high) * f
        alpha = min(1.0, dt / self.beta_tau)  # first-order smoothing
        self.beta += (target - self.beta) * alpha

        q1, q2, q3, q4 = self.q
        # gyro integration: q_dot = 0.5 * q (x) (0, wx, wy, wz)
        qd1 = 0.5 * (-q2 * wx - q3 * wy - q4 * wz)
        qd2 = 0.5 * (q1 * wx + q3 * wz - q4 * wy)
        qd3 = 0.5 * (q1 * wy - q2 * wz + q4 * wx)
        qd4 = 0.5 * (q1 * wz + q2 * wy - q3 * wx)

        # ---- gradient-descent accel correction (if measurement usable) ------
        if an > 1e-6:
            axn, ayn, azn = ax / an, ay / an, az / an
            # f_g(q, a): gravity-direction error, Madgwick eq. 21
            f1 = 2.0 * (q2 * q4 - q1 * q3) - axn
            f2 = 2.0 * (q1 * q2 + q3 * q4) - ayn
            f3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - azn
            # grad = J^T f, J = d f_g / d q, Madgwick eq. 23
            s1 = -2.0 * q3 * f1 + 2.0 * q2 * f2
            s2 = 2.0 * q4 * f1 + 2.0 * q1 * f2 - 4.0 * q2 * f3
            s3 = -2.0 * q1 * f1 + 2.0 * q4 * f2 - 4.0 * q3 * f3
            s4 = 2.0 * q2 * f1 + 2.0 * q3 * f2
            sn = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
            if sn > 1e-12:
                s1n, s2n, s3n, s4n = s1 / sn, s2 / sn, s3 / sn, s4 / sn
                qd1 -= self.beta * s1n
                qd2 -= self.beta * s2n
                qd3 -= self.beta * s3n
                qd4 -= self.beta * s4n
                # online gyro-bias estimate (Madgwick 3.4):
                # residual = 2*beta*vec(q* (x) s_hat), sensor frame; the
                # integral of the residual tracks the true gyro bias.
                _pw, px, py, pz = _qmul((q1, -q2, -q3, -q4),
                                        (s1n, s2n, s3n, s4n))
                bstep = self.zeta * dt * 2.0 * self.beta
                self.bias[0] = _clamp(self.bias[0] + bstep * px, self.bias_max)
                self.bias[1] = _clamp(self.bias[1] + bstep * py, self.bias_max)
                self.bias[2] = _clamp(self.bias[2] + bstep * pz, self.bias_max)

        # ---- integrate + normalise -----------------------------------------
        q1 += qd1 * dt
        q2 += qd2 * dt
        q3 += qd3 * dt
        q4 += qd4 * dt
        qn = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        if qn > 1e-12:
            self.q = [q1 / qn, q2 / qn, q3 / qn, q4 / qn]
        self.n_updates += 1

    def quat(self):
        """Current attitude quaternion (w, x, y, z), Hamilton convention."""
        return tuple(self.q)

    def roll_pitch_yaw(self):
        """(roll, pitch, yaw) in rad, ZYX Euler of the body->earth rotation."""
        q1, q2, q3, q4 = self.q
        roll = math.atan2(2.0 * (q3 * q4 + q1 * q2),
                          1.0 - 2.0 * (q2 * q2 + q3 * q3))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q1 * q3 - q2 * q4))))
        yaw = math.atan2(2.0 * (q2 * q3 + q1 * q4),
                         1.0 - 2.0 * (q3 * q3 + q4 * q4))
        return roll, pitch, yaw

    def rotation_matrix(self):
        """Body -> earth rotation matrix from current quaternion (3x3 tuples)."""
        q1, q2, q3, q4 = self.q
        r11 = 1.0 - 2.0 * (q3 * q3 + q4 * q4)
        r12 = 2.0 * (q2 * q3 - q1 * q4)
        r13 = 2.0 * (q2 * q4 + q1 * q3)
        r21 = 2.0 * (q2 * q3 + q1 * q4)
        r22 = 1.0 - 2.0 * (q2 * q2 + q4 * q4)
        r23 = 2.0 * (q3 * q4 - q1 * q2)
        r31 = 2.0 * (q2 * q4 - q1 * q3)
        r32 = 2.0 * (q3 * q4 + q1 * q2)
        r33 = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        return ((r11, r12, r13), (r21, r22, r23), (r31, r32, r33))


# ---------------------------------------------------------------------- tests
def _self_test():
    fails = []

    def check(name, cond, detail=""):
        status = "PASS" if cond else "FAIL"
        print("%-56s %s %s" % (name, status, detail))
        if not cond:
            fails.append(name)

    dt = 0.02

    # 1) static, level start (identity q), 2 s of [0,0,g]
    a = Attitude()
    for _ in range(100):
        a.update((0.0, 0.0, 0.0), (0.0, 0.0, G_STD), dt)
    r, p, y = a.roll_pitch_yaw()
    qn = math.sqrt(sum(v * v for v in a.quat()))
    check("static level 2s: |roll| < 0.01 rad", abs(r) < 0.01, "roll=%.6f" % r)
    check("static level 2s: |pitch| < 0.01 rad", abs(p) < 0.01, "pitch=%.6f" % p)
    check("static level 2s: quat norm ~ 1", abs(qn - 1.0) < 1e-12,
          "norm=%.12f" % qn)

    # 2) convergence from tilted start (10 deg roll, -10 deg pitch)
    b = Attitude(q0=quat_from_rpy(math.radians(10.0), math.radians(-10.0), 0.0))
    for _ in range(150):
        b.update((0.0, 0.0, 0.0), (0.0, 0.0, G_STD), dt)
    r, p, y = b.roll_pitch_yaw()
    check("tilted start -> converge < 0.5 deg in 3 s",
          abs(r) < math.radians(0.5) and abs(p) < math.radians(0.5),
          "roll=%.4f deg pitch=%.4f deg" % (math.degrees(r), math.degrees(p)))

    # 3) adaptive gain: swing with ||a|| = g .. g+3 -> beta must drop < 0.02
    c = Attitude()
    bmin, bmax = 9.9, -1.0
    for i in range(100):
        t = i * dt
        ax = 8.24 * math.sin(2.0 * math.pi * 1.0 * t)
        c.update((0.0, 0.0, 0.0), (ax, 0.0, G_STD), dt)
        bmin = min(bmin, c.beta)
        bmax = max(bmax, c.beta)
    check("swing: beta drops below 0.02", bmin < 0.02,
          "beta_min=%.4f beta_max=%.4f (normal=0.05)" % (bmin, bmax))

    # 4) online gyro bias: static level, +0.03 rad/s injected on x
    d = Attitude(zeta=0.5)
    for _ in range(500):
        d.update((0.03, 0.0, 0.0), (0.0, 0.0, G_STD), dt)
    r, p, y = d.roll_pitch_yaw()
    check("bias estimate x -> +0.03 rad/s (within 20%)",
          abs(d.bias[0] - 0.03) < 0.006,
          "bias_x=%.5f bias=%s" % (d.bias[0],
                                   [round(v, 5) for v in d.bias]))
    check("bias test: y/z unobservable axes stay at seed",
          abs(d.bias[1]) < 1e-3 and abs(d.bias[2]) < 1e-3,
          "bias_y=%.6f bias_z=%.6f" % (d.bias[1], d.bias[2]))
    check("bias test: attitude ends level", abs(r) < 0.02 and abs(p) < 0.02,
          "roll=%.4f pitch=%.4f" % (r, p))

    print("-" * 72)
    total = 7
    if fails:
        print("SELF-TEST FAILED: %d/%d checks" % (len(fails), total))
        return 1
    print("SELF-TEST OK: all %d checks passed" % total)
    return 0


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        sys.exit(_self_test())
    a = Attitude()
    for _ in range(100):
        a.update((0.0, 0.0, 0.0), (0.0, 0.0, G_STD), 0.02)
    r, p, y = a.roll_pitch_yaw()
    print("roll=%.6f pitch=%.6f yaw=%.6f quat=%s beta=%.4f"
          % (r, p, y, a.quat(), a.beta))

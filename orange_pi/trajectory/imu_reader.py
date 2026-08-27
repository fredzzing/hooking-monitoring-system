#!/usr/bin/env python3
"""imu_reader.py -- MPU6050 50 Hz FIFO reader for the GPS+IMU hook-trajectory project.

Board: Orange Pi 3B, MPU6050 on i2c-2 @ 0x68 (400 kHz).
Timing comes from the MPU6050 internal sample clock delivered through the FIFO,
never from a fixed assumed dt.

Register configuration
----------------------
PWR_MGMT_1 (0x6B) = 0x00          wake up
SMPLRT_DIV (0x19) = 19            1000 Hz / (1 + 19) = 50 Hz sample rate
CONFIG     (0x1A) = 0x04          DLPF_CFG=4 -> accel ~21 Hz, gyro ~20 Hz BW (anti-alias)
GYRO_CONFIG(0x1B) = 0x08          FS_SEL=1 -> +-500 dps, 65.5 LSB/(deg/s)
ACCEL_CONFIG(0x1C)= 0x08          AFS_SEL=1 -> +-4 g, 8192 LSB/g
FIFO_EN    (0x23) = 0x78          accel + gyro FIFO enabled
USER_CTRL  (0x6A) = 0x40          FIFO enable bit

NOTE: the plan text mentions "GYRO_CONFIG/ACCEL_CONFIG = 0x10" and
"FIFO_EN @ 0x3A". 0x10 on those registers is FS_SEL=2/AFS_SEL=2 (+-1000 dps /
+-8 g) which contradicts the mandated FS_SEL=1/AFS_SEL=1 ranges and scaling
(65.5 LSB/dps, 8192 LSB/g); 0x3A is INT_STATUS, not FIFO_EN. This module uses
the register values that implement the required ranges: 0x08/0x08 and 0x23.

Scaling
-------
accel [m/s^2] = raw / 8192 * 9.80665
gyro  [rad/s] = raw / 65.5 * pi / 180

FIFO packet: this MPU6050 (clone) omits TEMP from the FIFO when
FIFO_EN bit 7 (TEMP_FIFO_EN) = 0, so a sample packet is 12 bytes =
6 accel + 6 gyro (no temp), read from 0x74. Empirically verified:
a 14-byte read drifts 2 bytes per packet and was rejected.
"""

import math
import time
from dataclasses import dataclass

import smbus2

# ---------------------------------------------------------------- constants
I2C_BUS = 2
DEV_ADDR = 0x68

REG_PWR_MGMT_1 = 0x6B
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_FIFO_EN = 0x23          # MPU6050 FIFO_EN (plan doc wrote 0x3A = INT_STATUS)
REG_USER_CTRL = 0x6A
REG_FIFO_COUNT_H = 0x72
REG_FIFO_COUNT_L = 0x73
REG_FIFO_R_W = 0x74

SMPLRT_DIV_VAL = 19         # 1 kHz / (1 + 19) = 50 Hz
DLPF_CFG_VAL = 4            # accel ~21 Hz, gyro ~20 Hz anti-alias bandwidth
GYRO_FS_SEL_VAL = 1         # +-500 dps
ACCEL_AFS_SEL_VAL = 1       # +-4 g
FIFO_EN_ACCEL_GYRO = 0x78   # bits 6..3: XG/YG/ZG/ACCEL FIFO enable
USER_CTRL_FIFO_EN = 0x40    # bit 6 FIFO enable

G = 9.80665
ACCEL_LSB_PER_G = 8192.0
GYRO_LSB_PER_DPS = 65.5
DEG2RAD = math.pi / 180.0

SAMPLE_BYTES = 12           # this clone: accel(6) + gyro(6); temp omitted when TEMP_FIFO_EN=0
FIFO_OVERFLOW_THRESH = 1024

# saturation thresholds: |raw| >= 0.98 * full-scale count
ACCEL_SAT_LSB = int(0.98 * ACCEL_LSB_PER_G * 4)      # 32112
GYRO_SAT_LSB = int(0.98 * GYRO_LSB_PER_DPS * 500)    # 32095


# ---------------------------------------------------------------- helpers
def _i16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


@dataclass
class IMUSample:
    """One bias-corrected (if calibrated) IMU sample.

    ax/ay/az: m/s^2   gx/gy/gz: rad/s
    sat_flag: 1 if any |raw| >= 0.98 * full scale, else 0
    """
    t_mono: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    sat_flag: int


class IMUReader:
    """Configure MPU6050 and read 50 Hz samples from its FIFO."""

    def __init__(self, bus=I2C_BUS, addr=DEV_ADDR):
        self.bus = smbus2.SMBus(bus)
        self.addr = addr
        self.cal = None          # Calibration dict, set by calibrate()
        self._configure()
        self._reset_fifo()

    # ------------------------------------------------------------ config
    def _configure(self):
        b = self.bus
        b.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x00)        # wake
        time.sleep(0.10)
        b.write_byte_data(self.addr, REG_SMPLRT_DIV, SMPLRT_DIV_VAL)
        b.write_byte_data(self.addr, REG_CONFIG, DLPF_CFG_VAL)
        b.write_byte_data(self.addr, REG_GYRO_CONFIG, GYRO_FS_SEL_VAL << 3)
        b.write_byte_data(self.addr, REG_ACCEL_CONFIG, ACCEL_AFS_SEL_VAL << 3)
        b.write_byte_data(self.addr, REG_FIFO_EN, FIFO_EN_ACCEL_GYRO)
        b.write_byte_data(self.addr, REG_USER_CTRL, USER_CTRL_FIFO_EN)
        time.sleep(0.05)

    def _reset_fifo(self):
        b = self.bus
        b.write_byte_data(self.addr, REG_USER_CTRL, 0x44)   # FIFO_EN | FIFO_RST
        b.write_byte_data(self.addr, REG_USER_CTRL, 0x40)   # re-enable
        time.sleep(0.01)

    def _fifo_count(self):
        hi = self.bus.read_byte_data(self.addr, REG_FIFO_COUNT_H)
        lo = self.bus.read_byte_data(self.addr, REG_FIFO_COUNT_L)
        return (hi << 8) | lo

    # ------------------------------------------------------------ sample
    def _read_raw_sample(self):
        """Read one FIFO sample WITHOUT bias correction."""
        while True:
            n = self._fifo_count()
            if n >= SAMPLE_BYTES:
                break
            if n >= FIFO_OVERFLOW_THRESH:      # overflow guard
                self._reset_fifo()
            time.sleep(0.0005)

        data = self.bus.read_i2c_block_data(self.addr, REG_FIFO_R_W, SAMPLE_BYTES)
        t = time.monotonic()

        ax_raw = _i16(data[0], data[1])
        ay_raw = _i16(data[2], data[3])
        az_raw = _i16(data[4], data[5])
        gx_raw = _i16(data[6], data[7])
        gy_raw = _i16(data[8], data[9])
        gz_raw = _i16(data[10], data[11])

        sat_flag = 0
        if (abs(ax_raw) >= ACCEL_SAT_LSB or abs(ay_raw) >= ACCEL_SAT_LSB
                or abs(az_raw) >= ACCEL_SAT_LSB or abs(gx_raw) >= GYRO_SAT_LSB
                or abs(gy_raw) >= GYRO_SAT_LSB or abs(gz_raw) >= GYRO_SAT_LSB):
            sat_flag = 1

        ax = ax_raw / ACCEL_LSB_PER_G * G
        ay = ay_raw / ACCEL_LSB_PER_G * G
        az = az_raw / ACCEL_LSB_PER_G * G
        gx = gx_raw / GYRO_LSB_PER_DPS * DEG2RAD
        gy = gy_raw / GYRO_LSB_PER_DPS * DEG2RAD
        gz = gz_raw / GYRO_LSB_PER_DPS * DEG2RAD

        return IMUSample(t_mono=t, ax=ax, ay=ay, az=az,
                         gx=gx, gy=gy, gz=gz, sat_flag=sat_flag)

    def read_sample(self):
        """Block until the next FIFO sample is available; return it
        bias-corrected (after calibrate() has run)."""
        s = self._read_raw_sample()
        if self.cal is not None:               # apply startup calibration
            s.ax -= self.cal["accel_bias"][0]
            s.ay -= self.cal["accel_bias"][1]
            s.az -= self.cal["accel_bias"][2]
            s.gx -= self.cal["gyro_bias"][0]
            s.gy -= self.cal["gyro_bias"][1]
            s.gz -= self.cal["gyro_bias"][2]
        return s

    # ---------------------------------------------------------- calibrate
    def calibrate(self, duration=10.0):
        """Static-startup calibration.

        Requires the IMU to be stationary for `duration` seconds.
        Returns (and stores) a Calibration dict with keys:
            gyro_bias     : [rad/s, rad/s, rad/s] mean gyro output at rest
            accel_bias    : [m/s^2, ...]  mean accel minus gravity component
            mount_gravity : unit vector of gravity in the body frame
        """
        self._reset_fifo()
        n = 0
        s_ax = s_ay = s_az = s_gx = s_gy = s_gz = 0.0
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            s = self._read_raw_sample()        # raw, never self-corrected
            s_ax += s.ax; s_ay += s.ay; s_az += s.az
            s_gx += s.gx; s_gy += s.gy; s_gz += s.gz
            n += 1

        if n < 10:
            raise RuntimeError("calibration collected too few samples: %d" % n)

        m_ax, m_ay, m_az = s_ax / n, s_ay / n, s_az / n
        norm = math.sqrt(m_ax * m_ax + m_ay * m_ay + m_az * m_az)
        if norm < 1.0:
            raise RuntimeError("accel magnitude implausible during calibration: %.3f" % norm)

        mg = [m_ax / norm, m_ay / norm, m_az / norm]          # unit vector
        accel_bias = [m_ax - G * mg[0], m_ay - G * mg[1], m_az - G * mg[2]]
        gyro_bias = [s_gx / n, s_gy / n, s_gz / n]

        self.cal = {
            "gyro_bias": gyro_bias,
            "accel_bias": accel_bias,
            "mount_gravity": mg,
            "n_samples": n,
            "duration_s": duration,
        }
        return self.cal


if __name__ == "__main__":
    r = IMUReader()
    print(r.calibrate(10))

#!/usr/bin/env python3
"""imu_reader.py -- WitMotion WT901SDCL 50 Hz serial reader for the GPS+IMU
hook-trajectory project.

Sensor replacement: the MPU6050 (smbus2, i2c-2 @ 0x68, FIFO) is replaced by a
WitMotion WT901SDCL smart IMU on a USB-serial port (pyserial). The public
interface is unchanged, so recorder.py keeps working untouched:

    IMUSample dataclass : t_mono, ax, ay, az (m/s^2), gx, gy, gz (rad/s), sat_flag
    IMUReader()         : auto-detect device + baud rate, configure, open
    read_sample()       : block for the next accel+gyro pair, bias-corrected
    calibrate(10.0)     : static calibration -> {gyro_bias, accel_bias,
                          mount_gravity, n_samples, duration_s}
    _fifo_count()       : serial RX backlog in bytes (recorder backdates
                          sample stamps with rem // SAMPLE_BYTES; serial
                          in_waiting keeps that byte-based contract)
    SAMPLE_BYTES = 22   : one sample = accel frame (11 B) + gyro frame (11 B)

WitMotion frame (11 bytes):
    0x55 | TYPE | D0L D0H D1L D1H D2L D2H D3L D3H | SUM
    SUM  = (sum of the first 10 bytes) & 0xFF
    0x51 = acceleration frame (D0..D2 = ax, ay, az, int16 little-endian)
    0x52 = angular-rate frame (D0..D2 = wx, wy, wz, int16 little-endian)

Scaling (WitMotion defaults: +-16 g, +-2000 dps, 32768 = full scale):
    accel [m/s^2] = raw / 32768 * 16 * 9.80665
    gyro  [rad/s] = raw / 32768 * 2000 * pi / 180

Device / baud probing
---------------------
Candidates are /dev/ttyUSB* and /dev/ttyACM* MINUS /dev/ttyACM0 (u-blox GPS,
never touched). Baud order: 115200 then 9600 (factory default). A probe =
open the port and read for up to 2 s looking for a valid 0x55 frame with a
good checksum.

This project needs 50 Hz, and 9600 baud cannot carry 50 Hz x 22 B = 1100 B/s
(9600/10 = 960 B/s). The output configuration is therefore (re)written on
EVERY open, at the baud rate the module currently answers on (idempotent):
    unlock           : FF AA 69 88 B5
    reg 0x02 = 0x06  : output content = accel + gyro only (bandwidth)
    reg 0x03 = 0x08  : output rate = 50 Hz
    reg 0x04 = 0x06  : baud rate = 115200 -- ONLY on the factory 9600 path
    save             : FF AA 00 00 00
A module that already answers at 115200 keeps its baud (no pointless switch):
measured on-board, the baud can be right while the output rate/content are
still wrong (200 Hz, 5 packet types 0x50..0x54), so a mere "frames at 115200"
probe is NOT proof of correct configuration.

HARDWARE QUIRKS (measured on-board, decisive):
1. The firmware only accepts PROTECTED-register writes (rate 0x03, baud 0x04)
   when the unlock, the write command and the save arrive as ONE CONTIGUOUS
   burst in a single serial write(). Separate writes with 10-500 ms gaps are
   silently REJECTED (the rate register readback never changed, output stayed
   200 Hz), while unprotected content writes (0x02) applied anyway - which
   hid the bug. Multi-command bursts proved fragile (byte-sync glitches at
   115200), so EACH register gets its own unlock+cmd+save burst with a short
   gap between bursts. Changes then apply IMMEDIATELY (no power cycle).
2. The module flushes its output in bursts (measured: 10 accel+gyro pairs
   every 200 ms at 50 Hz). read_sample() therefore reconstructs the
   PRODUCTION timestamp of each pair by backdating the arrival stamp by the
   unconsumed backlog (OS buffer + parser buffer) // SAMPLE_BYTES * 20 ms.
   This yields smooth ~20 ms sample spacing (on-board p95 jitter < 1 ms)
   instead of the raw 0.06/200 ms arrival bimodality.

After writing, the port is re-probed: frames must still flow. If the module
goes silent at 115200 after configuration, the reader warns and falls back
to 9600 (degraded throughput beats no data).

Robustness
----------
read_sample() re-synchronises byte-by-byte: garbage bytes, dropped frames,
misaligned reads and checksum errors are skipped without crashing. The
timestamp is time.monotonic() at the arrival of the LATER of the two frames
of a pair. A no-data stall guard raises IMUError after NO_DATA_TIMEOUT_S s
so the recorder's IMU thread logs the fault instead of silently spinning.
"""

import glob
import math
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:            # pragma: no cover - board has pyserial 3.5
    serial = None

# ---------------------------------------------------------------- constants
PKT_ACCEL = 0x51
PKT_GYRO = 0x52
FRAME_LEN = 11
SAMPLE_BYTES = 22            # accel frame (11) + gyro frame (11)

G = 9.80665
DEG2RAD = math.pi / 180.0

ACCEL_FS_G = 16.0            # +-16 g full scale
GYRO_FS_DPS = 2000.0         # +-2000 dps full scale
RAW_FULL_SCALE = 32768.0     # int16 full-scale count
ACCEL_RAW_TO_MS2 = ACCEL_FS_G * G / RAW_FULL_SCALE
GYRO_RAW_TO_RADS = GYRO_FS_DPS * DEG2RAD / RAW_FULL_SCALE
SAT_LSB = int(0.98 * RAW_FULL_SCALE)   # 32112: |raw| >= this -> sat_flag = 1

# device probing: /dev/ttyUSB* first, then /dev/ttyACM*; ttyACM0 = u-blox GPS
DEVICE_GLOBS = ("/dev/ttyUSB*", "/dev/ttyACM*")
EXCLUDED_DEVICES = ("/dev/ttyACM0",)
BAUD_CANDIDATES = (115200, 9600)
CONFIG_BAUD = 9600
RUN_BAUD = 115200
PROBE_TIMEOUT = 2.0          # s of reading per (device, baud) probe attempt
NO_DATA_TIMEOUT_S = 5.0      # s without any serial bytes -> IMUError
READ_CHUNK = 64              # bytes per read() call

# WitMotion configuration commands
CMD_UNLOCK = b"\xFF\xAA\x69\x88\xB5"
CMD_SAVE = b"\xFF\xAA\x00\x00\x00"
REG_OUTPUT_CONTENT = 0x02    # 0x06 = accel + gyro only
REG_OUTPUT_RATE = 0x03       # 0x08 = 50 Hz
REG_BAUD = 0x04              # 0x06 = 115200
CONFIG_WRITES_CONTENT_RATE = (          # always (re)written on every open
    (REG_OUTPUT_CONTENT, 0x06),
    (REG_OUTPUT_RATE, 0x08),
)
CONFIG_WRITES_BAUD = (                  # factory 9600 path only
    (REG_BAUD, 0x06),
)
CONFIG_SLEEP_APPLY = 0.3    # s after config before re-probing

# production-time reconstruction (module flushes in 200 ms bursts)
NOMINAL_RATE_HZ = 50.0      # configured output rate
DT_NOMINAL = 1.0 / NOMINAL_RATE_HZ
MAX_BACKDATE_PAIRS = 10     # cap for the backlog backdating (200 ms)


# ---------------------------------------------------------------- helpers
def _i16(lo, hi):
    """Decode int16 little-endian (WitMotion DxL DxH byte order)."""
    v = lo | (hi << 8)
    return v - 65536 if v >= 32768 else v


def _extract_frame(buf):
    """Scan buf for the next valid 11-byte frame.

    Consumes buf up to and including a valid frame; returns
    (ptype, d0, d1, d2, d3) or None. Garbage, noise and checksum errors are
    skipped one byte at a time (re-sync on the next 0x55).
    """
    while len(buf) >= FRAME_LEN:
        idx = buf.find(0x55)
        if idx < 0:
            del buf[:max(0, len(buf) - (FRAME_LEN - 1))]   # keep tail: a
            return None                                    # header may straddle
        if idx > 0:
            del buf[:idx]
            continue
        ptype = buf[1]
        if (ptype in (PKT_ACCEL, PKT_GYRO)
                and (sum(buf[0:10]) & 0xFF) == buf[10]):
            d0 = _i16(buf[2], buf[3])
            d1 = _i16(buf[4], buf[5])
            d2 = _i16(buf[6], buf[7])
            d3 = _i16(buf[8], buf[9])
            del buf[:FRAME_LEN]
            return (ptype, d0, d1, d2, d3)
        del buf[0]             # bad checksum / unknown type: re-sync
    return None


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


class IMUError(RuntimeError):
    """Fatal IMU hardware/configuration error (readable message)."""


def _find_devices():
    """Existing /dev/ttyUSB* + /dev/ttyACM* devices, excluding the GPS."""
    found = []
    for pattern in DEVICE_GLOBS:
        found.extend(glob.glob(pattern))
    found = sorted(set(found),
                   key=lambda d: (not d.startswith("/dev/ttyUSB"), d))
    return [d for d in found if d not in EXCLUDED_DEVICES]


class IMUReader:
    """Open/configure the WitMotion WT901SDCL and read 50 Hz accel+gyro pairs."""

    def __init__(self, port=None, baud=None, probe_timeout=PROBE_TIMEOUT):
        self.ser = None
        self._buf = bytearray()
        self.cal = None              # Calibration dict, set by calibrate()
        self.port = None
        self.baud = None
        self._open(port=port, baud=baud, probe_timeout=probe_timeout)

    # ------------------------------------------------------------ open
    def _open(self, port=None, baud=None, probe_timeout=PROBE_TIMEOUT):
        if serial is None:
            raise IMUError("pyserial is not installed (pip install pyserial)")
        devices = [port] if port is not None else _find_devices()
        if not devices:
            raise IMUError(
                "no candidate WitMotion serial device found (searched %s; "
                "/dev/ttyACM0 excluded = u-blox GPS). Check the WT901SDCL "
                "USB cable / power." % ", ".join(DEVICE_GLOBS))

        if baud is not None:                       # fixed baud: verify only
            for dev in devices:
                try:
                    if self._probe_port(dev, baud, probe_timeout):
                        self._set_serial(dev, baud)
                        return
                except serial.SerialException as exc:
                    print("WARN: %s open failed: %s" % (dev, exc),
                          file=sys.stderr, flush=True)
            raise IMUError(
                "no valid 0x55 frames on %s at %d baud (%.1f s per probe). "
                "Check the USB connection." % (devices, baud, probe_timeout))

        for dev in devices:                        # auto baud detect + configure
            try:
                if self._open_on_device(dev, probe_timeout):
                    return
            except serial.SerialException as exc:
                print("WARN: %s open failed: %s" % (dev, exc),
                      file=sys.stderr, flush=True)
                continue
        raise IMUError(
            "could not detect/configure WT901SDCL on %s at %s baud: no valid "
            "0x55 frames within %.1f s per attempt. Check the USB cable / "
            "module power, and that the module is not on /dev/ttyACM0 "
            "(u-blox GPS)." % (devices, list(BAUD_CANDIDATES), probe_timeout))

    def _probe_port(self, port, baud, timeout):
        """Open port at baud and read until one valid frame appears (or not).

        Returns True if a checksum-valid 0x51/0x52 frame was received within
        `timeout` seconds. Never raises for a missing/non-IMU device.
        """
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException:
            return False
        buf = bytearray()
        t_end = time.monotonic() + timeout
        try:
            while time.monotonic() < t_end:
                try:
                    chunk = ser.read(READ_CHUNK)
                except serial.SerialException:
                    break
                if not chunk:
                    time.sleep(0.005)
                    continue
                buf.extend(chunk)
                if _extract_frame(buf) is not None:
                    return True
        finally:
            try:
                ser.close()
            except Exception:        # noqa: BLE001 - best effort
                pass
        return False

    def _write_config(self, port, baud, include_baud):
        """Open a temporary serial at `baud` and write the WitMotion config.

        Each register gets its own unlock+write+save CONTIGUOUS burst in a
        single write() call (hardware quirk: separate writes are silently
        rejected for protected registers - see module docstring), with a
        short gap between bursts. The temporary port is closed afterwards;
        the caller re-probes to verify frames still flow."""
        cmds = [bytes((0xFF, 0xAA, reg, val, 0x00))
                for reg, val in CONFIG_WRITES_CONTENT_RATE]
        if include_baud:
            cmds += [bytes((0xFF, 0xAA, reg, val, 0x00))
                     for reg, val in CONFIG_WRITES_BAUD]
        ser = serial.Serial(port, baud, timeout=0.2)
        try:
            time.sleep(0.2)
            for cmd in cmds:
                ser.write(CMD_UNLOCK + cmd + CMD_SAVE)
                time.sleep(0.25)
            time.sleep(CONFIG_SLEEP_APPLY)
        finally:
            ser.close()

    def _open_on_device(self, dev, probe_timeout):
        """Bring one device up; returns True once the serial is verified+set.

        A 115200 probe hit is NOT proof of correct configuration (measured
        on-board: module at 115200 but 200 Hz x 5 packet types), so the
        content/rate registers are ALWAYS rewritten, at 115200, and frames
        must keep flowing afterwards. Only a factory 9600 module gets the
        baud write; it is then re-opened at 115200. If the module goes silent
        at 115200 after configuration, the 9600 path is tried with a warning.
        """
        if self._probe_port(dev, RUN_BAUD, probe_timeout):
            print("IMU: %s @ %d baud - configuring accel+gyro @ 50 Hz"
                  % (dev, RUN_BAUD), flush=True)
            self._write_config(dev, RUN_BAUD, include_baud=False)
            time.sleep(CONFIG_SLEEP_APPLY)
            if self._probe_port(dev, RUN_BAUD, probe_timeout):
                self._set_serial(dev, RUN_BAUD)
                return True
            print("WARN: %s silent at %d baud after config; trying %d"
                  % (dev, RUN_BAUD, CONFIG_BAUD), file=sys.stderr, flush=True)
        if self._probe_port(dev, CONFIG_BAUD, probe_timeout):
            print("IMU: %s @ %d baud (factory) - configuring accel+gyro "
                  "50 Hz @ %d" % (dev, CONFIG_BAUD, RUN_BAUD), flush=True)
            self._write_config(dev, CONFIG_BAUD, include_baud=True)
            time.sleep(CONFIG_SLEEP_APPLY)
            if self._probe_port(dev, RUN_BAUD, probe_timeout):
                self._set_serial(dev, RUN_BAUD)
                return True
            print("WARN: no frames at %d baud after config; continuing at "
                  "%d (throughput-limited)" % (RUN_BAUD, CONFIG_BAUD),
                  file=sys.stderr, flush=True)
            if self._probe_port(dev, CONFIG_BAUD, probe_timeout):
                self._set_serial(dev, CONFIG_BAUD)
                return True
        return False

    def _set_serial(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.port = port
        self.baud = baud
        self._buf = bytearray()
        print("IMU: opened %s @ %d baud (pyserial)" % (port, baud), flush=True)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:        # noqa: BLE001
                pass
            self.ser = None

    # ------------------------------------------------------------ sample
    def _fifo_count(self):
        """Bytes backlogged in the serial RX buffer (recorder backdates late
        sample stamps by rem // SAMPLE_BYTES; serial in_waiting keeps the
        byte-based contract of the old MPU FIFO depth)."""
        try:
            return self.ser.in_waiting
        except Exception:            # noqa: BLE001
            return 0

    def _read_next_packet(self):
        """Block until one valid frame; return (ptype, d0, d1, d2, d3, t_mono).

        Raises IMUError when no bytes at all arrive for NO_DATA_TIMEOUT_S s
        (cable pulled / module dead) so the caller can log instead of
        spinning forever."""
        last_byte = time.monotonic()
        while True:
            while len(self._buf) < FRAME_LEN:
                try:
                    chunk = self.ser.read(READ_CHUNK)
                except serial.SerialException:
                    chunk = b""
                if chunk:
                    last_byte = time.monotonic()
                    self._buf.extend(chunk)
                else:
                    if time.monotonic() - last_byte > NO_DATA_TIMEOUT_S:
                        raise IMUError(
                            "no data from WT901SDCL on %s for %.1f s - check "
                            "the USB cable / module power"
                            % (self.port, NO_DATA_TIMEOUT_S))
                    time.sleep(0.001)
            pkt = _extract_frame(self._buf)
            if pkt is not None:
                return pkt + (time.monotonic(),)

    def _packets_to_sample(self, accel_pkt, gyro_pkt):
        """Convert a matched 0x51/0x52 packet pair to a raw IMUSample
        (raw = no bias correction).

        Timestamp = reconstructed PRODUCTION time: the module flushes its
        50 Hz output in bursts (measured: 10 pairs per 200 ms), so the raw
        arrival stamp is backdated by the unconsumed backlog (OS input
        buffer + parser buffer) // SAMPLE_BYTES * DT_NOMINAL. This yields
        smooth ~20 ms sample spacing (on-board: p95 jitter < 1 ms) instead
        of the raw 0.06 ms / 200 ms arrival bimodality."""
        _, ax_r, ay_r, az_r, _, t_a = accel_pkt
        _, gx_r, gy_r, gz_r, _, t_g = gyro_pkt
        t_arr = t_g if t_g >= t_a else t_a
        try:
            backlog = self.ser.in_waiting + len(self._buf)
        except Exception:                # noqa: BLE001
            backlog = len(self._buf)
        k = backlog // SAMPLE_BYTES
        if k > MAX_BACKDATE_PAIRS:
            k = MAX_BACKDATE_PAIRS
        t = t_arr - k * DT_NOMINAL
        sat_flag = 0
        if (abs(ax_r) >= SAT_LSB or abs(ay_r) >= SAT_LSB or abs(az_r) >= SAT_LSB
                or abs(gx_r) >= SAT_LSB or abs(gy_r) >= SAT_LSB
                or abs(gz_r) >= SAT_LSB):
            sat_flag = 1
        return IMUSample(t_mono=t,
                         ax=ax_r * ACCEL_RAW_TO_MS2,
                         ay=ay_r * ACCEL_RAW_TO_MS2,
                         az=az_r * ACCEL_RAW_TO_MS2,
                         gx=gx_r * GYRO_RAW_TO_RADS,
                         gy=gy_r * GYRO_RAW_TO_RADS,
                         gz=gz_r * GYRO_RAW_TO_RADS,
                         sat_flag=sat_flag)

    def _read_raw_sample(self):
        """Block for one accel+gyro pair of the SAME output cycle; raw."""
        accel = None
        while True:
            pkt = self._read_next_packet()
            if pkt[0] == PKT_ACCEL:
                accel = pkt
                while True:                    # wait for this accel's gyro
                    pkt2 = self._read_next_packet()
                    if pkt2[0] == PKT_GYRO:
                        return self._packets_to_sample(accel, pkt2)
                    if pkt2[0] == PKT_ACCEL:
                        accel = pkt2           # newer accel: keep waiting
            # a gyro before any accel is the stale tail of a lost cycle: skip

    def read_sample(self):
        """Block until the next accel+gyro pair is available; return it
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
        """Static-startup calibration (module configuration NOT modified).

        Requires the IMU to be stationary for `duration` seconds.
        Returns (and stores) a Calibration dict with keys:
            gyro_bias     : [rad/s, rad/s, rad/s] mean gyro output at rest
            accel_bias    : [m/s^2, ...]  mean accel minus gravity component
            mount_gravity : unit vector of gravity in the body frame
        """
        try:
            self.ser.reset_input_buffer()      # start from a clean stream
        except Exception:                      # noqa: BLE001
            pass
        self._buf = bytearray()
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
    try:
        print("device=%s baud=%d" % (r.port, r.baud))
        print(r.calibrate(10.0))
    finally:
        r.close()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""recorder.py - 3-thread GPS + IMU + EKF trajectory recorder (Orange Pi).

Threads:
  GPS thread            : gps_reader.GPSReader.read_fix()   -> gps_queue
                          (bounded 16, drop-oldest + counted)
  IMU thread            : imu_reader.IMUReader.read_sample()-> imu_queue
                          (bounded 64, drop-oldest + counted)
  Fusion + writer (main): consume IMU samples with MEASURED dt ->
                          attitude.update -> a_nav = R(q)*a_body - [0,0,g] ->
                          ekf.predict -> new GPS fix? -> ekf.update_gps(+vel) ->
                          ZUPT detect/apply -> 28-col row -> CSV buffer,
                          fsync every FSYNC_ROWS rows.

Bias convention (IMPORTANT): IMUReader.read_sample() ALREADY removes the
startup-calibration gyro_bias / accel_bias. Therefore:
  - Attitude is created with init_bias=(0,0,0) (its online gyro-bias
    estimator tracks residual drift from zero),
  - a_nav uses the bias-corrected a_body directly (R*a_body - [0,0,g]).
Subtracting b_a_body a second time would double-correct.

Attitude pre-alignment (IMPORTANT): the board is mounted far from flat
(mount_gravity unit vector measured at calibration). The Madgwick filter is
therefore started from a quaternion q0 built from mount_gravity (body
gravity direction -> earth +z) instead of identity; starting from identity
at this ~83 deg tilt takes ~25-30 s to converge (beta=0.05), during which
a_nav is wrong by up to 18 m/s^2 and the EKF velocity runs away (GPS
updates then get rejected by the chi-square gate). Pre-aligned, the
stationary a_nav error is ~0.02 m/s^2 from the first sample.

Timestamp reconstruction (IMPORTANT): read-time stamps are polluted by
intermittent serial-read stalls (measured on-board: UART read transactions
occasionally take 20-43 ms instead of ~1-3 ms when the GPS USB-serial reader
runs concurrently; the module keeps producing into the kernel input buffer
during the stall, so no sample is lost). Instead of writing late read-times,
the recorder backdates each sample by the serial-buffer backlog:
t_out = t_raw - k*dt_ema, where
k = (bytes remaining in the serial input buffer after the read) //
    SAMPLE_BYTES and dt_ema is the
measured production interval (initialized from the 10 s calibration sample
count, then refined online as the median of recent raw inter-read intervals,
robust to the serial-read stalls). The WitMotion WT901SDCL has no FIFO:
_fifo_count() returns the serial input buffer backlog in bytes
(ser.in_waiting). This reconstructs
the sensor's true production timeline (error ~ms) and keeps dt monotonic
and gap-free. Backdated sample count and max backdate are recorded in
.meta.json ("backdated"); nothing is dropped and no dt is hardcoded.

Startup: numpy soft check -> GPS reader open + GPS thread -> IMU calibrate
(10 s, GPS fix collection overlaps) -> wait first gated GPS fix ->
GPS survey-in (collect fixes for SURVEY_IN_SECONDS, set origin to the
element-wise MEDIAN so a cold-start convergence transient of the receiver
does not bake into the trajectory) -> record until --duration or
SIGINT/SIGTERM -> flush + fsync + tail integrity check -> .meta.json
(origin / survey_in / calibration / config / dropped / coast counts).

Attitude output convention: att_roll/pitch/yaw in the CSV are the attitude
of the hook assembly RELATIVE to the calibrated mount pose (q_rel =
q_mount^-1 * q_att). The board is bolted at a fixed large tilt
(mount_gravity ~153 deg roll), so absolute body attitude would read
~153 deg while static; the mount-relative output reads ~0 deg static and
reports real motion of the hook. Absolute board attitude is recoverable
from q_mount (meta mount_gravity) if ever needed.

Output: <out>/YYYYMMDD-HHMMSS.csv + same-stem .meta.json (28-column frozen
schema, see CSV_SCHEMA.md / validate_csv.py). zupt: 0=moving, 1=ZUPT,
2=coast (GPS stale > COAST_TIMEOUT s; gps numeric cols empty).
"""

import argparse
import collections
import datetime
import json
import math
import os
import signal
import sys
import threading
import time

# ---- local modules (deployed alongside this file) -------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geo                      # noqa: E402
import attitude as att_mod      # noqa: E402
import ekf as ekf_mod           # noqa: E402
import gps_reader               # noqa: E402
import imu_reader               # noqa: E402

# ---- soft numpy check (missing is OK: warning only) -----------------------
try:
    import numpy as _np  # noqa: F401
    HAVE_NUMPY = True
    NUMPY_VER = getattr(_np, "__version__", "?")
except Exception:
    HAVE_NUMPY = False
    NUMPY_VER = None

G_STD = 9.80665          # standard gravity, m/s^2 (ENU z-up convention)
FSYNC_ROWS = 50          # flush + fsync every N rows
IMU_QUEUE_CAP = 64       # bounded imu queue (drop-oldest + count)
GPS_QUEUE_CAP = 16       # bounded gps queue (drop-oldest + count)
BATCH_LIMIT = 256        # max imu samples processed per drain pass
COAST_TIMEOUT = 3.0      # s since last gated fix -> coast (GPS ~1 Hz)
FIRST_FIX_TIMEOUT = 30.0 # s to wait for the origin fix
SURVEY_IN_SECONDS = 15.0 # GPS survey-in: collect fixes this long after the
                         # first gated fix before setting the origin
SURVEY_IN_MIN_FIXES = 10 # ... and require at least this many fixes
SURVEY_IN_MAX_SECONDS = 30.0  # hard cap on the survey-in window
ZUPT_R_SCALE = 25.0   # inflate GPS position R by this factor while the
                      # position hold is active (see hold gate below): the
                      # on-board GPS wanders in a ~10 m envelope with 3-4 m
                      # jumps between adjacent fixes (multipath; hdop ~1 is
                      # misleading); that noise must not move a known-static
                      # position estimate. F3 fix: 9.0 was insufficient -
                      # 6 m multipath jumps stayed inside the chi-square gate
                      # (innovation < 3 sigma at sigma ~9 m) and the fused
                      # position followed the wander (night static_drift
                      # 5.2-6.9 m, threshold 5 m). 25 pins the hold gain near
                      # P0/(P0 + 25^2 * 9) ~ 0.004, so hold fixes barely move
                      # the estimate; horizontal jumps > 2.5 m are rejected
                      # outright by the ekf hold-time multipath guard.
HOLD_VGPS_MAX = 1.2   # m/s: GPS speed bound for the position hold. ZUPT's
                      # velocity gate stays at the frozen 0.5 m/s, but the
                      # hold must survive the receiver's static speed noise
                      # (measured 0.53-1.10 m/s spikes on a static board;
                      # F3 night runs saw up to 1.47 m/s);
                      # real hook motion is well above this bound.
                      # Spike-robust: the gate uses the MEDIAN of the last
                      # HOLD_SPD_WINDOW fix speeds (see hold speed gate in
                      # process_sample) so an isolated noisy fix does not
                      # drop the hold and re-admit full-weight updates.
HOLD_SPD_WINDOW = 5   # fixes: speed median window for the hold speed gate
HOLD_SPD_MIN_FIXES = 3  # fixes: fall back to the instantaneous speed below this
RATE_FLOOR = 40.0        # Hz: sample-rate guard threshold
RATE_GUARD_WINDOW = 5.0  # s: sustained window for the guard
MAX_BACKLOG = 10         # max serial-buffer backlog (samples) honoured for backdating
EXPECTED_COLS = 28

HEADER = [
    "t_mono",
    "gps_lat", "gps_lon", "gps_alt", "gps_fix", "gps_hdop",
    "gps_speed_ms", "gps_course_deg",
    "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz", "imu_sat",
    "att_roll", "att_pitch", "att_yaw",
    "fused_n", "fused_e", "fused_u", "fused_vn", "fused_ve", "fused_vu",
    "fused_lat", "fused_lon", "fused_alt",
    "zupt",
]

# Board IMU config: WitMotion WT901SDCL over UART (replaces the MPU6050
# on i2c-2 @ 0x68). See imu_reader.py.
IMU_CONFIG = {
    "model": "WitMotion WT901SDCL",
    "interface": "uart",
    "accel_range": "16g", "gyro_range": "2000dps",
    "output_rate_hz": 50,
}


class DropQueue(object):
    """Bounded FIFO with drop-oldest policy and an explicit drop counter."""

    def __init__(self, capacity):
        self.cap = int(capacity)
        self._lock = threading.Lock()
        self._dq = collections.deque()
        self.dropped = 0

    def put(self, item):
        with self._lock:
            if len(self._dq) >= self.cap:
                self._dq.popleft()      # drop oldest
                self.dropped += 1
            self._dq.append(item)

    def drain(self, limit=None):
        with self._lock:
            if limit is None:
                items = list(self._dq)
                self._dq.clear()
                return items
            n = min(int(limit), len(self._dq))
            return [self._dq.popleft() for _ in range(n)]


class CSVWriter(object):
    """Writes the frozen 28-col CSV; buffers rows, fsyncs every N rows,
    tracks bytes-of-complete-lines for tail-integrity repair."""

    def __init__(self, path):
        self.path = path
        self.rows = 0
        self._buf = []
        self._fh = open(path, "w", encoding="utf-8", newline="")
        self._fh.write(",".join(HEADER) + "\n")
        self._bytes_ok = self._fh.tell()

    def add(self, fields):
        self._buf.append(",".join(fields) + "\n")
        self.rows += 1
        if len(self._buf) >= FSYNC_ROWS:
            self._flush(fsync=True)

    def _flush(self, fsync=False):
        if self._buf:
            self._fh.write("".join(self._buf))
            self._buf = []
        self._fh.flush()
        if fsync:
            os.fsync(self._fh.fileno())
        self._bytes_ok = self._fh.tell()

    def close(self):
        try:
            self._flush(fsync=True)
        finally:
            self._fh.close()

    def repair_tail(self):
        """If the file grew beyond the last complete-line boundary (partial
        write), truncate back to the last complete line. Returns True when a
        repair was performed."""
        size = os.path.getsize(self.path)
        if size <= self._bytes_ok:
            return False
        with open(self.path, "r+b") as f:
            f.truncate(self._bytes_ok)
        return True

    def last_line_ok(self):
        """Check the final data line has EXPECTED_COLS fields."""
        with open(self.path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 4096))
            tail = f.read().decode("utf-8", errors="replace")
        lines = [ln for ln in tail.split("\n") if ln]
        if not lines:
            return False
        return len(lines[-1].split(",")) == EXPECTED_COLS


class Ctx(object):
    """Fusion/writer shared state (single writer thread owns it)."""

    def __init__(self, imu, gps, att, ekf, imu_q, gps_q, writer,
                 nominal_rate, stop, dt_ema, q_mount_inv=(1.0, 0.0, 0.0, 0.0)):
        self.imu = imu
        self.gps = gps
        self.att = att
        self.ekf = ekf
        self.imu_q = imu_q
        self.gps_q = gps_q
        self.writer = writer
        self.nominal_rate = nominal_rate
        self.stop = stop
        self.stop_reason = "duration"
        self.dt_ema = dt_ema          # shared measured production interval
        self.last_fix = None          # newest gated GPSFix seen
        self.last_fix_t = None        # t_mono of newest gated fix
        self.last_gps_applied_t = None
        self.last_t = None            # t_mono of previous IMU sample
        self.first_t = None
        self.zupt_rows = 0
        self.coast_rows = 0
        self.coast_seconds = 0.0
        self.rate_warnings = 0
        self.backdated = 0
        self.max_backdate = 0.0
        self.ts_deque = collections.deque()
        self.last_guard_check = 0.0
        self.last_rate_warn = -1e9
        self.last_status = 0.0
        self.q_mount_inv = q_mount_inv  # mount-relative attitude reference
        self.spd_win = collections.deque(maxlen=HOLD_SPD_WINDOW)  # recent fix speeds


# ------------------------------------------------------------------ helpers
def _qmul(a, b):
    """Hamilton quaternion product (a,b) = (w,x,y,z) tuples."""
    a1, a2, a3, a4 = a
    b1, b2, b3, b4 = b
    return (a1 * b1 - a2 * b2 - a3 * b3 - a4 * b4,
            a1 * b2 + a2 * b1 + a3 * b4 - a4 * b3,
            a1 * b3 - a2 * b4 + a3 * b1 + a4 * b2,
            a1 * b4 + a2 * b3 - a3 * b2 + a4 * b1)


def _rpy_from_quat(q):
    """ZYX roll/pitch/yaw (rad) of a Hamilton quaternion (matches
    attitude.Attitude.roll_pitch_yaw conventions)."""
    q1, q2, q3, q4 = q
    roll = math.atan2(2.0 * (q3 * q4 + q1 * q2),
                      1.0 - 2.0 * (q2 * q2 + q3 * q3))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q1 * q3 - q2 * q4))))
    yaw = math.atan2(2.0 * (q2 * q3 + q1 * q4),
                     1.0 - 2.0 * (q3 * q3 + q4 * q4))
    return roll, pitch, yaw


def _median(vals):
    s = sorted(vals)
    return s[len(s) // 2]


# ------------------------------------------------------------------- threads
def gps_thread(gps, q, stop):
    while not stop.is_set():
        try:
            fix = gps.read_fix()          # blocks <= gps.timeout (10 s)
        except Exception as exc:          # noqa: BLE001
            print("GPS thread error: %s" % exc, file=sys.stderr, flush=True)
            time.sleep(1.0)
            continue
        if fix is not None:
            q.put(fix)


def imu_thread(imu, q, stop, dt_ema):
    """Read IMU samples; annotate each with the serial-buffer backlog
    (bytes left after the read, converted to samples) so the fusion thread
    can backdate late stamps. dt_ema is a robust MEDIAN of recent raw
    inter-read intervals (immune to the intermittent serial-read stalls
    and catch-up bursts)."""
    prev_raw = None
    win = collections.deque(maxlen=256)
    while not stop.is_set():
        try:
            s = imu.read_sample()         # blocks ~20 ms @50 Hz
        except Exception as exc:          # noqa: BLE001
            print("IMU thread error: %s" % exc, file=sys.stderr, flush=True)
            time.sleep(0.05)
            continue
        k = 0
        try:
            rem = imu._fifo_count()       # bytes remaining after this read
            if rem >= imu_reader.SAMPLE_BYTES:
                k = min(rem // imu_reader.SAMPLE_BYTES, MAX_BACKLOG)
        except Exception:                 # noqa: BLE001
            k = 0
        if prev_raw is not None:
            win.append(s.t_mono - prev_raw)
            if len(win) >= 32:
                med = sorted(win)[len(win) // 2]
                if 0.015 < med < 0.030:
                    dt_ema[0] = med
        prev_raw = s.t_mono
        q.put((s, k))


# ---------------------------------------------------------------- fusion loop
def process_sample(item, ctx):
    """Fuse one IMU sample and emit one CSV row.

    item = (IMUSample, backlog_k). The stamp is backdated by
    k * dt_ema (serial-buffer-backlog timestamp reconstruction, see module
    docstring); dt is the MEASURED production interval, never a hardcoded
    0.02 s."""
    s, k = item
    dte = ctx.dt_ema[0]                  # measured production interval
    t_raw = s.t_mono
    t_out = t_raw - k * dte
    if ctx.last_t is None:
        ctx.first_t = t_out
        dt = dte
    else:
        dt = t_out - ctx.last_t
        if not (math.isfinite(dt) and dt > 1e-6):
            dt = dte
            t_out = ctx.last_t + dt   # keep t_mono strictly increasing
        elif dt > 0.5:
            dt = 0.5                  # clock glitch guard
    ctx.last_t = t_out
    t = t_out
    if k > 0:
        ctx.backdated += 1
        if k * dte > ctx.max_backdate:
            ctx.max_backdate = k * dte

    ax, ay, az = s.ax, s.ay, s.az      # already bias-corrected by imu_reader
    gx, gy, gz = s.gx, s.gy, s.gz
    a_mag = math.sqrt(ax * ax + ay * ay + az * az)
    w_mag = math.sqrt(gx * gx + gy * gy + gz * gz)

    coast = (ctx.last_fix_t is None) or (t - ctx.last_fix_t > COAST_TIMEOUT)

    # attitude -> a_nav (ENU) -> EKF predict
    ctx.att.update((gx, gy, gz), (ax, ay, az), dt)
    R = ctx.att.rotation_matrix()
    a_nav = (R[0][0] * ax + R[0][1] * ay + R[0][2] * az,
             R[1][0] * ax + R[1][1] * ay + R[1][2] * az,
             R[2][0] * ax + R[2][1] * ay + R[2][2] * az - G_STD)
    ctx.ekf.predict(dt, a_nav)

    fix = ctx.last_fix
    v_gps_h = None if coast else (fix.speed_ms if fix is not None else None)
    zupt_active = ctx.ekf.zupt_detect(v_gps_h, a_mag, w_mag)
    # position hold gate: IMU-static AND GPS speed below the noise floor.
    # Wider than the ZUPT velocity gate on purpose (see HOLD_VGPS_MAX), and
    # spike-robust: the median of the last few FIX speeds must be below
    # HOLD_VGPS_MAX so an isolated speed-noise spike (night runs: up to
    # 1.47 m/s on a static board) cannot drop the hold and re-admit
    # full-weight multipath jumps. Before enough fixes are seen, fall back
    # to the instantaneous speed.
    if v_gps_h is None:
        ok_hold_gps = True
    elif len(ctx.spd_win) >= HOLD_SPD_MIN_FIXES:
        med = sorted(ctx.spd_win)[len(ctx.spd_win) // 2]
        ok_hold_gps = math.isfinite(med) and med < HOLD_VGPS_MAX
    else:
        ok_hold_gps = math.isfinite(v_gps_h) and abs(v_gps_h) < HOLD_VGPS_MAX
    hold_active = (9.5 <= a_mag <= 10.1) and (w_mag < 0.1) and ok_hold_gps
    if not hold_active:
        # motion: drop the speed window so the next hold episode restarts
        # from clean (instantaneous) evidence
        ctx.spd_win.clear()

    if fix is not None and fix.t_mono != ctx.last_gps_applied_t:
        n, e, u = geo.latlonalt_to_enu(fix.lat, fix.lon, fix.alt_m)
        # position hold while static: trust the IMU static verdict, discount
        # GPS position noise (see ZUPT_R_SCALE / HOLD_VGPS_MAX) and let the
        # EKF reject multipath horizontal jumps via the hold flag.
        ctx.ekf.update_gps(n, e, u,
                           r_scale=ZUPT_R_SCALE if hold_active else 1.0,
                           hold=hold_active)
        spd = fix.speed_ms
        if spd > 0.1:                  # avoid redundant zero-vel obs
            crs = math.radians(fix.course_deg)
            ctx.ekf.update_vel(spd * math.cos(crs), spd * math.sin(crs), 0.0)
        ctx.last_gps_applied_t = fix.t_mono

    if hold_active:
        # Zero-velocity constraint whenever the IMU is static (hold gate),
        # independent of the frozen 0.5 m/s ZUPT velocity gate: static GPS
        # speed noise (0.5-1.4 m/s spikes) otherwise feeds update_vel and
        # integrates into position drift (~5 m over 60 s measured on-board).
        # The CSV zupt flag keeps the frozen ZUPT verdict (zupt_active).
        ctx.ekf.zupt()
    if coast:
        zupt_flag = 2
        ctx.coast_rows += 1
        ctx.coast_seconds += dt
    elif zupt_active:
        zupt_flag = 1
        ctx.zupt_rows += 1
    else:
        zupt_flag = 0

    st = ctx.ekf.state()
    fn, fe, fu = st[0], st[1], st[2]
    fvn, fve, fvu = st[3], st[4], st[5]
    flat, flon, falt = geo.enu_to_latlonalt(fn, fe, fu)
    # mount-relative attitude (see module docstring): static -> ~0 deg
    q_rel = _qmul(ctx.q_mount_inv, ctx.att.quat())
    roll, pitch, yaw = _rpy_from_quat(q_rel)

    if coast:
        gps_fields = ("", "", "", "0", "", "", "")
    else:
        gps_fields = (
            "%.7f" % fix.lat, "%.7f" % fix.lon, "%.3f" % fix.alt_m,
            str(int(fix.fix)), "%.2f" % fix.hdop,
            "%.3f" % fix.speed_ms, "%.2f" % fix.course_deg)

    row = (["%.6f" % t] + list(gps_fields) +
           ["%.5f" % ax, "%.5f" % ay, "%.5f" % az,
            "%.5f" % gx, "%.5f" % gy, "%.5f" % gz,
            "0",                       # imu_sat: GPSFix has no sat count
            "%.6f" % roll, "%.6f" % pitch, "%.6f" % yaw,
            "%.4f" % fn, "%.4f" % fe, "%.4f" % fu,
            "%.4f" % fvn, "%.4f" % fve, "%.4f" % fvu,
            "%.7f" % flat, "%.7f" % flon, "%.3f" % falt,
            str(zupt_flag)])
    ctx.writer.add(row)

    # ---- sample-rate guard: sustained < RATE_FLOOR Hz -> stderr warning ----
    # (uses raw read times: reflects real production, not reconstruction)
    ctx.ts_deque.append(t_raw)
    while ctx.ts_deque and ctx.ts_deque[0] < t_raw - RATE_GUARD_WINDOW:
        ctx.ts_deque.popleft()
    now = time.monotonic()
    if now - ctx.last_guard_check >= 1.0 and len(ctx.ts_deque) >= 2:
        ctx.last_guard_check = now
        span = ctx.ts_deque[-1] - ctx.ts_deque[0]
        if span > 0.5:
            inst_rate = (len(ctx.ts_deque) - 1) / span
            if (inst_rate < RATE_FLOOR
                    and now - ctx.last_rate_warn >= RATE_GUARD_WINDOW):
                print("WARN: IMU sample rate %.1f Hz < %.0f Hz sustained %ds"
                      % (inst_rate, RATE_FLOOR, RATE_GUARD_WINDOW),
                      file=sys.stderr, flush=True)
                ctx.rate_warnings += 1
                ctx.last_rate_warn = now


def run_recording(ctx, duration, t_start_mono, imu_thr):
    end_t = t_start_mono + duration

    while not ctx.stop.is_set():
        now = time.monotonic()
        if now >= end_t:
            ctx.stop_reason = "duration"
            break
        for fix in ctx.gps_q.drain():
            ctx.last_fix = fix
            ctx.last_fix_t = fix.t_mono
            ctx.spd_win.append(fix.speed_ms)
        batch = ctx.imu_q.drain(limit=BATCH_LIMIT)
        if not batch:
            time.sleep(0.001)
            continue
        for s in batch:
            process_sample(s, ctx)
        if now - ctx.last_status >= 10.0:
            ctx.last_status = now
            gps_age = (now - ctx.last_fix_t) if ctx.last_fix_t else -1.0
            print("status: t=%.1fs rows=%d gps_age=%.1fs coast=%d zupt=%d "
                  "dropped(imu=%d,gps=%d) backdated=%d"
                  % (now - t_start_mono, ctx.writer.rows, gps_age,
                     ctx.coast_rows, ctx.zupt_rows,
                     ctx.imu_q.dropped, ctx.gps_q.dropped, ctx.backdated),
                  flush=True)

    # graceful stop: stop threads, then drain what they already produced
    ctx.stop.set()
    imu_thr.join(timeout=2.0)
    for fix in ctx.gps_q.drain():
        ctx.last_fix = fix
        ctx.last_fix_t = fix.t_mono
        ctx.spd_win.append(fix.speed_ms)
    for s in ctx.imu_q.drain():
        process_sample(s, ctx)


# --------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="3-thread GPS+IMU+EKF trajectory recorder "
                    "(28-col CSV + .meta.json)")
    ap.add_argument("--duration", type=float, default=1800.0,
                    help="planned recording duration in seconds (default 1800)")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="nominal IMU sample rate in Hz (default 50)")
    ap.add_argument("--out", default="/root/trajectory/runs/",
                    help="output directory for CSV + meta.json "
                         "(default /root/trajectory/runs/)")
    args = ap.parse_args(argv)

    stop = threading.Event()
    sig_reason = {"reason": "duration"}

    def _sig(signum, frame):  # noqa: ARG001
        sig_reason["reason"] = "signal(%d)" % signum
        print("\nsignal %d received - stopping gracefully" % signum,
              file=sys.stderr, flush=True)
        stop.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    print("recorder.py starting: duration=%.1fs nominal_rate=%.1fHz out=%s "
          "python=%s numpy=%s"
          % (args.duration, args.rate, args.out,
             sys.version.split()[0],
             NUMPY_VER if HAVE_NUMPY else "MISSING"), flush=True)
    if not HAVE_NUMPY:
        print("WARN: numpy not available - recording uses stdlib math only "
              "(this is fine, no numpy needed)", file=sys.stderr, flush=True)

    os.makedirs(args.out, exist_ok=True)

    try:
        # 1) GPS reader + thread first so fix collection overlaps calibration
        print("opening GPS %s ..." % gps_reader.GPS_DEVICE, flush=True)
        gps = gps_reader.GPSReader()

        gps_q = DropQueue(GPS_QUEUE_CAP)
        imu_q = DropQueue(IMU_QUEUE_CAP)
        gps_thr = threading.Thread(target=gps_thread,
                                   args=(gps, gps_q, stop),
                                   daemon=True, name="gps")
        gps_thr.start()

        # 2) IMU open + 10 s static calibration (GPS fixes collected meanwhile)
        print("opening IMU (WitMotion WT901SDCL serial) ...", flush=True)
        imu = imu_reader.IMUReader()
        print("calibrating IMU (10 s, KEEP STATIC) ...", flush=True)
        cal = imu.calibrate(10.0)
        print("calibrated: gyro_bias=%s rad/s accel_bias=%s m/s^2 "
              "mount_gravity(unit)=%s n=%d"
              % (["%.4f" % v for v in cal["gyro_bias"]],
                 ["%.4f" % v for v in cal["accel_bias"]],
                 ["%.3f" % v for v in cal["mount_gravity"]],
                 cal["n_samples"]), flush=True)

        # measured production interval from the calibration sample count
        # (dt_ema[0] is refined online by the IMU thread; never hardcoded)
        dt0 = (cal["duration_s"] / max(1, cal["n_samples"] - 1)
               if cal["n_samples"] > 1 else 1.0 / args.rate)
        dt_ema = [min(max(dt0, 0.015), 0.030)]

        # Start the IMU thread NOW, before the first-fix wait / survey-in.
        # The WitMotion WT901SDCL streams over USB serial: its kernel input
        # buffer (~4 KB) also backs up while nothing reads, but no sample is
        # lost (the kernel buffer is large and the read thread drains it
        # continuously). The thread keeps draining the serial buffer through
        # the idle survey window (preventing buffer backlog from delaying
        # read timestamps); its queue is discarded and drop counters are
        # reset before recording starts.
        imu_thr = threading.Thread(target=imu_thread,
                                   args=(imu, imu_q, stop, dt_ema),
                                   daemon=True, name="imu")
        imu_thr.start()

        # 3) first gated GPS fix -> wait for it (survey-in follows below)
        print("waiting for first gated GPS fix (timeout %.0f s) ..."
              % FIRST_FIX_TIMEOUT, flush=True)
        first_fix = None
        t_wait = time.monotonic()
        while (time.monotonic() - t_wait < FIRST_FIX_TIMEOUT
               and not stop.is_set()):
            for fix in gps_q.drain():
                first_fix = fix
            imu_q.drain()      # keep the serial buffer drained (discarded)
            if first_fix is not None:
                break
            time.sleep(0.05)
        if first_fix is None:
            print("ERROR: no gated GPS fix within %.0f s"
                  % FIRST_FIX_TIMEOUT, file=sys.stderr, flush=True)
            stop.set()
            return 2

        # 3b) GPS survey-in: the receiver's solution wanders for the first
        # ~10-20 s after its first fix (on-board observation: fixes drifted
        # +8 m east / +3.6 m up over ~8 s before settling). Anchoring the ENU
        # origin to the first fix bakes that convergence transient into the
        # whole trajectory (7.27 m "static drift" on a 25 s run with the GPS
        # itself solid to +-0.7 m after settling). Collect fixes over a
        # survey-in window and set the origin to the element-wise MEDIAN
        # (robust to remaining outliers / multipath spikes).
        print("GPS survey-in: collecting fixes (%.0f s, >= %d fixes) ..."
              % (SURVEY_IN_SECONDS, SURVEY_IN_MIN_FIXES), flush=True)
        survey_fixes = [first_fix]
        t_survey = time.monotonic()
        while not stop.is_set():
            for fix in gps_q.drain():
                survey_fixes.append(fix)
            imu_q.drain()      # keep the serial buffer drained (discarded)
            elapsed = time.monotonic() - t_survey
            if elapsed >= SURVEY_IN_SECONDS \
                    and len(survey_fixes) >= SURVEY_IN_MIN_FIXES:
                break
            if elapsed >= SURVEY_IN_MAX_SECONDS:
                print("WARN: survey-in reached %.0f s cap with %d fixes"
                      % (SURVEY_IN_MAX_SECONDS, len(survey_fixes)),
                      file=sys.stderr, flush=True)
                break
            time.sleep(0.05)
        if len(survey_fixes) < 3:   # degenerate case: fall back to first fix
            survey_fixes = [first_fix] * 3
        lat_med = _median([f.lat for f in survey_fixes])
        lon_med = _median([f.lon for f in survey_fixes])
        alt_med = _median([f.alt_m for f in survey_fixes])
        ok_first = geo.set_origin(lat_med, lon_med, alt_med)
        # spread: max horizontal distance of survey fixes from the median
        spread_h = 0.0
        for f in survey_fixes:
            n, e, u = geo.latlonalt_to_enu(f.lat, f.lon, f.alt_m)
            spread_h = max(spread_h, math.hypot(n, e))
        survey_meta = {
            "n_fixes": len(survey_fixes),
            "duration_s": round(time.monotonic() - t_survey, 2),
            "spread_h_m": round(spread_h, 2),
            "origin_method": "median",
        }
        print("origin set (survey-in median of %d fixes over %.1f s, "
              "spread %.2f m): (%.7f, %.7f, %.2fm) first_set=%s"
              % (len(survey_fixes), survey_meta["duration_s"],
                 spread_h, lat_med, lon_med, alt_med, ok_first), flush=True)

        # 4) fusion state + output files
        # Pre-align the attitude from the measured mount gravity direction:
        # starting Madgwick from identity at this ~83 deg mount tilt takes
        # ~25-30 s to converge (beta=0.05), during which a_nav is wrong and
        # velocity runs away. q0 maps body gravity dir -> earth +z.
        mgv = cal["mount_gravity"]
        _dot = max(-1.0, min(1.0, mgv[2]))
        if _dot > 0.9999:
            q0 = (1.0, 0.0, 0.0, 0.0)
        elif _dot < -0.9999:
            q0 = (0.0, 1.0, 0.0, 0.0)
        else:
            _n = math.hypot(mgv[1], mgv[0])
            _ax, _ay = mgv[1] / _n, -mgv[0] / _n
            _ang = math.acos(_dot)
            _s = math.sin(_ang / 2.0)
            q0 = (math.cos(_ang / 2.0), _ax * _s, _ay * _s, 0.0)
        att = att_mod.Attitude(q0=q0, init_bias=(0.0, 0.0, 0.0))
        # Position prior: origin is now a survey-in MEDIAN of >=10 fixes
        # (spread ~2 m), so the T6-era 10 m std for a single bad first fix
        # is oversized; 5 m std still exceeds the residual uncertainty.
        ekf = ekf_mod.EKF9(p0_pos=(5.0, 5.0, 8.0))
        # mount-relative attitude reference: q_mount == q0 (body gravity dir
        # -> earth +z, zero yaw). q_rel = q_mount^-1 * q_att reads ~0 deg
        # while the hook hangs static.
        q_mount_inv = (q0[0], -q0[1], -q0[2], -q0[3])
        stem = datetime.datetime.fromtimestamp(
            time.time()).strftime("%Y%m%d-%H%M%S")
        csv_path = os.path.join(args.out, stem + ".csv")
        meta_path = os.path.join(args.out, stem + ".meta.json")
        writer = CSVWriter(csv_path)
        print("recording -> %s (dt_ema=%.4fms)" % (csv_path, dt_ema[0] * 1000),
              flush=True)

        ctx = Ctx(imu, gps, att, ekf, imu_q, gps_q, writer, args.rate, stop,
                  dt_ema, q_mount_inv=q_mount_inv)
        ctx.last_fix = survey_fixes[-1]
        ctx.last_fix_t = survey_fixes[-1].t_mono
        # survey-era queue drops are startup artefacts, not recording losses
        imu_q.dropped = 0
        gps_q.dropped = 0

        t_start_mono = time.monotonic()
        t_start_wall = time.time()
        run_recording(ctx, args.duration, t_start_mono, imu_thr)
        t_stop_wall = time.time()

        # 5) graceful shutdown: flush, fsync, tail integrity, meta
        writer.close()
        repaired = writer.repair_tail()
        tail_ok = writer.last_line_ok()
        print("tail check: repaired=%s last_line_ok=%s (rows=%d)"
              % (repaired, tail_ok, writer.rows), flush=True)

        span = (ctx.last_t - ctx.first_t) if (ctx.last_t is not None
                                              and ctx.first_t is not None) \
            else 0.0
        rows = writer.rows
        rate = ((rows - 1) / span) if (rows > 1 and span > 0) else 0.0

        def _iso(ts):
            return datetime.datetime.fromtimestamp(ts).astimezone().isoformat(
                timespec="milliseconds")

        mg = cal["mount_gravity"]
        meta = {
            "origin": {"lat": lat_med, "lon": lon_med, "alt": alt_med},
            "survey_in": survey_meta,
            "gyro_bias": {"x": cal["gyro_bias"][0], "y": cal["gyro_bias"][1],
                          "z": cal["gyro_bias"][2]},
            "accel_bias": {"x": cal["accel_bias"][0],
                           "y": cal["accel_bias"][1],
                           "z": cal["accel_bias"][2]},
            "mount_gravity": {"x": mg[0] * G_STD, "y": mg[1] * G_STD,
                              "z": mg[2] * G_STD},
            "calibration": {"n_samples": cal["n_samples"],
                            "duration_s": cal["duration_s"]},
            "sample_rate": round(rate, 3),
            "duration": round(span, 3),
            "planned_duration": args.duration,
            "nominal_rate": args.rate,
            "imu": dict(IMU_CONFIG),
            "gps": {"port": gps.device,
                    "baud": gps._ser_baud or gps_reader.BAUD_CANDIDATES[0],
                    "fix_min": 1, "hdop_max": 2.0},
            "start_time": _iso(t_start_wall),
            "stop_time": _iso(t_stop_wall),
            "rows": rows,
            "dropped": {"imu": imu_q.dropped, "gps": gps_q.dropped},
            "coast": {"rows": ctx.coast_rows,
                      "seconds": round(ctx.coast_seconds, 3),
                      "gps_rejected_events": gps.coast_events},
            "zupt": {"active_rows": ctx.zupt_rows},
            "multipath_rejected": ekf.multipath_rejected,
            "backdated": {"samples": ctx.backdated,
                          "max_backdate_ms": round(ctx.max_backdate * 1000, 1),
                          "dt_ema_ms": round(dt_ema[0] * 1000, 3)},
            "rate_warnings": ctx.rate_warnings,
            "numpy": HAVE_NUMPY,
            "stop_reason": sig_reason["reason"],
        }
        with open(meta_path, "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)

        print("DONE: rows=%d span=%.3fs rate=%.3fHz coast_rows=%d "
              "zupt_rows=%d multipath_rejected=%d dropped(imu=%d,gps=%d) "
              "backdated=%d stop_reason=%s"
              % (rows, span, rate, ctx.coast_rows, ctx.zupt_rows,
                 ekf.multipath_rejected, imu_q.dropped, gps_q.dropped,
                 ctx.backdated, meta["stop_reason"]),
              flush=True)
        print("csv=%s" % csv_path, flush=True)
        print("meta=%s" % meta_path, flush=True)
        return 0
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr, flush=True)
        return 130
    except Exception as exc:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        print("ERROR: %s" % exc, file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())

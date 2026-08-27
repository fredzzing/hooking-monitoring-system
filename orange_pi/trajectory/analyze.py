#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze.py — Physical-quantity assertions over a trajectory CSV.

Reads the frozen 28-column CSV written by recorder.py (see CSV_SCHEMA.md) and
computes physical metrics with machine-readable PASS/FAIL assertions for
scripted validation. No plots, no sample cherry-picking, no post-processing.

Metrics (--metric):
  report           duration / samples / mean rate / dropped samples (from
                   sibling .meta.json) / coast segment count / coordinate ranges
  static_drift     end-start position: horizontal < 5 m, vertical < 8 m,
                   end speed < 0.1 m/s  (ZUPT effectiveness proof)
  attitude_static  mean |roll| and |pitch| over static (zupt==1) rows < 1 deg
  swing_amplitude  amplitude / period features from IMU accel & fused velocity
                   (informational, no threshold)
  hoist_delta      fused_u displacement vs --expect_delta within --tolerance
                   (default 1.5 m)

Output: `PASS/FAIL: metric=value (expect ..., tolerance ...)` on stdout.
Exit codes: 0 = PASS, 1 = assertion FAIL, 2 = usage/IO error.
"""

import argparse
import csv
import json
import math
import os
import sys

EXPECTED_COLUMNS = [
    "t_mono", "gps_lat", "gps_lon", "gps_alt", "gps_fix", "gps_hdop",
    "gps_speed_ms", "gps_course_deg",
    "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz", "imu_sat",
    "att_roll", "att_pitch", "att_yaw",
    "fused_n", "fused_e", "fused_u",
    "fused_vn", "fused_ve", "fused_vu",
    "fused_lat", "fused_lon", "fused_alt",
    "zupt",
]

DEG = 180.0 / math.pi

# Metis acceptance thresholds (frozen, see plan Task 8 / Definition of Done)
STATIC_DRIFT_H = 5.0    # m, horizontal drift over static run
STATIC_DRIFT_V = 8.0    # m, vertical drift over static run (z axis weakest)
STATIC_SPEED = 0.1      # m/s, end speed of static run
ATTITUDE_DEG = 1.0      # deg, mean |roll| / |pitch| over static rows


def _to_float(s):
    s = (s or "").strip()
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def load_csv(path):
    """Load frozen-schema CSV; return (name->index map, rows as list of lists)."""
    if not os.path.isfile(path):
        sys.stderr.write("ERROR: file not found: %s\n" % path)
        sys.exit(2)
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            sys.stderr.write("ERROR: empty CSV\n")
            sys.exit(2)
        header = [h.strip() for h in header]
        if len(header) != len(EXPECTED_COLUMNS):
            sys.stderr.write("ERROR: expected %d columns, got %d\n"
                             % (len(EXPECTED_COLUMNS), len(header)))
            sys.exit(2)
        missing = [c for c in EXPECTED_COLUMNS if c not in header]
        if missing:
            sys.stderr.write("ERROR: CSV header missing columns: %s\n"
                             % ",".join(missing))
            sys.exit(2)
        idx = {name: i for i, name in enumerate(header)}
        rows = []
        for line in reader:
            if not line:
                continue
            rows.append([_to_float(v) for v in line])
    return idx, rows


def _col(rows, idx, name):
    return [r[idx[name]] for r in rows]


def _need_rows(rows, n=2):
    if len(rows) < n:
        sys.stderr.write("ERROR: need >= %d data rows, got %d\n" % (n, len(rows)))
        sys.exit(2)


def metric_static_drift(rows, idx):
    _need_rows(rows)
    a, b = rows[0], rows[-1]
    dn = b[idx["fused_n"]] - a[idx["fused_n"]]
    de = b[idx["fused_e"]] - a[idx["fused_e"]]
    du = b[idx["fused_u"]] - a[idx["fused_u"]]
    horizontal = math.hypot(dn, de)
    vertical = abs(du)
    end_speed = math.sqrt(b[idx["fused_vn"]] ** 2
                          + b[idx["fused_ve"]] ** 2
                          + b[idx["fused_vu"]] ** 2)
    ok_h = horizontal < STATIC_DRIFT_H
    ok_v = vertical < STATIC_DRIFT_V
    ok_s = end_speed < STATIC_SPEED
    ok = ok_h and ok_v and ok_s
    print("%s: static_drift horizontal=%.3fm vertical=%.3fm end_speed=%.3fm/s "
          "(expect horizontal<%.1fm vertical<%.1fm end_speed<%.1fm/s)"
          % ("PASS" if ok else "FAIL", horizontal, vertical, end_speed,
             STATIC_DRIFT_H, STATIC_DRIFT_V, STATIC_SPEED))
    return 0 if ok else 1


def metric_attitude_static(rows, idx):
    _need_rows(rows, 1)
    static_rows = [r for r in rows if int(r[idx["zupt"]]) == 1]
    if not static_rows:
        static_rows = rows  # no ZUPT flags present -> fall back to all rows
    mean_abs_roll = sum(abs(r[idx["att_roll"]]) for r in static_rows) \
        / len(static_rows) * DEG
    mean_abs_pitch = sum(abs(r[idx["att_pitch"]]) for r in static_rows) \
        / len(static_rows) * DEG
    ok = mean_abs_roll < ATTITUDE_DEG and mean_abs_pitch < ATTITUDE_DEG
    print("%s: attitude_static mean|roll|=%.3fdeg mean|pitch|=%.3fdeg "
          "(expect <%.1fdeg, static_rows=%d)"
          % ("PASS" if ok else "FAIL", mean_abs_roll, mean_abs_pitch,
             ATTITUDE_DEG, len(static_rows)))
    return 0 if ok else 1


def metric_swing_amplitude(rows, idx):
    _need_rows(rows, 2)
    ax = _col(rows, idx, "imu_ax")
    ay = _col(rows, idx, "imu_ay")
    az = _col(rows, idx, "imu_az")
    a_mag = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(ax, ay, az)]
    mean_a = sum(a_mag) / len(a_mag)
    a_dyn = [v - mean_a for v in a_mag]
    pp = max(a_dyn) - min(a_dyn)
    amp = pp / 2.0
    rms = math.sqrt(sum(v * v for v in a_dyn) / len(a_dyn))
    crossings = 0
    prev = a_dyn[0]
    for v in a_dyn[1:]:
        if (prev < 0.0 <= v) or (v < 0.0 <= prev):
            crossings += 1
        prev = v
    dt_span = rows[-1][idx["t_mono"]] - rows[0][idx["t_mono"]]
    period = (2.0 * dt_span / crossings) if crossings > 0 else float("nan")
    gx = _col(rows, idx, "imu_gx")
    gy = _col(rows, idx, "imu_gy")
    gz = _col(rows, idx, "imu_gz")
    gyro_peak = max(math.sqrt(x * x + y * y + z * z)
                    for x, y, z in zip(gx, gy, gz))
    vn = _col(rows, idx, "fused_vn")
    ve = _col(rows, idx, "fused_ve")
    hspeed_peak = max(math.hypot(x, y) for x, y in zip(vn, ve))
    period_s = "%.3f" % period if not math.isnan(period) else "n/a"
    print("PASS: swing_amplitude accel_amp=%.4fm/s^2 accel_rms=%.4fm/s^2 "
          "period_est=%ss gyro_peak=%.4frad/s horiz_speed_peak=%.4fm/s "
          "(informational, no threshold)"
          % (amp, rms, period_s, gyro_peak, hspeed_peak))
    return 0


def metric_hoist_delta(rows, idx, expect, tol):
    _need_rows(rows)
    delta = rows[-1][idx["fused_u"]] - rows[0][idx["fused_u"]]
    err = abs(delta - expect)
    ok = err <= tol
    print("%s: hoist_delta delta=%.3fm (expect_delta=%.1fm tolerance=%.1fm, "
          "err=%.3fm)" % ("PASS" if ok else "FAIL", delta, expect, tol, err))
    return 0 if ok else 1


def _rng(rows, idx, name):
    vals = _col(rows, idx, name)
    return min(vals), max(vals)


def metric_report(rows, idx, path):
    _need_rows(rows, 2)
    n = len(rows)
    t0 = rows[0][idx["t_mono"]]
    t1 = rows[-1][idx["t_mono"]]
    duration = t1 - t0
    mean_rate = (n - 1) / duration if duration > 0 else float("nan")
    dropped = None
    meta_path = os.path.splitext(path)[0] + ".meta.json"
    if os.path.isfile(meta_path):
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            rate = meta.get("sample_rate")
            dur = meta.get("duration")
            if rate and dur:
                expected = int(round(float(dur) * float(rate))) + 1
                dropped = max(0, expected - n)
        except (ValueError, OSError):
            dropped = None
    coast = 0
    in_coast = False
    for r in rows:
        is_coast = int(r[idx["zupt"]]) == 2
        if is_coast and not in_coast:
            coast += 1
        in_coast = is_coast
    n_min, n_max = _rng(rows, idx, "fused_n")
    e_min, e_max = _rng(rows, idx, "fused_e")
    u_min, u_max = _rng(rows, idx, "fused_u")
    lat_min, lat_max = _rng(rows, idx, "fused_lat")
    lon_min, lon_max = _rng(rows, idx, "fused_lon")
    alt_min, alt_max = _rng(rows, idx, "fused_alt")
    dropped_s = str(dropped) if dropped is not None else "n/a(meta.json absent)"
    print("PASS: report duration=%.3fs samples=%d mean_rate=%.2fHz dropped=%s "
          "coast_segments=%d" % (duration, n, mean_rate, dropped_s, coast))
    print("  fused_n range: min=%.3f max=%.3f span=%.3fm" % (n_min, n_max, n_max - n_min))
    print("  fused_e range: min=%.3f max=%.3f span=%.3fm" % (e_min, e_max, e_max - e_min))
    print("  fused_u range: min=%.3f max=%.3f span=%.3fm" % (u_min, u_max, u_max - u_min))
    print("  fused_lat range: min=%.6f max=%.6f deg" % (lat_min, lat_max))
    print("  fused_lon range: min=%.6f max=%.6f deg" % (lon_min, lon_max))
    print("  fused_alt range: min=%.3f max=%.3f m" % (alt_min, alt_max))
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Physical-quantity assertions over a trajectory CSV "
                    "(frozen 28-column schema, see CSV_SCHEMA.md)")
    ap.add_argument("csv_path", help="trajectory CSV file")
    ap.add_argument("--metric", required=True,
                    choices=["report", "static_drift", "attitude_static",
                             "swing_amplitude", "hoist_delta"],
                    help="metric to compute/assert")
    ap.add_argument("--expect_delta", type=float, default=None,
                    help="expected fused_u displacement in meters "
                         "(required for hoist_delta)")
    ap.add_argument("--tolerance", type=float, default=1.5,
                    help="hoist_delta tolerance in meters (default 1.5)")
    args = ap.parse_args()

    idx, rows = load_csv(args.csv_path)
    if not rows:
        sys.stderr.write("ERROR: no data rows\n")
        sys.exit(2)

    if args.metric == "report":
        code = metric_report(rows, idx, args.csv_path)
    elif args.metric == "static_drift":
        code = metric_static_drift(rows, idx)
    elif args.metric == "attitude_static":
        code = metric_attitude_static(rows, idx)
    elif args.metric == "swing_amplitude":
        code = metric_swing_amplitude(rows, idx)
    else:  # hoist_delta
        if args.expect_delta is None:
            ap.error("--metric hoist_delta requires --expect_delta")
        code = metric_hoist_delta(rows, idx, args.expect_delta, args.tolerance)
    sys.exit(code)


if __name__ == "__main__":
    main()

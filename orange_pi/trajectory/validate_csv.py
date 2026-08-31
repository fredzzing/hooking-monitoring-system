#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_csv.py - Integrity + schema validator for the hook trajectory CSV.
FROZEN project-wide schema; recorder.py MUST write exactly these columns
and analyze.py MUST read exactly these columns. Any schema change requires
full-project sync (validate_csv.py / recorder.py / analyze.py).

FROZEN CSV SCHEMA (28 columns):
  t_mono, gps_lat, gps_lon, gps_alt, gps_fix, gps_hdop, gps_speed_ms, gps_course_deg,
  imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz, imu_sat,
  att_roll, att_pitch, att_yaw,
  fused_n, fused_e, fused_u, fused_vn, fused_ve, fused_vu, fused_lat, fused_lon, fused_alt,
  zupt

Units:
  accel            m/s^2   (imu_ax/ay/az)
  gyro             rad/s   (imu_gx/gy/gz)
  attitude         rad     (att_roll/pitch/yaw)
  ENU trajectory   m       (fused_n/e/u), velocities m/s (fused_vn/ve/vu)
  GPS              deg/deg/m, gps_fix quality flag (0=no fix..2), gps_sat count,
                   gps_hdop (float), gps_speed_ms (m/s), gps_course_deg
  t_mono           s since boot (time.monotonic), STRICTLY increasing
  zupt             integer flag: 0 = moving, 1 = zero-velocity update active,
                   2 = coast (GPS invalid; fused states still valid)

NaN policy:
  gps_lat/gps_lon/gps_alt/gps_hdop/gps_speed_ms/gps_course_deg MAY be empty/NaN
  ONLY when zupt == 2 (coast row). All other columns (t_mono, gps_fix, gps_sat,
  imu_*, att_*, fused_*, zupt) must always be finite numbers.

Sibling metadata file (.meta.json, same stem as CSV, written by recorder.py):
{
  "origin":        {"lat": float, "lon": float, "alt": float},   # first valid GPS fix = ENU origin
  "gyro_bias":     {"x": float, "y": float, "z": float},          # rad/s
  "accel_bias":    {"x": float, "y": float, "z": float},          # m/s^2
  "mount_gravity": {"x": float, "y": float, "z": float},          # measured gravity in body frame (m/s^2)
  "sample_rate":   50.0,
  "duration":      1800.0,                                        # planned seconds
  "imu":           {"model": "WitMotion WT901SDCL", "interface": "uart",
                    "accel_range": "16g", "gyro_range": "2000dps", "output_rate_hz": 50},
  "gps":           {"port": "/dev/ttyACM0", "baud": 115200, "fix_min": 1, "hdop_max": 2.0},
  "start_time":    "2026-08-24T12:00:00.000+08:00"
}
If the sibling .meta.json exists and carries "sample_rate"/"duration", they override
the CLI defaults for the row-count expectation.

Checks performed:
  1. HEADER      - column count and names match frozen schema
  2. ROW COUNT   - rows ~= (t_last - t_first)*rate + 1 within +-tolerance_pct
                   (or meta duration*rate when meta present)
  3. MONOTONIC   - t_mono strictly increasing
  4. RATE/JITTER - mean sample rate within rate +-2 Hz; p95 inter-sample jitter < 5 ms;
                   no gap > 2.5x median interval (detects missing rows / tail truncation)
  5. NON-FINITE  - no NaN/Inf outside the coast-row GPS exception (zupt==2)
  6. ZUPT        - zupt must be integer in {0,1,2}

Exit code 0 = ALL CHECKS PASS; 1 = at least one FAIL. Every FAIL is printed
as "FAIL <check>: <detail>". Use from recorder.py as the accept gate.

Usage:
  python3 validate_csv.py <file.csv> [--rate 50] [--tolerance_pct 1]
"""

import argparse
import csv
import json
import math
import os
import sys

EXPECTED_HEADER = [
    "t_mono",
    "gps_lat", "gps_lon", "gps_alt", "gps_fix", "gps_hdop",
    "gps_speed_ms", "gps_course_deg",
    "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz", "imu_sat",
    "att_roll", "att_pitch", "att_yaw",
    "fused_n", "fused_e", "fused_u", "fused_vn", "fused_ve", "fused_vu",
    "fused_lat", "fused_lon", "fused_alt",
    "zupt",
]
NUM_COLS = len(EXPECTED_HEADER)  # 28

GPS_NUM_COLS = {"gps_lat", "gps_lon", "gps_alt", "gps_hdop",
                "gps_speed_ms", "gps_course_deg"}
INT_FLAG_COLS = {"gps_fix", "imu_sat", "zupt"}
ZUPT_VALID = {0, 1, 2}
MAX_NAV_REPORTS = 10  # cap for per-row NaN reports


def to_float(s):
    """Return float(value) if s parses to a finite number, else None."""
    t = s.strip()
    if t == "":
        return None
    try:
        v = float(t)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def p95(sorted_vals):
    """0.95 quantile (inclusive lower bound) of a sorted list."""
    m = len(sorted_vals)
    if m == 0:
        return 0.0
    return sorted_vals[max(0, int(math.ceil(0.95 * m)) - 1)]


def load_meta(csv_path):
    """Read sibling <stem>.meta.json if present; return dict or None."""
    stem = os.path.splitext(csv_path)[0]
    meta_path = stem + ".meta.json"
    if not os.path.isfile(meta_path):
        return None
    try:
        with open(meta_path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def check_header(header_row, errors):
    if header_row != EXPECTED_HEADER:
        errors.append(
            "HEADER: column count/name mismatch - expected %d cols %s, "
            "got %d cols %s"
            % (NUM_COLS, EXPECTED_HEADER, len(header_row), header_row)
        )
        return False
    print("PASS HEADER: 28 columns match frozen schema")
    return True


def check_row_count(n_rows, t_first, t_last, rate, tol_pct, meta, errors):
    if n_rows == 0:
        errors.append("ROW COUNT: file contains no data rows")
        return
    duration = t_last - t_first
    meta_duration = None
    if meta is not None and isinstance(meta.get("duration"), (int, float)):
        meta_duration = float(meta["duration"])
    if meta_duration is not None and meta_duration > 0:
        expected = max(1, int(round(meta_duration * rate)))
        basis = "meta duration %.1fs" % meta_duration
    else:
        expected = max(1, int(round(duration * rate)) + 1)
        basis = "span %.3fs" % duration
    tol = tol_pct / 100.0 * expected
    diff = abs(n_rows - expected)
    if diff <= tol:
        print(
            "PASS ROW COUNT: %d rows (expected ~%d, +-%d, basis=%s)"
            % (n_rows, expected, int(round(tol)), basis)
        )
    else:
        errors.append(
            "ROW COUNT: got %d rows, expected ~%d (+-%d from %s); "
            "missing/extra %d rows"
            % (n_rows, expected, int(round(tol)), basis, n_rows - expected)
        )


def check_monotonic(ts, errors):
    for i in range(1, len(ts)):
        if not (ts[i] > ts[i - 1]):
            errors.append(
                "MONOTONIC: t_mono not strictly increasing at row %d: "
                "t[%d]=%.9f >= t[%d]=%.9f" % (i, i - 1, ts[i - 1], i, ts[i])
            )
            return
    print(
        "PASS MONOTONIC: t_mono strictly increasing (%d samples, %.3fs span)"
        % (len(ts), ts[-1] - ts[0])
    )


def check_rate_jitter(ts, rate, errors):
    if len(ts) < 2:
        errors.append("RATE/JITTER: need >=2 rows to assess sample rate")
        return
    dt = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    span = ts[-1] - ts[0]
    mean_rate = (len(ts) - 1) / span if span > 0 else 0.0
    if abs(mean_rate - rate) > 2.0:
        errors.append(
            "RATE: mean sample rate %.3f Hz out of %.3f +- 2 Hz" % (mean_rate, rate)
        )
    else:
        print("PASS RATE: mean %.3f Hz (expect %.3f +- 2 Hz)" % (mean_rate, rate))
    ideal = 1.0 / rate
    jit = sorted(abs(d - ideal) for d in dt)
    p95j = p95(jit)
    maxj = jit[-1]
    if p95j >= 0.005:
        errors.append(
            "JITTER: p95 inter-sample jitter %.3f ms >= 5 ms limit" % (p95j * 1000.0)
        )
    else:
        print(
            "PASS JITTER: p95 %.3f ms, max %.3f ms (limit p95 < 5 ms)"
            % (p95j * 1000.0, maxj * 1000.0)
        )
    median_dt = jit[len(jit) // 2] + ideal  # median interval
    max_dt = max(dt)
    if median_dt > 0 and max_dt > 2.5 * median_dt:
        errors.append(
            "GAP: max interval %.1f ms > 2.5x median %.1f ms - missing rows "
            "or tail truncation" % (max_dt * 1000.0, median_dt * 1000.0)
        )
    else:
        print("PASS GAP: no interval exceeds 2.5x median")


def check_finite(rows, errors):
    bad = 0
    first_reports = []
    for r_idx, row in enumerate(rows):
        if len(row) != NUM_COLS:
            if bad < MAX_NAV_REPORTS:
                first_reports.append(
                    "row %d: %d cells (expected %d)" % (r_idx, len(row), NUM_COLS)
                )
            bad += 1
            continue
        zupt_raw = row[EXPECTED_HEADER.index("zupt")]
        zupt = to_float(zupt_raw)
        coast = (zupt is not None and zupt == 2)
        for col_idx, cell in enumerate(row):
            col = EXPECTED_HEADER[col_idx]
            if to_float(cell) is not None:
                continue
            if col in GPS_NUM_COLS and coast:
                continue  # coast row: GPS numeric fields may be empty/NaN
            if bad < MAX_NAV_REPORTS:
                first_reports.append(
                    "row %d col %s: non-finite value %r (zupt=%s, coast_allowed=%s)"
                    % (r_idx, col, cell, zupt_raw, coast)
                )
            bad += 1
    if bad == 0:
        print("PASS NON-FINITE: all required cells are finite numbers")
    else:
        shown = "; ".join(first_reports[:MAX_NAV_REPORTS])
        errors.append(
            "NON-FINITE: %d bad cells found (%s%s)"
            % (bad, shown, "" if bad <= MAX_NAV_REPORTS else "; ...")
        )


def check_zupt(rows, errors):
    bad = 0
    first = None
    for r_idx, row in enumerate(rows):
        if len(row) != NUM_COLS:
            continue
        v = to_float(row[EXPECTED_HEADER.index("zupt")])
        if v is None or int(v) != v or int(v) not in ZUPT_VALID:
            bad += 1
            if first is None:
                first = "row %d zupt=%r (must be integer 0/1/2)" % (r_idx, row[EXPECTED_HEADER.index("zupt")])
    if bad == 0:
        print("PASS ZUPT: zupt flags all in {0,1,2}")
    else:
        errors.append("ZUPT: %d invalid flag(s), e.g. %s" % (bad, first))


def main():
    ap = argparse.ArgumentParser(
        description="Validate hook-trajectory CSV against frozen schema"
    )
    ap.add_argument("csv_file", help="path to CSV file")
    ap.add_argument("--rate", type=float, default=50.0, help="nominal sample rate Hz")
    ap.add_argument("--tolerance_pct", type=float, default=1.0,
                    help="allowed row-count deviation in percent")
    args = ap.parse_args()

    meta = load_meta(args.csv_file)
    if meta is not None:
        mr = meta.get("sample_rate")
        if isinstance(mr, (int, float)) and mr > 0:
            args.rate = float(mr)

    errors = []
    try:
        with open(args.csv_file, "r", encoding="utf-8", newline="") as fh:
            reader = csv.reader(fh)
            try:
                header = next(reader)
            except StopIteration:
                print("FAIL HEADER: empty file")
                return 1
            rows = [r for r in reader]
    except OSError as exc:
        print("FAIL IO: cannot read %s: %s" % (args.csv_file, exc))
        return 1

    ok = check_header(header, errors)

    if ok:
        try:
            ts = [float(r[0]) for r in rows]
            have_ts = True
        except (ValueError, IndexError):
            ts = []
            have_ts = False
            errors.append("ROW COUNT: t_mono not parseable as float in data rows")
        if have_ts and len(ts) == len(rows) and rows:
            check_row_count(len(rows), ts[0], ts[-1], args.rate,
                            args.tolerance_pct, meta, errors)
            check_monotonic(ts, errors)
            check_rate_jitter(ts, args.rate, errors)
        check_finite(rows, errors)
        check_zupt(rows, errors)
    else:
        errors.append("SUBSEQUENT CHECKS SKIPPED: header mismatch")

    if errors:
        print("")
        for e in errors:
            print("FAIL %s" % e)
        print("")
        print("RESULT: %d check(s) FAILED" % len(errors))
        return 1

    print("")
    print("RESULT: ALL CHECKS PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

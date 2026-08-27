#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gps_reader.py - u-blox GNSS NMEA reader for Orange Pi.

Reads NMEA sentences from /dev/ttyACM0 via pyserial, parses $GNGGA / $GNRMC /
$GNVTG with checksum validation, and applies a fix gate: only fix quality >= 1
AND HDOP < 2.0 updates are returned. Rejected samples increment coast_events.

Design notes:
  - Relative trajectory uses time.monotonic(); GPS UTC is NOT used for
    backfill/sync (per plan).
  - Device baud is auto-detected (USB CDC ACM ignores host baud in practice);
    the module configuration is never touched.

Usage (standalone):
    python3 gps_reader.py [N]     # print N gated fixes (default 3)
"""

import sys
import time
from dataclasses import dataclass
from typing import Optional

import serial

GPS_DEVICE = "/dev/ttyACM0"
BAUD_CANDIDATES = (115200, 9600)
DEFAULT_TIMEOUT = 10.0
KNOTS_TO_MS = 0.514444  # 1 knot = 0.514444 m/s


def _checksum_ok(line: str) -> bool:
    """Return True if the NMEA checksum (XOR of chars after '$' up to '*')
    matches the 2-hex value following '*'."""
    if "*" not in line:
        return False
    body, _, csum = line.partition("*")
    csum = csum.strip()
    if len(csum) != 2:
        return False
    payload = body[1:] if body[:1] in ("$", "!") else body
    acc = 0
    for ch in payload:
        acc ^= ord(ch)
    try:
        return "%02X" % acc == csum.upper()
    except Exception:
        return False


def _dmm_to_deg(dmm: str, hemi: str) -> Optional[float]:
    """Convert NMEA DDDMM.mmmm + hemisphere to signed decimal degrees."""
    if not dmm or not hemi or hemi not in "NSEW":
        return None
    try:
        if "." in dmm:
            int_part, frac = dmm.split(".")
            deg = int(int_part[:-2])
            minutes = float(int_part[-2:] + "." + frac)
        else:
            deg = int(dmm[:-2])
            minutes = float(dmm[-2:])
    except (ValueError, IndexError):
        return None
    value = deg + minutes / 60.0
    return -value if hemi in ("S", "W") else value


def _to_float(field: str) -> Optional[float]:
    try:
        return float(field) if field.strip() else None
    except ValueError:
        return None


def _to_int(field: str) -> Optional[int]:
    try:
        return int(float(field)) if field.strip() else None
    except ValueError:
        return None


@dataclass
class GPSFix:
    """One gated GNSS fix."""

    t_mono: float      # time.monotonic() seconds at fix time
    lat: float         # degrees, N positive
    lon: float         # degrees, E positive
    alt_m: float       # altitude above mean sea level [m]
    fix: int           # GGA fix quality (1+ after gating)
    hdop: float        # horizontal dilution of precision (< 2.0 after gating)
    speed_ms: float    # speed over ground [m/s] = knots * 0.514444
    course_deg: float  # true course over ground [deg]


def gating_ok(fix: int, hdop: float) -> bool:
    """Gate rule: accept only fix quality >= 1 AND HDOP < 2.0."""
    return fix >= 1 and hdop < 2.0


class GPSReader:
    """Blocking GPS fix reader with fix gating."""

    def __init__(self, device: str = GPS_DEVICE, timeout: float = DEFAULT_TIMEOUT):
        self.device = device
        self.timeout = timeout
        self.coast_events = 0      # count of rejected (gated-out) updates
        self._sentences_read = 0   # raw NMEA lines read from device
        self._sentences_parsed = 0 # lines with valid start + checksum
        self._ser_baud = None
        self._ser = self._open_port()

    # ------------------------------------------------------------------ I/O
    def _open_port(self) -> serial.Serial:
        """Open the device and keep the baud that actually streams NMEA."""
        for baud in BAUD_CANDIDATES:
            try:
                ser = serial.Serial(self.device, baud, timeout=1.0)
            except serial.SerialException as exc:
                raise RuntimeError("cannot open %s: %s" % (self.device, exc))
            for _ in range(3):
                line = ser.readline().decode("ascii", errors="replace")
                if line.startswith("$") and _checksum_ok(line):
                    self._ser_baud = baud
                    return ser
            ser.close()
        raise RuntimeError("no usable NMEA stream on %s" % self.device)

    # ----------------------------------------------------------------- read
    def read_fix(self) -> Optional[GPSFix]:
        """Block until the next gated fix.

        Merges the latest $GNGGA (position/fix/HDOP/alt) with recent $GNRMC /
        $GNVTG (speed/course). Returns a GPSFix, or None if the internal
        timeout expires first.
        """
        deadline = time.monotonic() + self.timeout
        state = {}  # rolling latest fields from RMC/VTG: spd_kts, course
        while time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            self._sentences_read += 1
            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue
            if not line.startswith("$") or not _checksum_ok(line):
                continue
            self._sentences_parsed += 1
            fields = line.split(",")
            kind = fields[0][3:]
            if kind == "GGA" and len(fields) >= 12:
                lat = _dmm_to_deg(fields[2], fields[3])
                lon = _dmm_to_deg(fields[4], fields[5])
                fix = _to_int(fields[6])
                hdop = _to_float(fields[8])
                alt = _to_float(fields[9])
                if lat is None or lon is None or fix is None or hdop is None:
                    continue
                if not gating_ok(fix, hdop):
                    self.coast_events += 1  # reject: bad quality, count coast
                    continue
                spd = state.get("spd_kts")
                return GPSFix(
                    t_mono=time.monotonic(),
                    lat=lat,
                    lon=lon,
                    alt_m=alt if alt is not None else 0.0,
                    fix=fix,
                    hdop=hdop,
                    speed_ms=(spd * KNOTS_TO_MS) if spd is not None else 0.0,
                    course_deg=float(state.get("course") or 0.0),
                )
            elif kind == "RMC" and len(fields) >= 12 and fields[2] == "A":
                spd = _to_float(fields[7])   # speed over ground, knots
                crs = _to_float(fields[8])   # true course, deg
                if spd is not None:
                    state["spd_kts"] = spd
                if crs is not None:
                    state["course"] = crs
            elif kind == "VTG" and len(fields) >= 8:
                crs = _to_float(fields[1])   # true course, deg
                if crs is not None:
                    state["course"] = crs
        return None


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    r = GPSReader()
    print("device=%s baud=%s timeout=%.1fs" % (r.device, r._ser_baud, r.timeout))
    for i in range(n):
        fx = r.read_fix()
        print("fix[%d] = %s" % (i, fx if fx is not None else "NONE"))
    print("stats: sentences_read=%d parsed=%d coast_events=%d"
          % (r._sentences_read, r._sentences_parsed, r.coast_events))

#!/usr/bin/env python3
"""test_imu_reader.py -- offline unit tests for the WitMotion WT901SDCL
imu_reader rewrite. No hardware needed: synthetic byte streams are fed to
the parser through a fake serial object.

Run:  python3 test_imu_reader.py   (exit 0 = all pass)
"""
import math
import sys
import time

import imu_reader
from imu_reader import (IMUReader, IMUError, IMUSample, _extract_frame,
                        _i16, _find_devices, SAMPLE_BYTES, FRAME_LEN,
                        PKT_ACCEL, PKT_GYRO, G, DEG2RAD,
                        ACCEL_RAW_TO_MS2, GYRO_RAW_TO_RADS, SAT_LSB)

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("PASS: %s" % name)
    else:
        FAIL += 1
        print("FAIL: %s %s" % (name, detail))


def frame(ptype, d0, d1, d2, d3=0):
    """Build a checksum-valid 11-byte WitMotion frame."""
    body = bytearray([0x55, ptype])
    for d in (d0, d1, d2, d3):
        body.append(d & 0xFF)
        body.append((d >> 8) & 0xFF)
    body.append(sum(body) & 0xFF)
    return bytes(body)


def corrupt(b):
    """Flip the checksum byte so the frame must be rejected."""
    return b[:-1] + bytes([(b[-1] + 1) & 0xFF])


class FakeSer:
    """Minimal pyserial stand-in: serves queued byte chunks, records writes.

    loop=True models a continuously transmitting module (chunks repeat at
    `period` seconds per chunk); the default finite mode returns b"" once
    the stream is exhausted (used for the stall guard test).
    """

    def __init__(self, chunks=(), in_waiting=0, loop=False, period=0.0):
        self._chunks = [bytes(c) for c in chunks]
        self._loop = loop
        self._period = period
        self._idx = 0
        self._next_t = 0.0
        self.in_waiting = in_waiting
        self.timeout = 0.2
        self.written = []
        self.closed = False

    def read(self, n):
        if self._loop and self._chunks:
            if self._idx >= len(self._chunks):
                self._idx = 0
            now = time.monotonic()
            if now < self._next_t:
                time.sleep(min(self._next_t - now, 0.05))
            c = self._chunks[self._idx]
            self._idx += 1
            self._next_t = max(self._next_t, time.monotonic()) + self._period
            return c
        if self._idx < len(self._chunks):
            c = self._chunks[self._idx]
            self._idx += 1
            return c
        return b""

    def write(self, data):
        self.written.append(bytes(data))

    def reset_input_buffer(self):
        # no-op: chunks model the live incoming stream (bytes "sent by the
        # module"), not the OS RX buffer, so a buffer reset must not eat them
        pass

    def close(self):
        self.closed = True


def make_reader(chunks, loop=False, period=0.0):
    """Build an IMUReader without touching a real serial port."""
    r = IMUReader.__new__(IMUReader)
    r.ser = FakeSer(chunks, loop=loop, period=period)
    r._buf = bytearray()
    r.cal = None
    r.port = "FAKE"
    r.baud = 115200
    return r


def approx(a, b, tol=0.001):
    return abs(a - b) <= tol * abs(b)


# ------------------------------------------------------------------ frames
def test_frame_build_and_i16():
    check("SAMPLE_BYTES == 22", SAMPLE_BYTES == 22)
    check("FRAME_LEN == 11", FRAME_LEN == 11)
    check("_i16 little-endian +127", _i16(0x7F, 0x00) == 127)
    check("_i16 little-endian -1", _i16(0xFF, 0xFF) == -1)
    check("_i16 little-endian -32768", _i16(0x00, 0x80) == -32768)
    f = frame(PKT_ACCEL, 1, -2, 32767)
    check("frame is 11 bytes with 0x55 header", len(f) == 11 and f[0] == 0x55)
    check("frame checksum valid", (sum(f[:10]) & 0xFF) == f[10])


# ------------------------------------------------------------- scaling
def test_scaling():
    # raw = 16384 = half of 32768 -> accel = 8 g = 8 * 9.80665 m/s^2
    ax = 16384 * ACCEL_RAW_TO_MS2
    check("accel half-scale = 8g ~ 78.453 m/s^2",
          approx(ax, 8.0 * G) and approx(ax, 78.453), "ax=%.5f" % ax)
    # raw = 16384 -> gyro = 1000 dps = 1000 * pi/180 rad/s
    gx = 16384 * GYRO_RAW_TO_RADS
    check("gyro half-scale = 1000 dps ~ 17.453 rad/s",
          approx(gx, 1000.0 * DEG2RAD) and approx(gx, 17.453), "gx=%.5f" % gx)
    check("SAT_LSB = 0.98 * 32768", SAT_LSB == int(0.98 * 32768))


# ------------------------------------------------------- frame extraction
def test_extract_frame():
    # clean single frame
    buf = bytearray(frame(PKT_ACCEL, 100, -200, 300))
    pkt = _extract_frame(buf)
    check("extract valid accel frame",
          pkt == (PKT_ACCEL, 100, -200, 300, 0), str(pkt))
    check("buffer consumed after extraction", len(buf) == 0)

    # checksum-corrupted frame is skipped, valid one found after it
    buf = bytearray(corrupt(frame(PKT_ACCEL, 1, 1, 1)) +
                    frame(PKT_ACCEL, 5, 6, 7))
    pkt = _extract_frame(buf)
    check("checksum error skipped -> next frame decoded",
          pkt == (PKT_ACCEL, 5, 6, 7, 0), str(pkt))

    # noise bytes before the frame: parser re-syncs on 0x55
    buf = bytearray(b"\x12\x34\x55\x00\x99\xAA" + frame(PKT_GYRO, 10, 20, 30))
    pkt = _extract_frame(buf)
    check("noise before frame tolerated",
          pkt == (PKT_GYRO, 10, 20, 30, 0), str(pkt))

    # 0x55 inside payload (d0 = 0x5555) must not derail the checksum scan
    f = frame(PKT_ACCEL, 0x5555, 0, 0)
    buf = bytearray(f + frame(PKT_ACCEL, 1, 2, 3))
    pkt = _extract_frame(buf)
    check("0x55 payload byte handled", pkt == (PKT_ACCEL, 0x5555, 0, 0, 0),
          str(pkt))
    pkt2 = _extract_frame(buf)
    check("frame after 0x55 payload intact", pkt2 == (PKT_ACCEL, 1, 2, 3, 0),
          str(pkt2))

    # partial header kept for next call
    buf = bytearray(frame(PKT_ACCEL, 1, 2, 3)[:9])   # truncated frame
    check("partial frame -> None", _extract_frame(buf) is None)
    buf.extend(frame(PKT_ACCEL, 1, 2, 3)[9:])
    check("partial frame completes", _extract_frame(buf) == (PKT_ACCEL, 1, 2, 3, 0))


# --------------------------------------------------- read_sample end-to-end
def test_read_sample():
    # known values: accel raw 16384, gyro raw 8192 (500 dps)
    acc = frame(PKT_ACCEL, 16384, 0, 0)
    gyr = frame(PKT_GYRO, 0, 8192, 0)
    r = make_reader([acc, gyr])
    s = r.read_sample()
    check("read_sample accel 16384 -> 8g", approx(s.ax, 8.0 * G), "ax=%.5f" % s.ax)
    check("read_sample gyro 8192 -> 500 dps", approx(s.gy, 500.0 * DEG2RAD),
          "gy=%.5f" % s.gy)
    check("read_sample unused axes zero", s.ay == 0.0 and s.az == 0.0
          and s.gx == 0.0 and s.gz == 0.0)
    check("read_sample sat_flag 0 (half scale)", s.sat_flag == 0)
    check("read_sample returns IMUSample", isinstance(s, IMUSample))

    # saturation: |raw| = 32767 >= 0.98 * 32768 -> sat_flag 1
    r = make_reader([frame(PKT_ACCEL, 32767, 0, 0), frame(PKT_GYRO, 0, 0, 0)])
    s = r.read_sample()
    check("sat_flag 1 at raw 32767", s.sat_flag == 1, "sat=%d" % s.sat_flag)

    # checksum error between the pair: skipped, pair still matched
    r = make_reader([frame(PKT_ACCEL, 16384, 0, 0),
                     corrupt(frame(PKT_GYRO, 9, 9, 9)),
                     frame(PKT_GYRO, 8192, 0, 0)])
    s = r.read_sample()
    check("bad frame between pair skipped", approx(s.gx, 500.0 * DEG2RAD)
          and s.sat_flag == 0, "gx=%.5f" % s.gx)

    # lost gyro: accel,accel,gyro -> newer accel wins
    r = make_reader([frame(PKT_ACCEL, 0, 0, 0),
                     frame(PKT_ACCEL, 16384, 0, 0),
                     frame(PKT_GYRO, 0, 0, 0)])
    s = r.read_sample()
    check("lost gyro -> newest accel paired", approx(s.ax, 8.0 * G), "ax=%.5f" % s.ax)

    # leading stale gyro (mid-stream join) is skipped
    r = make_reader([frame(PKT_GYRO, 999, 999, 999),
                     frame(PKT_ACCEL, 0, 16384, 0),
                     frame(PKT_GYRO, 0, 8192, 0)])
    s = r.read_sample()
    check("leading stale gyro skipped", approx(s.ay, 8.0 * G)
          and approx(s.gy, 500.0 * DEG2RAD), "ay=%.5f gy=%.5f" % (s.ay, s.gy))

    # noise bytes between the pair: no crash, no misalignment
    r = make_reader([frame(PKT_ACCEL, 0, 0, 16384),
                     b"\x00\x55\xFF\xFE\x01\x02",
                     frame(PKT_GYRO, 0, 0, 8192)])
    s = r.read_sample()
    check("inter-pair noise tolerated", approx(s.az, 8.0 * G)
          and approx(s.gz, 500.0 * DEG2RAD), "az=%.5f gz=%.5f" % (s.az, s.gz))

    # small chunks: parser state survives arbitrary read slicing
    acc = frame(PKT_ACCEL, 16384, 0, 0)
    gyr = frame(PKT_GYRO, 0, 8192, 0)
    stream = acc + gyr
    r = make_reader([stream[i:i + 3] for i in range(0, len(stream), 3)])
    s = r.read_sample()
    check("3-byte chunks reassembled", approx(s.ax, 8.0 * G)
          and approx(s.gy, 500.0 * DEG2RAD), "ax=%.5f gy=%.5f" % (s.ax, s.gy))


# ------------------------------------------------------- bias correction
def test_bias_correction():
    acc = frame(PKT_ACCEL, 16384, 0, 0)
    gyr = frame(PKT_GYRO, 0, 8192, 0)
    r = make_reader([acc, gyr])
    r.cal = {"gyro_bias": [0.1, 0.2, 0.3],
             "accel_bias": [0.5, -0.5, 0.25]}
    s = r.read_sample()
    check("accel bias subtracted", approx(s.ax, 8.0 * G - 0.5),
          "ax=%.5f" % s.ax)
    check("gyro bias subtracted", approx(s.gy, 500.0 * DEG2RAD - 0.2),
          "gy=%.5f" % s.gy)


# ------------------------------------------------------------- calibrate
def test_calibrate():
    pairs = []
    for _ in range(40):
        pairs.append(frame(PKT_ACCEL, 0, 0, 2048))    # +z 1 g (mounted flat)
        pairs.append(frame(PKT_GYRO, 0, 0, 0))
    # loop=True: continuous 50 Hz stream (period 20 ms/chunk) as on hardware
    r = make_reader(pairs, loop=True, period=0.02)
    cal = r.calibrate(duration=1.0)
    check("calibrate keys", sorted(cal.keys()) ==
          sorted(["gyro_bias", "accel_bias", "mount_gravity",
                  "n_samples", "duration_s"]), str(sorted(cal.keys())))
    check("calibrate n_samples >= 10", cal["n_samples"] >= 10,
          "n=%d" % cal["n_samples"])
    check("calibrate gyro_bias ~ 0", all(abs(v) < 1e-9 for v in cal["gyro_bias"]),
          str(cal["gyro_bias"]))
    check("calibrate mount_gravity = +z", approx(cal["mount_gravity"][2], 1.0)
          and abs(cal["mount_gravity"][0]) < 1e-9
          and abs(cal["mount_gravity"][1]) < 1e-9, str(cal["mount_gravity"]))
    check("calibrate accel_bias ~ 0", all(abs(v) < 1e-9 for v in cal["accel_bias"]),
          str(cal["accel_bias"]))
    check("calibrate stored as self.cal", r.cal is cal)

    # mounted tilted: raw accel (5g, 12g, 3g) -> gravity vector normalized
    pairs = []
    for _ in range(40):
        pairs.append(frame(PKT_ACCEL, 5 * 32768 // 16, 12 * 32768 // 16,
                           3 * 32768 // 16))
        pairs.append(frame(PKT_GYRO, 0, 0, 0))
    r2 = make_reader(pairs, loop=True, period=0.02)
    cal2 = r2.calibrate(duration=1.0)
    mg = cal2["mount_gravity"]
    check("tilted mount_gravity unit vector", approx(math.sqrt(mg[0] ** 2 + mg[1] ** 2
          + mg[2] ** 2), 1.0), str(mg))
    check("tilted accel_bias = mean - G*mg (per-axis)",
          approx(cal2["accel_bias"][0],
                 (5.0 * G) - G * mg[0]), str(cal2["accel_bias"]))

    # accel magnitude implausible -> RuntimeError
    pairs = [frame(PKT_ACCEL, 0, 0, 0), frame(PKT_GYRO, 0, 0, 0)] * 40
    r3 = make_reader(pairs, loop=True, period=0.02)
    try:
        r3.calibrate(duration=1.0)
        check("calibrate zero-accel raises RuntimeError", False)
    except RuntimeError as exc:
        check("calibrate zero-accel raises RuntimeError",
              "implausible" in str(exc), str(exc))

    # too few samples (n < 10) -> RuntimeError: fake time ends the loop
    sample = IMUSample(0.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0)
    r4 = make_reader([])
    r4._read_raw_sample = lambda: sample
    real_mono = imu_reader.time.monotonic
    calls = {"n": 0}
    def fake_mono():                                    # noqa: E306
        calls["n"] += 1
        return 0.0 if calls["n"] <= 6 else 1000.0
    imu_reader.time.monotonic = fake_mono
    try:
        try:
            r4.calibrate(duration=1.0)
            check("calibrate n<10 raises RuntimeError", False)
        except RuntimeError as exc:
            check("calibrate n<10 raises RuntimeError", "too few" in str(exc),
                  str(exc))
    finally:
        imu_reader.time.monotonic = real_mono


# ------------------------------------------------------------- fifo count
def test_fifo_count():
    r = make_reader([])
    r.ser.in_waiting = 44
    check("_fifo_count = in_waiting", r._fifo_count() == 44)
    r2 = IMUReader.__new__(IMUReader)
    r2.ser = None
    check("_fifo_count safe when closed", r2._fifo_count() == 0)


# -------------------------------------------------------- open / probing
def test_open_errors():
    saved = imu_reader._find_devices

    imu_reader._find_devices = lambda: []
    try:
        try:
            IMUReader()
            check("no devices -> IMUError", False)
        except IMUError as exc:
            check("no devices -> IMUError", "USB" in str(exc)
                  and "ttyACM0" in str(exc), str(exc))
    finally:
        imu_reader._find_devices = saved

    # all probes fail -> readable error mentioning bauds and GPS exclusion
    imu_reader._find_devices = lambda: ["/dev/ttyUSB0"]
    real_serial = imu_reader.serial
    class _NoSerial:                                    # noqa: E306
        SerialException = real_serial.SerialException

        class Serial:
            def __init__(self, *a, **k):
                raise _NoSerial.SerialException("nope")
    try:
        imu_reader.serial = _NoSerial
        try:
            IMUReader()
            check("no frames -> IMUError", False)
        except IMUError as exc:
            check("no frames -> IMUError", "ttyACM0" in str(exc)
                  and "9600" in str(exc) and "115200" in str(exc), str(exc))
    finally:
        imu_reader.serial = real_serial
        imu_reader._find_devices = saved


def test_probe_and_config_commands():
    # _probe_port finds a valid frame in a noisy stream
    r = IMUReader.__new__(IMUReader)
    real_serial = imu_reader.serial

    class _FakeSerialFactory:
        SerialException = real_serial.SerialException
        instances = []
        class Serial(FakeSer):                          # noqa: E306
            def __init__(self, *a, **k):
                FakeSer.__init__(self, [b"\x00\x00\x00\x00",
                                        frame(PKT_ACCEL, 1, 2, 3)])
                _FakeSerialFactory.instances.append(self)

    imu_reader.serial = _FakeSerialFactory
    try:
        check("_probe_port detects valid frame", r._probe_port("/dev/ttyUSB0",
                                                              115200, 2.0))
        check("_probe_port closed the probe port",
              _FakeSerialFactory.instances[-1].closed)

        # configuration command sequence (bytes must match the protocol)
        ser = FakeSer()
        r2 = IMUReader.__new__(IMUReader)
        r2._probe_port = lambda port, baud, t: baud == 115200
        orig_serial_cls = imu_reader.serial.Serial
        imu_reader.serial.Serial = lambda *a, **k: ser
        try:
            baud = r2._configure_device("/dev/ttyUSB0", 1.0)
            check("_configure_device returns 115200", baud == 115200)
            check("unlock command written",
                  ser.written[0] == b"\xFF\xAA\x69\x88\xB5",
                  str(ser.written[0]))
            check("content reg write", ser.written[1] == b"\xFF\xAA\x02\x03\x00",
                  str(ser.written[1]))
            check("rate reg write", ser.written[2] == b"\xFF\xAA\x03\x08\x00",
                  str(ser.written[2]))
            check("baud reg write", ser.written[3] == b"\xFF\xAA\x04\x06\x00",
                  str(ser.written[3]))
            check("exactly 4 commands written", len(ser.written) == 4,
                  "n=%d" % len(ser.written))
            check("config port closed", ser.closed)
        finally:
            imu_reader.serial.Serial = orig_serial_cls

        # config fallback: 115200 silent, 9600 alive -> warn + use 9600
        ser2 = FakeSer()
        r3 = IMUReader.__new__(IMUReader)
        r3._probe_port = lambda port, baud, t: baud == 9600
        imu_reader.serial.Serial = lambda *a, **k: ser2
        try:
            baud = r3._configure_device("/dev/ttyUSB0", 1.0)
            check("_configure_device falls back to 9600", baud == 9600)
        finally:
            imu_reader.serial.Serial = orig_serial_cls

        # config total failure -> IMUError
        r4 = IMUReader.__new__(IMUReader)
        r4._probe_port = lambda port, baud, t: False
        imu_reader.serial.Serial = lambda *a, **k: FakeSer()
        try:
            try:
                r4._configure_device("/dev/ttyUSB0", 1.0)
                check("dead module -> IMUError", False)
            except IMUError as exc:
                check("dead module -> IMUError", "stopped answering" in str(exc),
                      str(exc))
        finally:
            imu_reader.serial.Serial = orig_serial_cls
    finally:
        imu_reader.serial = real_serial


def test_full_open_flow():
    """IMUReader() end-to-end with a fake port factory: auto-detect + config."""
    real_serial = imu_reader.serial
    real_find = imu_reader._find_devices

    class Env:
        stream = None
        serials = []

    def mk_serial(port, baud, **kw):
        s = FakeSer(Env.stream)
        s.port = port
        s.baud = baud
        Env.serials.append(s)
        return s

    class FakeModule:
        SerialException = imu_reader.serial.SerialException

        @staticmethod
        def Serial(port, baud, **kw):
            return mk_serial(port, baud, **kw)

    # case A: module already at 115200 -> used as-is, zero writes
    Env.stream = [frame(PKT_ACCEL, 16384, 0, 0)]
    Env.serials = []
    imu_reader.serial = FakeModule
    imu_reader._find_devices = lambda: ["/dev/ttyUSB0"]
    try:
        r = IMUReader(probe_timeout=0.3)
        check("auto-open picks 115200 when configured", r.baud == 115200
              and r.port == "/dev/ttyUSB0", "baud=%s port=%s" % (r.baud, r.port))
        check("no config writes in the pre-configured case",
              all(len(s.written) == 0 for s in Env.serials))
    finally:
        imu_reader.serial = real_serial
        imu_reader._find_devices = real_find

    # case C: factory 9600 -> first 115200 probe fails, 9600 succeeds,
    # config written, post-config 115200 probe succeeds -> final baud 115200
    class EnvC:
        serials = []
        n115 = 0

    def mk_serial_c(port, baud, **kw):
        if baud == 115200:
            EnvC.n115 += 1
            stream = [] if EnvC.n115 == 1 else [frame(PKT_ACCEL, 1, 2, 3)]
        else:
            stream = [frame(PKT_ACCEL, 1, 2, 3)]
        s = FakeSer(stream)
        s.port, s.baud = port, baud
        EnvC.serials.append(s)
        return s

    class FakeModuleC:
        SerialException = imu_reader.serial.SerialException

        @staticmethod
        def Serial(port, baud, **kw):
            return mk_serial_c(port, baud, **kw)

    imu_reader.serial = FakeModuleC
    imu_reader._find_devices = lambda: ["/dev/ttyUSB0"]
    try:
        r = IMUReader(probe_timeout=0.3)
        check("factory 9600 -> configured and reopened at 115200",
              r.baud == 115200, "baud=%s" % r.baud)
        written = [w for s in EnvC.serials for w in s.written]
        check("factory path wrote unlock", b"\xFF\xAA\x69\x88\xB5" in written)
        check("factory path wrote content reg", b"\xFF\xAA\x02\x03\x00" in written)
        check("factory path wrote rate reg", b"\xFF\xAA\x03\x08\x00" in written)
        check("factory path wrote baud reg", b"\xFF\xAA\x04\x06\x00" in written)
        check("factory path opened 9600 first", any(s.baud == 9600 for s in EnvC.serials))
    finally:
        imu_reader.serial = real_serial
        imu_reader._find_devices = real_find

    # case D: ttyACM0 (GPS) never probed even when present
    real_glob = imu_reader.glob.glob
    try:
        imu_reader.glob.glob = lambda pat: (["/dev/ttyUSB0", "/dev/ttyUSB1"]
                                            if pat == "/dev/ttyUSB*"
                                            else ["/dev/ttyACM0", "/dev/ttyACM1"])
        devs = _find_devices()
        check("ttyACM0 excluded from devices", "/dev/ttyACM0" not in devs,
              str(devs))
        check("device ordering ttyUSB before ttyACM",
              devs == ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM1"],
              str(devs))
    finally:
        imu_reader.glob.glob = real_glob


# ------------------------------------------------------------- stall guard
def test_no_data_stall_guard():
    r = make_reader([])              # empty stream: never any bytes
    saved = imu_reader.NO_DATA_TIMEOUT_S
    imu_reader.NO_DATA_TIMEOUT_S = 0.05
    try:
        t0 = time.monotonic()
        try:
            r._read_next_packet()
            check("stall -> IMUError", False)
        except IMUError as exc:
            check("stall -> IMUError", "no data" in str(exc), str(exc))
        check("stall detected promptly", time.monotonic() - t0 < 2.0,
              "took %.2fs" % (time.monotonic() - t0))
    finally:
        imu_reader.NO_DATA_TIMEOUT_S = saved


def main():
    tests = [
        test_frame_build_and_i16,
        test_scaling,
        test_extract_frame,
        test_read_sample,
        test_bias_correction,
        test_calibrate,
        test_fifo_count,
        test_open_errors,
        test_probe_and_config_commands,
        test_full_open_flow,
        test_no_data_stall_guard,
    ]
    for t in tests:
        try:
            t()
        except Exception as exc:          # noqa: BLE001
            global FAIL
            FAIL += 1
            import traceback
            print("FAIL: %s raised %r" % (t.__name__, exc))
            traceback.print_exc()
    print("")
    print("RESULT: %d passed, %d failed" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

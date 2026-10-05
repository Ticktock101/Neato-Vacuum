import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from neato_mapper.control import (PID, DiffDriveOdometry, QuadratureDecoder,  # noqa: E402
                                  slew, twist_to_wheels, wheels_to_twist)
from neato_mapper.hardware import MockBackend, MotorChannel, WheelEncoder  # noqa: E402
from neato_mapper.xv11_protocol import (PacketParser, ScanAssembler,  # noqa: E402
                                        decode_packet, encode_packet,
                                        scan_to_ranges)


def _rev(rpm=300.0, dist=1234):
    out = b""
    for i in range(90):
        rd = [(dist + i, 100 + i, i == 5, False) for _ in range(4)]
        out += encode_packet(i, rpm, rd)
    return out


# ---------------------------------------------------------------- protocol
def test_roundtrip():
    raw = encode_packet(7, 301.5, [(1000, 50, False, False), (16383, 1, False, True),
                                   (0, 0, True, False), (42, 65535, False, False)])
    p = decode_packet(raw)
    assert p.index == 7 and abs(p.rpm - 301.5) < 1 / 64
    r = p.readings
    assert [x.angle_deg for x in r] == [28, 29, 30, 31]
    assert r[0].distance_mm == 1000 and r[0].valid
    assert r[1].distance_mm == 16383 and r[1].strength_warning
    assert not r[2].valid
    assert r[3].strength == 65535


def test_checksum_rejects_corruption():
    raw = bytearray(encode_packet(3, 300, [(500, 10, False, False)] * 4))
    raw[6] ^= 0x01
    assert decode_packet(raw) is None


def test_parser_resyncs_through_garbage_and_splits():
    data = b"Piccolo Laser Distance Scanner\r\n\xfa\x00\xfa" + _rev()
    parser = PacketParser()
    pkts = []
    for i in range(0, len(data), 7):       # odd-sized chunks
        pkts += parser.feed(data[i:i + 7])
    assert len(pkts) == 90
    assert [p.index for p in pkts] == list(range(90))


def test_assembler_and_ranges():
    parser, asm = PacketParser(), ScanAssembler()
    scans = []
    for t, p in enumerate(parser.feed(_rev() + _rev())):
        s = asm.add(p, t * 0.002)
        if s:
            scans.append(s)
    assert len(scans) == 1
    s = scans[0]
    assert s.packet_count == 90 and abs(s.rpm - 300) < 0.1
    assert s.distances_mm[20] is None          # packet 5 was flagged invalid
    ranges, inten = scan_to_ranges(s, 0.06, 5.0)
    assert math.isinf(ranges[20])
    assert abs(ranges[0] - 1.234) < 1e-9
    rev, _ = scan_to_ranges(s, 0.06, 5.0, reverse=True)
    assert rev[0] == ranges[0] and rev[359] == ranges[1]


# ------------------------------------------------------------------ control
def test_kinematics_roundtrip():
    vl, vr = twist_to_wheels(0.2, 1.0, 0.24)
    v, w = wheels_to_twist(vl, vr, 0.24)
    assert abs(v - 0.2) < 1e-12 and abs(w - 1.0) < 1e-12


def test_odometry_straight_and_spin():
    o = DiffDriveOdometry(0.24)
    for _ in range(100):
        o.update(0.01, 0.01)
    assert abs(o.pose.x - 1.0) < 1e-9 and abs(o.pose.y) < 1e-9
    # Spin in place a full turn.
    arc = math.pi * 0.24 / 1000
    for _ in range(1000):
        o.update(-arc, arc)
    assert abs(o.pose.x - 1.0) < 1e-9 and abs(o.pose.theta) < 1e-6


def test_odometry_quarter_circle():
    o = DiffDriveOdometry(0.2)
    r = 1.0
    n = 1000
    dth = (math.pi / 2) / n
    for _ in range(n):
        o.update((r - 0.1) * dth, (r + 0.1) * dth)
    assert abs(o.pose.x - 1.0) < 1e-6 and abs(o.pose.y - 1.0) < 1e-6
    assert abs(o.pose.theta - math.pi / 2) < 1e-9


def test_quadrature():
    q = QuadratureDecoder(0, 0)
    seq = [(0, 1), (1, 1), (1, 0), (0, 0)] * 10
    for a, b in seq:
        q.update(a, b)
    assert q.count == 40
    for a, b in reversed([(0, 0)] + seq[:-1]):
        q.update(a, b)
    assert q.count == 0 and q.errors == 0


def test_wheel_encoder_with_mock_backend():
    g = MockBackend()
    enc = WheelEncoder(g, 17, 27, invert=True)
    for a, b in [(0, 1), (1, 1), (1, 0), (0, 0)] * 5:
        g.inject(27, b)
        g.inject(17, a)
    assert enc.count == -20


def test_motor_channel_direction():
    g = MockBackend()
    m = MotorChannel(g, 5, 6, 12, 2000, invert=True)
    m.set(0.5)
    assert g.levels[5] == 0 and g.levels[6] == 1 and g.duty[12] == 0.5
    m.set(0)
    assert g.duty[12] == 0


def test_slew():
    assert slew(0, 1, 0.1) == 0.1 and slew(0, -1, 0.1) == -0.1 and slew(0, 0.05, 0.1) == 0.05


def test_lidar_pid_converges_on_motor_model():
    """First-order motor (rpm = 520*duty, tau 0.4 s) must settle at 300 RPM."""
    pid = PID(0.001, 0.002, 0.0, 0.0, 0.9, bias=0.45)   # deliberately wrong FF
    rpm, dt = 0.0, 0.02
    for _ in range(int(20 / dt)):
        duty = pid.update(300.0, rpm, dt)
        rpm += (duty * 520 - rpm) * dt / 0.4
    assert abs(rpm - 300) < 3

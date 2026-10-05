"""Pure-Python decoder for the Neato XV-11 / Botvac LDS serial protocol.

No ROS or hardware imports here so it can be unit tested anywhere.

Packet format (firmware v2.4+, 115200 8N1), 22 bytes:

    byte 0      0xFA start byte
    byte 1      index 0xA0..0xF9  (90 packets per revolution, 4 degrees each)
    byte 2-3    motor speed, little endian, in 1/64 RPM
    byte 4-19   4 x 4-byte readings:
                  b0      distance[7:0]
                  b1      bit7 = invalid flag, bit6 = strength warning,
                          bits5..0 = distance[13:8]
                  b2-b3   signal strength, little endian
    byte 20-21  checksum, little endian (see :func:`checksum`)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

PACKET_LEN = 22
START_BYTE = 0xFA
INDEX_MIN = 0xA0
INDEX_MAX = 0xF9
PACKETS_PER_REV = 90
READINGS_PER_PACKET = 4
READINGS_PER_REV = PACKETS_PER_REV * READINGS_PER_PACKET  # 360


@dataclass(frozen=True)
class Reading:
    angle_deg: int          # 0..359 in the sensor's own frame
    distance_mm: int        # only meaningful when valid
    strength: int
    valid: bool
    strength_warning: bool


@dataclass(frozen=True)
class Packet:
    index: int              # 0..89
    rpm: float
    readings: Tuple[Reading, ...]


def checksum(data: Sequence[int]) -> int:
    """Checksum over the first 20 bytes of a packet."""
    chk32 = 0
    for i in range(10):
        word = data[2 * i] | (data[2 * i + 1] << 8)
        chk32 = (chk32 << 1) + word
    result = (chk32 & 0x7FFF) + (chk32 >> 15)
    return result & 0x7FFF


def decode_packet(raw: Sequence[int]) -> Optional[Packet]:
    """Decode one 22-byte packet. Returns None if it is malformed."""
    if len(raw) != PACKET_LEN or raw[0] != START_BYTE:
        return None
    if not INDEX_MIN <= raw[1] <= INDEX_MAX:
        return None
    if checksum(raw) != (raw[20] | (raw[21] << 8)):
        return None

    index = raw[1] - INDEX_MIN
    rpm = (raw[2] | (raw[3] << 8)) / 64.0
    readings = []
    for j in range(READINGS_PER_PACKET):
        b0, b1, b2, b3 = raw[4 + 4 * j: 8 + 4 * j]
        readings.append(Reading(
            angle_deg=index * READINGS_PER_PACKET + j,
            distance_mm=b0 | ((b1 & 0x3F) << 8),
            strength=b2 | (b3 << 8),
            valid=not (b1 & 0x80),
            strength_warning=bool(b1 & 0x40),
        ))
    return Packet(index=index, rpm=rpm, readings=tuple(readings))


def encode_packet(index: int, rpm: float,
                  readings: Sequence[Tuple[int, int, bool, bool]]) -> bytes:
    """Build a valid packet (used by tests and the mock LiDAR).

    ``readings`` is 4 tuples of (distance_mm, strength, invalid, warning).
    """
    speed = int(round(rpm * 64)) & 0xFFFF
    out = [START_BYTE, INDEX_MIN + index, speed & 0xFF, speed >> 8]
    for dist, strength, invalid, warning in readings:
        b1 = (dist >> 8) & 0x3F
        if invalid:
            b1 |= 0x80
        if warning:
            b1 |= 0x40
        out += [dist & 0xFF, b1, strength & 0xFF, (strength >> 8) & 0xFF]
    chk = checksum(out)
    out += [chk & 0xFF, chk >> 8]
    return bytes(out)


class PacketParser:
    """Streaming parser: feed arbitrary byte chunks, get decoded packets.

    Re-synchronises automatically on garbage (e.g. the ASCII banner the LDS
    prints while its motor is stopped) and on checksum failures.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.good_packets = 0
        self.bad_packets = 0

    def feed(self, data: bytes) -> List[Packet]:
        self._buf.extend(data)
        packets: List[Packet] = []
        buf = self._buf
        while True:
            start = buf.find(START_BYTE)
            if start < 0:
                buf.clear()
                break
            if start:
                del buf[:start]
            if len(buf) < 2:
                break
            if not INDEX_MIN <= buf[1] <= INDEX_MAX:
                del buf[0]
                continue
            if len(buf) < PACKET_LEN:
                break
            pkt = decode_packet(buf[:PACKET_LEN])
            if pkt is None:
                self.bad_packets += 1
                del buf[0]          # slide by one byte and resync
                continue
            self.good_packets += 1
            packets.append(pkt)
            del buf[:PACKET_LEN]
        return packets


@dataclass
class CompletedScan:
    distances_mm: List[Optional[int]]   # 360 entries, None = no valid return
    strengths: List[int]
    warnings: List[bool]
    start_time: float
    end_time: float
    rpm: float
    packet_count: int


@dataclass
class ScanAssembler:
    """Groups packets into full revolutions (index wrap = new revolution)."""

    _last_index: Optional[int] = None
    _distances: List[Optional[int]] = field(default_factory=list)
    _strengths: List[int] = field(default_factory=list)
    _warnings: List[bool] = field(default_factory=list)
    _rpms: List[float] = field(default_factory=list)
    _start: float = 0.0
    _last_time: float = 0.0
    _count: int = 0

    def __post_init__(self) -> None:
        self._reset()

    def _reset(self) -> None:
        self._distances = [None] * READINGS_PER_REV
        self._strengths = [0] * READINGS_PER_REV
        self._warnings = [False] * READINGS_PER_REV
        self._rpms = []
        self._count = 0

    def add(self, pkt: Packet, t: float) -> Optional[CompletedScan]:
        """Add a packet received at time ``t``. Returns a scan on wrap."""
        completed = None
        if self._last_index is not None and pkt.index <= self._last_index:
            completed = self._finish()
        if self._count == 0:
            self._start = t
        for r in pkt.readings:
            if r.valid:
                self._distances[r.angle_deg] = r.distance_mm
            self._strengths[r.angle_deg] = r.strength
            self._warnings[r.angle_deg] = r.strength_warning
        self._rpms.append(pkt.rpm)
        self._count += 1
        self._last_index = pkt.index
        self._last_time = t
        return completed

    def _finish(self) -> Optional[CompletedScan]:
        if self._count == 0:
            return None
        scan = CompletedScan(
            distances_mm=self._distances,
            strengths=self._strengths,
            warnings=self._warnings,
            start_time=self._start,
            end_time=self._last_time,
            rpm=sum(self._rpms) / len(self._rpms),
            packet_count=self._count,
        )
        self._reset()
        return scan


def scan_to_ranges(scan: CompletedScan, range_min: float, range_max: float,
                   reverse: bool = False, drop_warnings: bool = False
                   ) -> Tuple[List[float], List[float]]:
    """Convert to ROS LaserScan ``ranges``/``intensities`` (metres).

    Bin k corresponds to angle ``angle_min + k * 1deg`` (CCW positive, REP 103).
    If the sensor's angles increase clockwise when viewed from above in your
    mounting, set ``reverse`` so that bin k holds sensor angle (360 - k).
    Invalid / out-of-range readings become +inf (REP 117 "no return").
    """
    ranges = [math.inf] * READINGS_PER_REV
    intensities = [0.0] * READINGS_PER_REV
    for a in range(READINGS_PER_REV):
        k = (READINGS_PER_REV - a) % READINGS_PER_REV if reverse else a
        intensities[k] = float(scan.strengths[a])
        d = scan.distances_mm[a]
        if d is None or (drop_warnings and scan.warnings[a]):
            continue
        r = d / 1000.0
        if range_min <= r <= range_max:
            ranges[k] = r
    return ranges, intensities

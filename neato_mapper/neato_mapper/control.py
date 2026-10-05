"""Pure-Python control and kinematics helpers (no ROS / hardware imports)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


class PID:
    """PID with output clamping, conditional-integration anti-windup and
    derivative-on-measurement (no derivative kick on setpoint changes)."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float, out_max: float, bias: float = 0.0) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.bias = bias            # feed-forward / nominal output
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._last_meas: Optional[float] = None

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3
        error = setpoint - measurement
        deriv = 0.0
        if self._last_meas is not None:
            deriv = -(measurement - self._last_meas) / dt
        self._last_meas = measurement

        unclamped = (self.bias + self.kp * error
                     + self._integral + self.ki * error * dt + self.kd * deriv)
        out = min(self.out_max, max(self.out_min, unclamped))
        # Only integrate if not saturated in the direction of the error.
        if out == unclamped or (unclamped > out) != (error > 0):
            self._integral += self.ki * error * dt
        return out


def slew(current: float, target: float, max_step: float) -> float:
    """Move ``current`` towards ``target`` by at most ``max_step``."""
    if target > current + max_step:
        return current + max_step
    if target < current - max_step:
        return current - max_step
    return target


def twist_to_wheels(v: float, w: float, track: float) -> Tuple[float, float]:
    """Body (v m/s, w rad/s) -> (left, right) wheel linear speeds in m/s."""
    return v - w * track / 2.0, v + w * track / 2.0


def wheels_to_twist(vl: float, vr: float, track: float) -> Tuple[float, float]:
    return (vl + vr) / 2.0, (vr - vl) / track


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class DiffDriveOdometry:
    """Integrates wheel travel into a 2D pose using the exact arc model."""

    def __init__(self, track: float) -> None:
        self.track = track
        self.pose = Pose2D()

    def update(self, d_left: float, d_right: float) -> Tuple[float, float]:
        """Advance by wheel distances (m). Returns (d_center, d_theta)."""
        d = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / self.track
        p = self.pose
        if abs(dth) < 1e-9:
            p.x += d * math.cos(p.theta)
            p.y += d * math.sin(p.theta)
        else:
            r = d / dth
            p.x += r * (math.sin(p.theta + dth) - math.sin(p.theta))
            p.y -= r * (math.cos(p.theta + dth) - math.cos(p.theta))
        p.theta = normalize_angle(p.theta + dth)
        return d, dth


# Gray-code transition table: index = (prev_state << 2) | new_state,
# value = count delta. Invalid (double) transitions count as 0.
_QUAD_TABLE = (
    0, +1, -1, 0,
    -1, 0, 0, +1,
    +1, 0, 0, -1,
    0, -1, +1, 0,
)


class QuadratureDecoder:
    """x4 quadrature decoding from A/B edge events."""

    def __init__(self, a: int = 0, b: int = 0, invert: bool = False) -> None:
        self.state = (a << 1) | b
        self.count = 0
        self.errors = 0
        self._sign = -1 if invert else 1

    def update(self, a: int, b: int) -> None:
        new = (a << 1) | b
        if new == self.state:
            return
        delta = _QUAD_TABLE[(self.state << 2) | new]
        if delta == 0:
            self.errors += 1     # missed an edge (both lines changed)
        self.count += self._sign * delta
        self.state = new

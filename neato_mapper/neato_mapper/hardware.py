"""Thin hardware abstraction over lgpio (works on Pi 4 and Pi 5, Ubuntu and
Raspberry Pi OS) with a mock backend for development off the robot.

Nothing in here is ROS-specific.
"""

from __future__ import annotations

import os
import threading
from typing import Callable, Dict, Optional

EdgeCallback = Callable[[int, int], None]   # (gpio, level)


class GpioBackend:
    def output(self, pin: int, level: int = 0) -> None: ...
    def write(self, pin: int, level: int) -> None: ...
    def pwm(self, pin: int, freq_hz: float, duty_0_1: float) -> None: ...
    def watch(self, pin: int, cb: EdgeCallback, pull_up: bool = True,
              debounce_us: int = 0) -> int:
        """Call ``cb`` on both edges; returns the current level."""
        ...
    def close(self) -> None: ...


class LgpioBackend(GpioBackend):
    def __init__(self, chip: int = 0) -> None:
        import lgpio  # imported lazily so the package imports on any machine
        self._lg = lgpio
        self._h = lgpio.gpiochip_open(chip)
        self._cbs = []
        self._lock = threading.Lock()

    def output(self, pin: int, level: int = 0) -> None:
        self._lg.gpio_claim_output(self._h, pin, level)

    def write(self, pin: int, level: int) -> None:
        self._lg.gpio_write(self._h, pin, 1 if level else 0)

    def pwm(self, pin: int, freq_hz: float, duty_0_1: float) -> None:
        duty = max(0.0, min(1.0, duty_0_1)) * 100.0
        with self._lock:
            self._lg.tx_pwm(self._h, pin, freq_hz, duty)

    def watch(self, pin: int, cb: EdgeCallback, pull_up: bool = True,
              debounce_us: int = 0) -> int:
        lg = self._lg
        flags = lg.SET_PULL_UP if pull_up else 0
        lg.gpio_claim_alert(self._h, pin, lg.BOTH_EDGES, flags)
        if debounce_us:
            lg.gpio_set_debounce_micros(self._h, pin, debounce_us)

        def _cb(_chip, gpio, level, _tick):
            if level in (0, 1):          # 2 == watchdog timeout, ignore
                cb(gpio, level)

        self._cbs.append(lg.callback(self._h, pin, lg.BOTH_EDGES, _cb))
        return lg.gpio_read(self._h, pin)

    def close(self) -> None:
        for c in self._cbs:
            c.cancel()
        self._cbs.clear()
        self._lg.gpiochip_close(self._h)


class MockBackend(GpioBackend):
    """Records outputs; lets a simulator inject edges via :meth:`inject`."""

    def __init__(self) -> None:
        self.levels: Dict[int, int] = {}
        self.duty: Dict[int, float] = {}
        self._watchers: Dict[int, EdgeCallback] = {}

    def output(self, pin: int, level: int = 0) -> None:
        self.levels[pin] = level

    def write(self, pin: int, level: int) -> None:
        self.levels[pin] = 1 if level else 0

    def pwm(self, pin: int, freq_hz: float, duty_0_1: float) -> None:
        self.duty[pin] = max(0.0, min(1.0, duty_0_1))

    def watch(self, pin: int, cb: EdgeCallback, pull_up: bool = True,
              debounce_us: int = 0) -> int:
        self._watchers[pin] = cb
        return self.levels.setdefault(pin, 0)

    def inject(self, pin: int, level: int) -> None:
        if self.levels.get(pin) == level:
            return
        self.levels[pin] = level
        if pin in self._watchers:
            self._watchers[pin](pin, level)

    def close(self) -> None:
        pass


def make_backend(kind: str, chip: int = 0) -> GpioBackend:
    if kind == "lgpio":
        return LgpioBackend(chip)
    if kind == "mock":
        return MockBackend()
    raise ValueError(f"unknown gpio backend '{kind}' (use lgpio or mock)")


class SysfsHardwarePwm:
    """Kernel hardware PWM via /sys/class/pwm (needs ``dtoverlay=pwm`` in
    config.txt). Jitter-free, used optionally for the LiDAR motor."""

    def __init__(self, chip: int, channel: int, freq_hz: float) -> None:
        self._base = f"/sys/class/pwm/pwmchip{chip}"
        self._dir = f"{self._base}/pwm{channel}"
        if not os.path.isdir(self._dir):
            self._put(f"{self._base}/export", str(channel))
            for _ in range(50):                 # wait for udev
                if os.access(f"{self._dir}/period", os.W_OK):
                    break
                threading.Event().wait(0.02)
        self._period = int(1e9 / freq_hz)
        self._put(f"{self._dir}/duty_cycle", "0")
        self._put(f"{self._dir}/period", str(self._period))
        self._put(f"{self._dir}/enable", "1")

    @staticmethod
    def _put(path: str, value: str) -> None:
        with open(path, "w") as f:
            f.write(value)

    def set(self, duty_0_1: float) -> None:
        duty = max(0.0, min(1.0, duty_0_1))
        self._put(f"{self._dir}/duty_cycle", str(int(self._period * duty)))

    def close(self) -> None:
        try:
            self.set(0.0)
            self._put(f"{self._dir}/enable", "0")
        except OSError:
            pass


class MotorChannel:
    """One H-bridge channel: IN1/IN2 for direction, PWM pin for speed.

    Works for TB6612FNG (AIN1/AIN2/PWMA) and L298N (IN1/IN2/ENA).
    """

    def __init__(self, gpio: GpioBackend, in1: int, in2: int, pwm_pin: int,
                 freq_hz: float, invert: bool = False,
                 brake_on_zero: bool = False) -> None:
        self.gpio, self.in1, self.in2, self.pwm_pin = gpio, in1, in2, pwm_pin
        self.freq = freq_hz
        self.invert = invert
        self.brake_on_zero = brake_on_zero
        for p in (in1, in2, pwm_pin):
            gpio.output(p, 0)
        self.duty = 0.0

    def set(self, duty: float) -> None:
        """duty in [-1, 1]; sign is direction."""
        duty = max(-1.0, min(1.0, duty))
        if self.invert:
            duty = -duty
        self.duty = duty
        if duty > 0:
            self.gpio.write(self.in1, 1)
            self.gpio.write(self.in2, 0)
        elif duty < 0:
            self.gpio.write(self.in1, 0)
            self.gpio.write(self.in2, 1)
        else:
            lvl = 1 if self.brake_on_zero else 0
            self.gpio.write(self.in1, lvl)
            self.gpio.write(self.in2, lvl)
        self.gpio.pwm(self.pwm_pin, self.freq, abs(duty))

    def stop(self) -> None:
        self.set(0.0)


class WheelEncoder:
    """Counts ticks from one wheel.

    Quadrature (A+B, x4 decoding) by default. ``b_pin < 0`` selects
    single-channel mode: every A edge is a tick and its sign comes from
    ``direction``, which the controller sets from the commanded wheel speed.
    """

    def __init__(self, gpio: GpioBackend, a_pin: int, b_pin: int,
                 invert: bool, debounce_us: int = 0) -> None:
        from .control import QuadratureDecoder
        self._lock = threading.Lock()
        self.single = b_pin < 0
        self.direction = 1
        self.a_pin, self.b_pin = a_pin, b_pin
        self._count = 0
        self._dec = QuadratureDecoder(0, 0, invert)
        self._levels = {a_pin: 0, b_pin: 0}
        # Register callbacks only after all state exists (edges can arrive
        # immediately on the lgpio callback thread).
        with self._lock:
            self._levels[a_pin] = gpio.watch(a_pin, self._edge, True, debounce_us)
            if not self.single:
                self._levels[b_pin] = gpio.watch(b_pin, self._edge, True, debounce_us)
            self._dec.state = (self._levels[a_pin] << 1) | self._levels[b_pin]

    def _edge(self, pin: int, level: int) -> None:
        with self._lock:
            self._levels[pin] = level
            if self.single:
                self._count += self.direction
            else:
                self._dec.update(self._levels[self.a_pin], self._levels[self.b_pin])

    def add_ticks(self, n: int) -> None:
        """Simulator hook (counts are in the corrected, post-invert frame)."""
        with self._lock:
            if self.single:
                self._count += n
            else:
                self._dec.count += n

    @property
    def count(self) -> int:
        with self._lock:
            return self._count if self.single else self._dec.count

    @property
    def errors(self) -> int:
        return 0 if self.single else self._dec.errors

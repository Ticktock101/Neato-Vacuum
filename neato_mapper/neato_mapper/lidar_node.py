"""ROS 2 driver for the Neato XV-11 / Botvac LDS with closed-loop spin control.

Publishes:
  /scan        sensor_msgs/LaserScan   (one message per revolution, ~5 Hz)
  /lidar/rpm   std_msgs/Float32        (measured turret speed, for tuning)
"""

from __future__ import annotations

import math
import time

import rclpy
import rclpy.duration
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from .control import PID
from .hardware import SysfsHardwarePwm, make_backend
from .xv11_protocol import (PACKETS_PER_REV, READINGS_PER_PACKET,
                            READINGS_PER_REV, PacketParser, ScanAssembler,
                            encode_packet, scan_to_ranges)


class _FakeLds:
    """Simulates the sensor: a motor with first-order lag whose speed depends
    on PWM duty, emitting real protocol bytes for a 4 m x 3 m room."""

    def __init__(self, rpm_per_duty: float = 520.0, tau: float = 0.4) -> None:
        self.rpm_per_duty, self.tau = rpm_per_duty, tau
        self.rpm = 0.0
        self.duty = 0.0
        self._pkt_accum = 0.0
        self._index = 0

    @staticmethod
    def _room_mm(angle_deg: float) -> int:
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        hx, hy = 2.0, 1.5
        t = min(hx / abs(c) if abs(c) > 1e-6 else 1e9,
                hy / abs(s) if abs(s) > 1e-6 else 1e9)
        return int(t * 1000)

    def read(self, dt: float) -> bytes:
        self.rpm += (self.duty * self.rpm_per_duty - self.rpm) * min(1.0, dt / self.tau)
        if self.rpm < 60:           # real LDS sends no packets when stalled
            return b""
        self._pkt_accum += self.rpm / 60.0 * PACKETS_PER_REV * dt
        out = bytearray()
        while self._pkt_accum >= 1.0:
            self._pkt_accum -= 1.0
            readings = []
            for j in range(READINGS_PER_PACKET):
                ang = self._index * READINGS_PER_PACKET + j
                readings.append((self._room_mm(ang), 800, False, False))
            out += encode_packet(self._index, self.rpm, readings)
            self._index = (self._index + 1) % PACKETS_PER_REV
        return bytes(out)


class NeatoLidarNode(Node):
    def __init__(self) -> None:
        super().__init__("neato_lidar")
        p = self.declare_parameter
        self.port = p("port", "/dev/serial0").value
        self.baud = p("baud", 115200).value
        self.frame_id = p("frame_id", "laser_frame").value
        self.fake = p("fake_lidar", False).value

        self.target_rpm = float(p("target_rpm", 300.0).value)
        self.min_publish_rpm = float(p("min_publish_rpm", 240.0).value)
        self.max_publish_rpm = float(p("max_publish_rpm", 330.0).value)
        self.range_min = float(p("range_min", 0.06).value)
        self.range_max = float(p("range_max", 5.0).value)
        self.reverse = bool(p("reverse_direction", False).value)
        self.angle_offset = math.radians(float(p("angle_offset_deg", 0.0).value))
        self.drop_warnings = bool(p("drop_strength_warnings", False).value)

        # Motor drive
        self.motor_backend = p("motor_backend", "lgpio").value  # lgpio|sysfs|mock
        self.motor_pin = int(p("motor_pin", 18).value)
        self.pwm_freq = float(p("motor_pwm_hz", 1000.0).value)
        self.gpio_chip = int(p("gpio_chip", 0).value)
        self.sysfs_chip = int(p("sysfs_pwm_chip", 0).value)
        self.sysfs_channel = int(p("sysfs_pwm_channel", 0).value)
        self.duty_min = float(p("duty_min", 0.0).value)
        self.duty_max = float(p("duty_max", 0.9).value)
        self.duty_ff = float(p("duty_feedforward", 0.55).value)
        self.pid = PID(kp=float(p("kp", 0.0010).value),
                       ki=float(p("ki", 0.0020).value),
                       kd=float(p("kd", 0.0).value),
                       out_min=self.duty_min, out_max=self.duty_max,
                       bias=self.duty_ff)

        self._open_motor()
        self._open_serial()

        self.parser = PacketParser()
        self.assembler = ScanAssembler()
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.rpm_pub = self.create_publisher(Float32, "lidar/rpm", 10)

        self._rpm_samples: list[float] = []
        self._last_packet_time = 0.0
        self._last_tick = time.monotonic()
        self._last_stats = self._last_tick
        self._scans = 0
        self.timer = self.create_timer(0.02, self._tick)   # 50 Hz
        self.get_logger().info(
            f"LDS on {'FAKE' if self.fake else self.port}, motor via "
            f"{self.motor_backend} pin {self.motor_pin}, target {self.target_rpm:.0f} RPM")

    # ---------------------------------------------------------------- setup
    def _open_motor(self) -> None:
        self._gpio = None
        self._hwpwm = None
        if self.fake:
            self.motor_backend = "mock"
        if self.motor_backend == "sysfs":
            self._hwpwm = SysfsHardwarePwm(self.sysfs_chip, self.sysfs_channel, self.pwm_freq)
        else:
            self._gpio = make_backend(self.motor_backend, self.gpio_chip)
            self._gpio.output(self.motor_pin, 0)
        self._duty = 0.0

    def _set_duty(self, duty: float) -> None:
        self._duty = duty
        if self.fake:
            self._fake.duty = duty
        if self._hwpwm:
            self._hwpwm.set(duty)
        else:
            self._gpio.pwm(self.motor_pin, self.pwm_freq, duty)

    def _open_serial(self) -> None:
        if self.fake:
            self._fake = _FakeLds()
            self._serial = None
            return
        import serial  # pyserial
        self._serial = serial.Serial(self.port, self.baud, timeout=0)
        self._serial.reset_input_buffer()

    # ----------------------------------------------------------------- loop
    def _read_bytes(self, dt: float) -> bytes:
        if self.fake:
            return self._fake.read(dt)
        n = self._serial.in_waiting
        return self._serial.read(n) if n else b""

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        for pkt in self.parser.feed(self._read_bytes(dt)):
            self._last_packet_time = now
            self._rpm_samples.append(pkt.rpm)
            scan = self.assembler.add(pkt, now)
            if scan is not None:
                self._publish(scan)

        # --- spin control ---------------------------------------------
        if now - self._last_packet_time > 0.5:
            # No packets: turret stopped or still spinning up. Run open loop
            # at the feed-forward duty and keep the integrator clean.
            self.pid.reset()
            self._set_duty(self.duty_ff)
        elif self._rpm_samples:
            rpm = sum(self._rpm_samples) / len(self._rpm_samples)
            self._rpm_samples.clear()
            self._set_duty(self.pid.update(self.target_rpm, rpm, dt))
            self.rpm_pub.publish(Float32(data=float(rpm)))

        if now - self._last_stats > 10.0:
            self._last_stats = now
            self.get_logger().info(
                f"scans/10s={self._scans} duty={self._duty:.2f} "
                f"good={self.parser.good_packets} bad={self.parser.bad_packets}")
            self._scans = 0

    def _publish(self, scan) -> None:
        if not self.min_publish_rpm <= scan.rpm <= self.max_publish_rpm:
            self.get_logger().warn(
                f"Dropping scan: {scan.rpm:.0f} RPM outside "
                f"[{self.min_publish_rpm:.0f}, {self.max_publish_rpm:.0f}]",
                throttle_duration_sec=2.0)
            return
        if scan.packet_count < PACKETS_PER_REV * 0.8:
            return  # partial revolution (startup / dropped bytes)

        ranges, intensities = scan_to_ranges(
            scan, self.range_min, self.range_max, self.reverse, self.drop_warnings)
        scan_time = 60.0 / scan.rpm
        msg = LaserScan()
        # Stamp the start of the revolution (ROS convention).
        stamp = self.get_clock().now() - rclpy.duration.Duration(seconds=scan_time)
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        inc = 2.0 * math.pi / READINGS_PER_REV
        msg.angle_min = self.angle_offset
        msg.angle_max = self.angle_offset + inc * (READINGS_PER_REV - 1)
        msg.angle_increment = inc
        msg.time_increment = scan_time / READINGS_PER_REV
        msg.scan_time = scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intensities
        self.scan_pub.publish(msg)
        self._scans += 1

    def shutdown(self) -> None:
        try:
            self._set_duty(0.0)
        finally:
            if self._hwpwm:
                self._hwpwm.close()
            if self._gpio:
                self._gpio.close()
            if self._serial:
                self._serial.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NeatoLidarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

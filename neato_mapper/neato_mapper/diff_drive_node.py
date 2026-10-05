"""Differential-drive base controller + wheel odometry for Neato drive modules.

Subscribes:
  /cmd_vel   geometry_msgs/Twist
Publishes:
  /odom      nav_msgs/Odometry
  TF         odom -> base_link   (disable with publish_tf:=false if you add an
                                  EKF such as robot_localization later)
"""

from __future__ import annotations

import math
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .control import (PID, DiffDriveOdometry, slew, twist_to_wheels,
                      wheels_to_twist)
from .hardware import MotorChannel, WheelEncoder, make_backend


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class DiffDriveNode(Node):
    def __init__(self) -> None:
        super().__init__("neato_base")
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731

        # Geometry / calibration
        self.track = float(p("wheel_track", 0.240))          # m, wheel centre to centre
        self.ticks_per_m = float(p("ticks_per_meter", 2000.0))  # CALIBRATE
        self.max_wheel_speed = float(p("max_wheel_speed", 0.35))  # m/s at duty 1.0
        self.max_lin = float(p("max_linear_speed", 0.25))
        self.max_ang = float(p("max_angular_speed", 1.5))
        self.max_lin_acc = float(p("max_linear_accel", 0.5))
        self.max_ang_acc = float(p("max_angular_accel", 3.0))
        self.cmd_timeout = float(p("cmd_vel_timeout", 0.5))
        self.rate = float(p("control_rate", 50.0))
        self.min_duty = float(p("min_duty", 0.12))           # overcome static friction
        self.speed_alpha = float(p("speed_filter_alpha", 0.3))

        # Frames
        self.odom_frame = p("odom_frame", "odom")
        self.base_frame = p("base_frame", "base_link")
        self.publish_tf = bool(p("publish_tf", True))

        # Hardware
        backend = p("gpio_backend", "lgpio")                 # lgpio | mock
        self.simulate = bool(p("simulate", False)) or backend == "mock"
        self.gpio = make_backend("mock" if self.simulate else backend,
                                 int(p("gpio_chip", 0)))
        freq = float(p("motor_pwm_hz", 2000.0))
        brake = bool(p("brake_on_zero", False))
        self.stby = int(p("stby_pin", 25))                   # TB6612 STBY, -1 if none
        if self.stby >= 0:
            self.gpio.output(self.stby, 1)
        self.left_motor = MotorChannel(
            self.gpio, int(p("left_in1", 5)), int(p("left_in2", 6)),
            int(p("left_pwm", 12)), freq, bool(p("left_motor_invert", False)), brake)
        self.right_motor = MotorChannel(
            self.gpio, int(p("right_in1", 16)), int(p("right_in2", 20)),
            int(p("right_pwm", 13)), freq, bool(p("right_motor_invert", True)), brake)
        deb = int(p("encoder_debounce_us", 0))
        self.left_enc = WheelEncoder(
            self.gpio, int(p("left_enc_a", 17)), int(p("left_enc_b", 27)),
            bool(p("left_encoder_invert", False)), deb)
        self.right_enc = WheelEncoder(
            self.gpio, int(p("right_enc_a", 22)), int(p("right_enc_b", 23)),
            bool(p("right_encoder_invert", True)), deb)

        # Wheel velocity loops (feed-forward + PI). Set kp=ki=0 for open loop.
        kp, ki = float(p("wheel_kp", 1.0)), float(p("wheel_ki", 4.0))
        self.left_pid = PID(kp, ki, 0.0, -1.0, 1.0)
        self.right_pid = PID(kp, ki, 0.0, -1.0, 1.0)

        cov = p("pose_covariance_diagonal", [0.001, 0.001, 1e6, 1e6, 1e6, 0.01])
        self.pose_cov = [float(c) for c in cov]

        self.odom = DiffDriveOdometry(self.track)
        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)

        self._target = (0.0, 0.0)
        self._cmd = (0.0, 0.0)          # after accel limiting
        self._last_cmd_time = 0.0
        self._prev_counts = (self.left_enc.count, self.right_enc.count)
        self._vel = (0.0, 0.0)          # filtered wheel speeds m/s
        self._last = time.monotonic()
        self._sim_accum = [0.0, 0.0]
        self.create_timer(1.0 / self.rate, self._loop)
        self.get_logger().info(
            f"Base ready ({'SIMULATED' if self.simulate else backend}); "
            f"track={self.track} m, ticks/m={self.ticks_per_m}")

    def _on_cmd(self, msg: Twist) -> None:
        v = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        w = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self._target = (v, w)
        self._last_cmd_time = time.monotonic()

    def _simulate_encoders(self, dt: float) -> None:
        for i, (motor, enc) in enumerate(((self.left_motor, self.left_enc),
                                           (self.right_motor, self.right_enc))):
            duty = -motor.duty if motor.invert else motor.duty
            self._sim_accum[i] += duty * self.max_wheel_speed * dt * self.ticks_per_m
            n = int(self._sim_accum[i])
            self._sim_accum[i] -= n
            enc.add_ticks(n)   # counts are stored post-invert, so no sign flip

    def _loop(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._last)
        self._last = now

        # ---------- command shaping (timeout + accel limits)
        target = self._target
        if now - self._last_cmd_time > self.cmd_timeout:
            target = (0.0, 0.0)
        v = slew(self._cmd[0], target[0], self.max_lin_acc * dt)
        w = slew(self._cmd[1], target[1], self.max_ang_acc * dt)
        self._cmd = (v, w)
        vl_t, vr_t = twist_to_wheels(v, w, self.track)

        # ---------- odometry
        if self.simulate:
            self._simulate_encoders(dt)
        cl, cr = self.left_enc.count, self.right_enc.count
        dl = (cl - self._prev_counts[0]) / self.ticks_per_m
        dr = (cr - self._prev_counts[1]) / self.ticks_per_m
        self._prev_counts = (cl, cr)
        self.odom.update(dl, dr)
        a = self.speed_alpha
        self._vel = (a * dl / dt + (1 - a) * self._vel[0],
                     a * dr / dt + (1 - a) * self._vel[1])

        # ---------- wheel control
        self.left_motor.set(self._wheel_duty(self.left_pid, vl_t, self._vel[0], dt))
        self.right_motor.set(self._wheel_duty(self.right_pid, vr_t, self._vel[1], dt))
        self.left_enc.direction = 1 if vl_t >= 0 else -1
        self.right_enc.direction = 1 if vr_t >= 0 else -1

        self._publish(dl / dt, dr / dt)

    def _wheel_duty(self, pid: PID, target: float, meas: float, dt: float) -> float:
        if abs(target) < 1e-4:
            pid.reset()
            return 0.0
        ff = target / self.max_wheel_speed
        pid.bias = ff
        duty = pid.update(target, meas, dt)
        if abs(duty) < self.min_duty:
            duty = math.copysign(self.min_duty, target)
        return duty

    def _publish(self, vl: float, vr: float) -> None:
        stamp = self.get_clock().now().to_msg()
        pose = self.odom.pose
        qx, qy, qz, qw = _yaw_to_quat(pose.theta)
        v, w = wheels_to_twist(vl, vr, self.track)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        for i, c in enumerate(self.pose_cov):
            msg.pose.covariance[i * 7] = c
            msg.twist.covariance[i * 7] = c
        self.odom_pub.publish(msg)

        if self.tf_pub:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = pose.x
            t.transform.translation.y = pose.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_pub.sendTransform(t)

    def shutdown(self) -> None:
        self.left_motor.stop()
        self.right_motor.stop()
        if self.stby >= 0:
            self.gpio.write(self.stby, 0)
        self.gpio.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiffDriveNode()
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

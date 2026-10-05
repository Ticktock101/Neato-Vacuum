"""Hardware bring-up utilities that do NOT need ROS running.

    neato_bringup lidar            # spin turret, show RPM / packet health
    neato_bringup motors           # pulse each wheel fwd/back to check wiring
    neato_bringup encoders         # live tick counts + edge rate
    neato_bringup calibrate        # push the robot exactly 1 m -> ticks_per_meter
    neato_bringup calibrate-track  # spin in place 10 turns -> wheel_track

Pin numbers default to config/robot.yaml defaults; override with flags.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from .control import PID
from .hardware import MotorChannel, WheelEncoder, make_backend
from .xv11_protocol import PacketParser, ScanAssembler


def _encoders(gpio, a):
    left = WheelEncoder(gpio, a.left_enc_a, a.left_enc_b, a.left_encoder_invert, 0)
    right = WheelEncoder(gpio, a.right_enc_a, a.right_enc_b, a.right_encoder_invert, 0)
    return left, right


def cmd_lidar(a) -> None:
    import serial
    gpio = make_backend("lgpio", a.chip)
    gpio.output(a.lidar_motor_pin, 0)
    ser = serial.Serial(a.port, 115200, timeout=0.05)
    parser, asm = PacketParser(), ScanAssembler()
    pid = PID(0.001, 0.002, 0.0, 0.0, 0.9, bias=a.duty)
    last_print = last = last_pkt = time.monotonic()
    rpms = []
    last_rpm = None
    print(f"Spinning LiDAR on GPIO{a.lidar_motor_pin}, reading {a.port}. Ctrl-C to stop.")
    try:
        duty = a.duty
        while True:
            data = ser.read(512)
            now = time.monotonic()
            for pkt in parser.feed(data):
                rpms.append(pkt.rpm)
                last_pkt = now
                scan = asm.add(pkt, now)
                if scan:
                    valid = [d for d in scan.distances_mm if d is not None]
                    front = scan.distances_mm[0]
                    print(f"  scan: {scan.packet_count} pkts, {len(valid)}/360 valid, "
                          f"0deg={front} mm, rpm={scan.rpm:.1f}")
            if rpms:
                last_rpm = sum(rpms) / len(rpms)
                rpms.clear()
                if not a.open_loop:
                    duty = pid.update(300.0, last_rpm, now - last)
            elif now - last_pkt > 0.5:
                pid.reset()
                duty = a.duty
                last_rpm = None
            gpio.pwm(a.lidar_motor_pin, 1000, duty)
            last = now
            if now - last_print > 1.0:
                r = f"{last_rpm:.1f}" if last_rpm else "no packets"
                print(f"duty={duty:.2f} rpm={r} good={parser.good_packets} "
                      f"bad={parser.bad_packets}")
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        gpio.pwm(a.lidar_motor_pin, 1000, 0.0)
        gpio.close()


def cmd_motors(a) -> None:
    gpio = make_backend("lgpio", a.chip)
    if a.stby_pin >= 0:
        gpio.output(a.stby_pin, 1)
    left = MotorChannel(gpio, a.left_in1, a.left_in2, a.left_pwm, 2000, a.left_motor_invert)
    right = MotorChannel(gpio, a.right_in1, a.right_in2, a.right_pwm, 2000, a.right_motor_invert)
    l_enc, r_enc = _encoders(gpio, a)
    print("PROP THE ROBOT UP SO THE WHEELS ARE OFF THE GROUND.")
    input("Press Enter to start...")
    try:
        for name, m, enc in (("LEFT", left, l_enc), ("RIGHT", right, r_enc)):
            for label, duty in (("forward", a.duty), ("backward", -a.duty)):
                c0 = enc.count
                print(f"{name} wheel {label} (should roll the robot {label})...")
                m.set(duty)
                time.sleep(1.5)
                m.stop()
                time.sleep(0.5)
                print(f"   encoder delta = {enc.count - c0:+d} "
                      f"(expect {'positive' if duty > 0 else 'negative'})")
        print("\nIf a wheel spun the wrong way flip *_motor_invert; if the encoder "
              "sign is wrong flip *_encoder_invert.")
    finally:
        left.stop()
        right.stop()
        if a.stby_pin >= 0:
            gpio.write(a.stby_pin, 0)
        gpio.close()


def cmd_encoders(a) -> None:
    gpio = make_backend("lgpio", a.chip)
    l, r = _encoders(gpio, a)
    print("Roll the wheels by hand. Ctrl-C to stop.")
    pl, pr, pt = l.count, r.count, time.monotonic()
    try:
        while True:
            time.sleep(0.5)
            t = time.monotonic()
            cl, cr = l.count, r.count
            print(f"left={cl:+8d} ({(cl - pl) / (t - pt):7.0f}/s, err {l.errors})   "
                  f"right={cr:+8d} ({(cr - pr) / (t - pt):7.0f}/s, err {r.errors})")
            pl, pr, pt = cl, cr, t
    except KeyboardInterrupt:
        pass
    finally:
        gpio.close()


def cmd_calibrate(a) -> None:
    gpio = make_backend("lgpio", a.chip)
    l, r = _encoders(gpio, a)
    input(f"Mark the floor. Press Enter, then push the robot STRAIGHT forward "
          f"exactly {a.distance} m, then press Enter again.")
    l0, r0 = l.count, r.count
    input("Pushing... press Enter when done.")
    dl, dr = l.count - l0, r.count - r0
    gpio.close()
    tpm = (dl + dr) / 2.0 / a.distance
    print(f"left ticks={dl}, right ticks={dr}")
    if dl <= 0 or dr <= 0:
        print("A count is not positive: fix *_encoder_invert first.")
    print(f"\n==> ticks_per_meter: {tpm:.1f}")
    if abs(dl - dr) > 0.03 * max(abs(dl), abs(dr), 1):
        print("Warning: wheels differ by >3% - check for slip or a bad encoder.")


def cmd_calibrate_track(a) -> None:
    gpio = make_backend("lgpio", a.chip)
    l, r = _encoders(gpio, a)
    input(f"Mark the robot's heading. Press Enter, then rotate it in place "
          f"(counter-clockwise) exactly {a.turns} full turns, then press Enter.")
    l0, r0 = l.count, r.count
    input("Rotating... press Enter when done.")
    dl, dr = (l.count - l0) / a.ticks_per_meter, (r.count - r0) / a.ticks_per_meter
    gpio.close()
    track = (dr - dl) / (2 * math.pi * a.turns)
    print(f"left={dl:.3f} m right={dr:.3f} m\n==> wheel_track: {track:.4f}")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="neato_bringup", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--chip", type=int, default=0, help="gpiochip (Pi 5 on old kernels: 4)")
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--lidar-motor-pin", type=int, default=18)
    ap.add_argument("--stby-pin", type=int, default=25)
    for side, (i1, i2, pw, ea, eb, minv, einv) in {
            "left": (5, 6, 12, 17, 27, False, False),
            "right": (16, 20, 13, 22, 23, True, True)}.items():
        ap.add_argument(f"--{side}-in1", type=int, default=i1)
        ap.add_argument(f"--{side}-in2", type=int, default=i2)
        ap.add_argument(f"--{side}-pwm", type=int, default=pw)
        ap.add_argument(f"--{side}-enc-a", type=int, default=ea)
        ap.add_argument(f"--{side}-enc-b", type=int, default=eb, help="-1 = single channel")
        ap.add_argument(f"--{side}-motor-invert", type=lambda s: s.lower() == "true", default=minv)
        ap.add_argument(f"--{side}-encoder-invert", type=lambda s: s.lower() == "true", default=einv)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("lidar")
    s.add_argument("--duty", type=float, default=0.55)
    s.add_argument("--open-loop", action="store_true")
    s = sub.add_parser("motors")
    s.add_argument("--duty", type=float, default=0.4)
    sub.add_parser("encoders")
    s = sub.add_parser("calibrate")
    s.add_argument("--distance", type=float, default=1.0)
    s = sub.add_parser("calibrate-track")
    s.add_argument("--turns", type=int, default=10)
    s.add_argument("--ticks-per-meter", type=float, required=True)
    a = ap.parse_args(argv)
    {"lidar": cmd_lidar, "motors": cmd_motors, "encoders": cmd_encoders,
     "calibrate": cmd_calibrate, "calibrate-track": cmd_calibrate_track}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())

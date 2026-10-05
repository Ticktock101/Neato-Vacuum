# neato_mapper

ROS 2 package for building a 2D mapping robot using salvaged Neato vacuum parts (the XV-11 / Botvac LiDAR and the differential-drive wheel assemblies) and a Raspberry Pi.

It includes:
1. `neato_lidar_node`: Reads the raw binary serial packets, adjusts the LiDAR motor PWM duty to hold exactly 300 RPM (required for accurate readings), and publishes `sensor_msgs/LaserScan` on `/scan`.
2. `diff_drive_node`: Accepts `geometry_msgs/Twist` commands on `/cmd_vel`, drives the wheels via an H-Bridge, counts wheel encoder ticks to generate wheel odometry, and publishes `nav_msgs/Odometry` on `/odom` and the `odom -> base_link` TF transform.
3. `neato_bringup`: A standalone (no ROS needed) Python CLI to test motor directions and calibrate the encoders.
4. Launch files tuned for `slam_toolbox` mapping.

## Hardware & Wiring

This package uses `lgpio` by default, which works on both Raspberry Pi OS and Ubuntu on Pi 4 and Pi 5.

### 1. The LiDAR
- **Orange (TX):** Connect to Pi GPIO 15 (UART RX) via a 3.3V logic level converter if needed (though typically Neato TX is 3.3V).
- **Red:** 5V
- **Black:** GND
- **LiDAR Motor:** Power via an N-Channel MOSFET or a spare motor driver channel using a 3.3V-5V supply. Connect the gate/PWM pin to Pi GPIO 18.
- *Note:* Ensure you disable the serial console in `raspi-config` but leave the serial port hardware enabled.

### 2. The Drive Motors
- Power via a dual H-Bridge (e.g. L298N or TB6612FNG) from a 7.4V - 12V battery.
- Connect IN1, IN2, and PWM for each side to Pi GPIOs.
- **Encoders:** Neato wheels have built-in magnetic quadrature encoders. Connect Phase A and Phase B to Pi GPIOs. 
- *Check `config/robot.yaml` for default pin mappings.*

## Pre-Flight Calibration (No ROS Required)

Before launching ROS, you must calibrate the motor directions and odometry ticks.

```bash
# 1. Test LiDAR spin and packet parsing
ros2 run neato_mapper neato_bringup lidar

# 2. Test motor directions (prop the robot up!)
ros2 run neato_mapper neato_bringup motors

# 3. Test encoder tick counting
ros2 run neato_mapper neato_bringup encoders

# 4. Calibrate ticks per meter (push robot exactly 1m)
ros2 run neato_mapper neato_bringup calibrate --distance 1.0

# 5. Calibrate wheel track (spin robot 10 times)
ros2 run neato_mapper neato_bringup calibrate-track --turns 10 --ticks-per-meter <result_from_step_4>
```
Update `config/robot.yaml` with the measured `ticks_per_meter` and `wheel_track`, and flip `*_invert` booleans if a motor or encoder ran backwards.

## Running

1. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select neato_mapper
   source install/setup.bash
   ```

2. **Launch the robot hardware:**
   ```bash
   ros2 launch neato_mapper robot.launch.py
   ```
   *(To test without hardware, run: `ros2 launch neato_mapper robot.launch.py sim:=true`)*

3. **Launch SLAM:**
   ```bash
   ros2 launch neato_mapper mapping.launch.py
   ```

4. **Drive and Map:**
   In a new terminal:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Open RViz on your PC, add a Map display subscribed to `/map`, add a LaserScan display for `/scan`, and set the fixed frame to `map`. Drive around to build the map!

5. **Save the map:**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_house
   ```


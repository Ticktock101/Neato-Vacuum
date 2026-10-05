"""Robot bringup: LiDAR driver + base controller + base_link->laser_frame TF.

    ros2 launch neato_mapper robot.launch.py
    ros2 launch neato_mapper robot.launch.py sim:=true     # no hardware needed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("neato_mapper")
    default_cfg = os.path.join(share, "config", "robot.yaml")

    sim = LaunchConfiguration("sim")
    cfg = LaunchConfiguration("config")
    sim_bool = ParameterValue(sim, value_type=bool)

    args = [
        DeclareLaunchArgument("sim", default_value="false",
                              description="Fake LiDAR + simulated wheels (no GPIO)"),
        DeclareLaunchArgument("config", default_value=default_cfg),
        # MEASURE these on your robot: offset of the LiDAR spin axis from the
        # midpoint between the drive wheels (x forward, y left, z up), metres.
        DeclareLaunchArgument("laser_x", default_value="-0.090"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.090"),
        # Rotate until a wall directly in front of the robot shows up in
        # front of it in RViz (try 0, 1.5708, 3.1416, -1.5708).
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
    ]

    lidar = Node(
        package="neato_mapper", executable="lidar_node", name="neato_lidar",
        output="screen",
        parameters=[cfg, {"fake_lidar": sim_bool}],
        respawn=True, respawn_delay=2.0,
    )
    base = Node(
        package="neato_mapper", executable="diff_drive_node", name="neato_base",
        output="screen",
        parameters=[cfg, {"simulate": sim_bool}],
        respawn=True, respawn_delay=2.0,
    )
    laser_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_laser",
        arguments=[
            "--x", LaunchConfiguration("laser_x"),
            "--y", LaunchConfiguration("laser_y"),
            "--z", LaunchConfiguration("laser_z"),
            "--yaw", LaunchConfiguration("laser_yaw"),
            "--pitch", "0", "--roll", "0",
            "--frame-id", "base_link", "--child-frame-id", "laser_frame",
        ],
    )
    return LaunchDescription(args + [lidar, base, laser_tf])

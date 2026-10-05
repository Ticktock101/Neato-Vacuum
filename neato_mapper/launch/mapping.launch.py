"""Full mapping stack: robot bringup + slam_toolbox (online async).

    ros2 launch neato_mapper mapping.launch.py
    # drive it (separate terminal, needs a TTY):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/house
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("neato_mapper")
    slam_share = get_package_share_directory("slam_toolbox")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "robot.launch.py")),
        launch_arguments={"sim": LaunchConfiguration("sim")}.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "use_sim_time": "false",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        robot,
        slam,
    ])

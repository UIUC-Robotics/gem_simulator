#!/usr/bin/env python3
"""Launch keyboard teleop (drive.py) for GEM. Run after sim is up. Uses script defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )
    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        default_value="6.0",
        description="Max speed (m/s) for keyboard teleop",
    )

    drive_node = Node(
        package="gem_gazebo",
        executable="drive.py",
        name="drive",
        output="screen",
        arguments=["--max_speed", LaunchConfiguration("max_speed")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        max_speed_arg,
        drive_node,
    ])

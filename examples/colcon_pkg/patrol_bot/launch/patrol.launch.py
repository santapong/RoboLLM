"""Launch file — start the patrol node with parameters, the ROS 2 way.

    ros2 launch patrol_bot patrol.launch.py
    ros2 launch patrol_bot patrol.launch.py side_m:=0.6 laps:=2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side_m", default_value="0.45"),
        DeclareLaunchArgument("laps", default_value="1"),
        Node(
            package="patrol_bot",
            executable="patrol",
            parameters=[{
                "side_m": LaunchConfiguration("side_m"),
                "laps": LaunchConfiguration("laps"),
            }],
            output="screen",
        ),
    ])

"""Launch the physical arm driver with an explicit calibration profile."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robo_arm_driver"))
    default_config = str(share / "config" / "joints.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=""),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("enable_on_start", default_value="true"),
        Node(
            package="robo_arm_driver",
            executable="arm_bridge",
            name="robo_arm_driver",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "config_file": LaunchConfiguration("config_file"),
                "enable_on_start": LaunchConfiguration("enable_on_start"),
            }],
        ),
    ])

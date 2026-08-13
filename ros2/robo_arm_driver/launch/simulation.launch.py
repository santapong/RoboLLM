"""Connect the arm driver to a separately started pseudo-terminal Uno."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robo_arm_driver"))
    sim_config = str(share / "config" / "joints.sim.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            description="PTY printed by hardware/sim_uno.py (for example /dev/pts/4)",
        ),
        Node(
            package="robo_arm_driver",
            executable="arm_bridge",
            name="robo_arm_driver_sim",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "config_file": sim_config,
                "enable_on_start": True,
            }],
        ),
    ])

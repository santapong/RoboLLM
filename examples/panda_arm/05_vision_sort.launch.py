"""Vision-guided sorting v2: camera solves positions from pixels.

Usage:
    ros2 launch examples/panda_arm/05_vision_sort.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

DEMO_DIR = os.path.dirname(__file__)


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'),
        'urdf', 'panda.urdf',
    )
    with open(urdf_path) as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(DEMO_DIR, 'panda.rviz')],
        ),
        ExecuteProcess(
            cmd=['python3', os.path.join(DEMO_DIR, 'arduino_sim.py')],
            output='screen',
        ),
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=['python3', os.path.join(DEMO_DIR, 'vision_sort_demo.py')],
                output='screen',
            )],
        ),
    ])

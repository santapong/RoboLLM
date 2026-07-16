"""Pick-and-place demo: robot_state_publisher + RViz2 + virtual Arduino + task GUI.

Usage:
    ros2 launch examples/panda_arm/03_pick_place.launch.py

With a real Arduino, skip arduino_sim and set PANDA_SERIAL=/dev/ttyUSB0.
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
        TimerAction(  # give the virtual serial port a moment to appear
            period=2.0,
            actions=[ExecuteProcess(
                cmd=['python3', os.path.join(DEMO_DIR, 'pick_place.py')],
                output='screen',
            )],
        ),
    ])

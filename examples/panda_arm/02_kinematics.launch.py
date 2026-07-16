"""Panda FK/IK demo: robot_state_publisher + RViz2 + kinematics GUI.

The kinematics GUI publishes /joint_states itself, so joint_state_publisher
is intentionally NOT launched here.

Usage:
    ros2 launch examples/panda_arm/02_kinematics.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
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
            cmd=['python3', os.path.join(DEMO_DIR, 'kinematics_gui.py')],
            output='screen',
        ),
    ])

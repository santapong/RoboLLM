"""move_group only, for the ROBOTIS FFW semi-humanoid.

Use mock_bringup.launch.py for the full stack (RSP + ros2_control + RViz).
This file exists so move_group can be brought up alone for headless checks.

The corrected MoveItConfigsBuilder chain lives in humanoid_mirror.ffw_config —
see that module for why ffw_moveit_config's own moveit.launch.py crashes.

EXPECTED BENIGN WARNINGS: move_group logs 7 lines of
"Link 'lift_link' / '*_wheel_*_link' is not known to URDF. Cannot disable/enable
collisons." Those are disable_collisions entries for links that exist only on
the SG2 swerve variant. Harmless — do NOT "fix" them by switching to SG2, which
would swap 7 cosmetic warnings for a real robot-name mismatch and 3 broken
meshes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from humanoid_mirror.ffw_config import build_moveit_configs


def _launch_setup(context, *args, **kwargs):
    mock = LaunchConfiguration("use_mock_hardware").perform(context) == "true"
    moveit_configs = build_moveit_configs(use_mock_hardware=mock)

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_configs.to_dict(),
                {"publish_robot_description_semantic": True},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description=(
                    "true = mock_components/GenericSystem (no robot, no GPU). "
                    "false = the real dynamixel_hardware_interface on /dev/follower."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

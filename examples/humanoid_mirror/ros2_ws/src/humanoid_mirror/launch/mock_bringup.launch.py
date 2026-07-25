"""Full mock-hardware bring-up of the FFW semi-humanoid: RSP + ros2_control +
four JointTrajectoryControllers + move_group + RViz.

This is the `ros2-arm humanoid` verb — the robot on its own, with no tracking.
It is milestone M1: prove the humanoid loads, renders, and plans before any
vision code exists.

Same shape as gen3_pick_place's demo bring-up, with one structural difference:
FFW has FOUR controllers (arm_l, arm_r, head, lift) over disjoint joint sets
instead of one arm + one gripper. They are spawned sequentially after
joint_state_broadcaster, because a controller that activates before the
broadcaster occasionally races on the state interfaces.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from humanoid_mirror.ffw_config import build_moveit_configs

# Controllers, in spawn order. joint_state_broadcaster MUST be first.
CONTROLLERS = [
    "arm_l_controller",
    "arm_r_controller",
    "head_controller",
    "lift_controller",
]


def _launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("rviz").perform(context) == "true"
    moveit_configs = build_moveit_configs(use_mock_hardware=True)

    pkg = get_package_share_directory("humanoid_mirror")
    controllers_yaml = os.path.join(pkg, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(pkg, "rviz", "mirror.rviz")

    nodes = [
        # robot_state_publisher — publishes TF from /joint_states.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_configs.robot_description],
        ),
        # ros2_control node. The URDF's <ros2_control> block resolves to
        # mock_components/GenericSystem because build_moveit_configs passed
        # use_mock_hardware:=true — without that mapping it would try to open
        # /dev/follower and talk Dynamixel to hardware that isn't there.
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[moveit_configs.robot_description, controllers_yaml],
        ),
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

    # Spawn joint_state_broadcaster first, then chain the four controllers off
    # its exit so they activate in a deterministic order.
    jsb = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"],
        output="screen",
    )
    nodes.append(jsb)

    previous = jsb
    for name in CONTROLLERS:
        spawner = ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", name],
            output="screen",
        )
        nodes.append(
            RegisterEventHandler(
                OnProcessExit(target_action=previous, on_exit=[spawner])
            )
        )
        previous = spawner

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[
                    moveit_configs.robot_description,
                    moveit_configs.robot_description_semantic,
                    moveit_configs.robot_description_kinematics,
                    moveit_configs.planning_pipelines,
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="launch RViz2"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

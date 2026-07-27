"""Full mock-hardware bring-up of TALOS: robot_state_publisher +
ros2_control + six JointTrajectoryControllers + move_group + RViz.

Same shape as humanoid_mirror's mock_bringup.launch.py for FFW, with one
structural difference: TALOS has SIX controllers (torso, arm_l, arm_r,
head, leg_l, leg_r) over disjoint joint sets instead of FFW's four. They
are spawned sequentially after joint_state_broadcaster, chained on
OnProcessExit, because a controller that activates before the broadcaster
occasionally races on the state interfaces.

The hardware is unconditionally mock_components/GenericSystem -- see
config/mock_hardware.ros2_control.xacro for why there is no real-hardware
switch to pass through here (this package has no PAL EtherCAT driver
vendored, unlike FFW's dynamixel_hardware_interface which is on the apt
image, just unreachable without real servos).
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
from moveit_configs_utils import MoveItConfigsBuilder

# Controllers, in spawn order. joint_state_broadcaster MUST be first.
CONTROLLERS = [
    "torso_controller",
    "arm_l_controller",
    "arm_r_controller",
    "head_controller",
    "leg_l_controller",
    "leg_r_controller",
]


def build_moveit_configs():
    return (
        MoveItConfigsBuilder(robot_name="talos", package_name="talos_moveit_config")
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution()
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


def _launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    use_moveit = LaunchConfiguration("use_moveit").perform(context) == "true"
    moveit_configs = build_moveit_configs()

    pkg = get_package_share_directory("talos_moveit_config")
    controllers_yaml = os.path.join(pkg, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(pkg, "rviz", "talos_moveit.rviz")

    nodes = [
        # robot_state_publisher -- publishes TF from /joint_states.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_configs.robot_description],
        ),
        # ros2_control node. The URDF's <ros2_control> block is our own
        # mock_components/GenericSystem block (config/mock_hardware.ros2_control.xacro),
        # not talos_description's real one -- no /dev or EtherCAT is opened.
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[moveit_configs.robot_description, controllers_yaml],
        ),
    ]

    # move_group -- OPTIONAL (use_moveit:=false). Measured (see TECHNICAL.md
    # "CPU profile" section): a flat ~3-3.5% CPU tax from its
    # PlanningSceneMonitor subscribing /joint_states + /tf, paid whether or
    # not anything ever calls /compute_fk or /compute_ik. talos-check and
    # retarget-bench --fk are the only consumers of those services in this
    # package (grep-confirmed: neither track_node nor mirror_node ever calls
    # a MoveIt service), so this defaults true here (the `talos` verb, used
    # by talos-check) and defaults false at the mirror/sweep launch files
    # that include this one, since mirroring/sweeping never plans.
    if use_moveit:
        nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_configs.to_dict(),
                    {"publish_robot_description_semantic": True},
                ],
            )
        )

    # Spawn joint_state_broadcaster first, then chain the six controllers off
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
                "use_rviz", default_value="true", description="launch RViz2"
            ),
            DeclareLaunchArgument(
                "use_moveit", default_value="true",
                description="launch move_group. Only talos-check and retarget-bench "
                             "--fk need it (FK/IK services); mirroring/sweeping never "
                             "plans, so mirror.launch.py and sweep.launch.py default "
                             "this false to skip move_group's ~3-3.5% CPU "
                             "PlanningSceneMonitor tax (see TECHNICAL.md CPU profile). "
                             "This launch file's own default stays true (the `talos` "
                             "verb, used by talos-check).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

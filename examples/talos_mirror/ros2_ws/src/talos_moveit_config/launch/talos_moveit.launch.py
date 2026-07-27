"""move_group alone, for headless checks -- the TALOS analogue of
humanoid_mirror's ffw_moveit.launch.py.

Use mock_bringup.launch.py for the full stack (robot_state_publisher +
ros2_control + move_group + RViz). This file exists so move_group can be
brought up on its own, e.g. to answer /compute_fk and /compute_ik or run a
planning smoke test with nothing else in the graph.

use_rviz:=true (the default) also launches RViz with a MotionPlanning
display pointed at the arm_l group and a live RobotModel, so this demo
does not open on an empty window -- the same lesson humanoid_mirror's
TECHNICAL.md calls out for FFW ("a demo that opens on nothing wastes the
demo"). Note that without robot_state_publisher (this launch file does not
start one) the RobotModel display will show the robot at whatever state
MotionPlanning's own internal PlanningSceneMonitor last received; use
mock_bringup.launch.py if you want live TF too.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_configs():
    """The MoveItConfigsBuilder chain this package's config/ was authored for.

    config/talos.urdf.xacro (this package) already defaults use_mock_hardware
    to true internally via mock_hardware.ros2_control.xacro's unconditional
    mock_components/GenericSystem block -- see that file for why there is no
    mock/real toggle to pass through here, unlike FFW's ffw_config.py.
    """
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
    moveit_configs = build_moveit_configs()
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"

    nodes = [
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

    if use_rviz:
        from ament_index_python.packages import get_package_share_directory
        import os

        rviz_config = os.path.join(
            get_package_share_directory("talos_moveit_config"),
            "rviz",
            "talos_moveit.rviz",
        )
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
                "use_rviz",
                default_value="true",
                description="Launch RViz with the MotionPlanning display alongside move_group.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

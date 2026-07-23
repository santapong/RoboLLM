"""Gen3 lite pick-and-place bring-up (mock hardware, no kortex_driver).

There is no Jazzy kortex_driver binary, so the stock kinova demo.launch.py
crashes. This launch uses the VERIFIED custom bring-up instead:

  * URDF processed from kortex_description/robots/gen3_lite_gen3_lite_2f.xacro
    with use_fake_hardware:=true fake_sensor_commands:=true
    (2x mock_components/GenericSystem: arm + gripper)
  * robot_state_publisher + ros2_control_node
    (~/robot_description remapped to /robot_description)
  * joint_state_broadcaster + arm JTC + gripper JTC
    (gripper JTC replaces the stock GripperActionController, which silently
    ignores topic streams)
  * move_group via MoveItConfigsBuilder on kinova_gen3_lite_moveit_config
  * optional RViz (RobotModel + PlanningScene on /monitored_planning_scene)

  ros2 launch gen3_pick_place pickplace_demo.launch.py [rviz:=false]
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Processed-URDF drop point: move_group's MoveItConfigsBuilder and any
# debugging tools read the exact same file the controllers were started with.
PROCESSED_URDF = "/tmp/gen3_lite_pickplace.urdf"


def generate_launch_description():
    pkg_share = get_package_share_directory("gen3_pick_place")
    kortex_share = get_package_share_directory("kortex_description")

    xacro_file = os.path.join(kortex_share, "robots", "gen3_lite_gen3_lite_2f.xacro")
    robot_description = xacro.process_file(
        xacro_file,
        mappings={"use_fake_hardware": "true", "fake_sensor_commands": "true"},
    ).toxml()
    with open(PROCESSED_URDF, "w") as f:
        f.write(robot_description)

    moveit_config = (
        MoveItConfigsBuilder(
            "gen3_lite_gen3_lite_2f", package_name="kinova_gen3_lite_moveit_config"
        )
        .robot_description(file_path=PROCESSED_URDF)
        .robot_description_semantic(file_path="config/gen3_lite.srdf")
        .trajectory_execution(
            file_path=os.path.join(pkg_share, "config", "moveit_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start RViz"
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "ros2_controllers.yaml")],
        remappings=[("~/robot_description", "/robot_description")],
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
        )
        for name in (
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "gripper_controller",
        )
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(pkg_share, "rviz", "pickplace.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([rviz_arg, rsp, ros2_control, *spawners, move_group, rviz])

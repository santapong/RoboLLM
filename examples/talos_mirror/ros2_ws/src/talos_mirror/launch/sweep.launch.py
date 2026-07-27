"""M2 bring-up: TALOS mock hardware (talos_moveit_config's mock_bringup) +
the whole-body sine sweep + a live limit monitor, all in ONE launch file —
the `sweep` verb of docker/ros2-arm.

    ros2 launch talos_mirror sweep.launch.py duration:=10.0 use_rviz:=false

`duration` is forwarded to limit_monitor's `duration_s` parameter. After that
many seconds it prints a RESULT:{json} line and shuts itself down; sweep_node
and the rest of the stack keep running until the container/process is
stopped — this file does not try to tear down move_group/ros2_control
underneath itself, matching mock_bringup.launch.py's own lifecycle.

`start_delay` (default 25 s) is how long sweep_node and limit_monitor wait
for move_group + the six sequentially-chained controller spawners to finish
activating — six controllers chained on OnProcessExit is slower than FFW's
four, so this is longer than humanoid_mirror's 18 s default.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_pkg = get_package_share_directory("talos_moveit_config")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "mock_bringup.launch.py")
        ),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
    )

    sweep = Node(
        package="talos_mirror",
        executable="sweep_node",
        name="talos_sweep",
        output="screen",
        ros_arguments=[
            "-p", ["rate_hz:=", LaunchConfiguration("rate_hz")],
            "-p", ["period_s:=", LaunchConfiguration("period_s")],
        ],
    )

    monitor = Node(
        package="talos_mirror",
        executable="limit_monitor",
        name="talos_limit_monitor",
        output="screen",
        ros_arguments=[
            "-p", ["duration_s:=", LaunchConfiguration("duration")],
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz", default_value="false",
                description="launch RViz2 alongside the mock stack",
            ),
            DeclareLaunchArgument(
                "start_delay", default_value="25.0",
                description="seconds to wait for move_group + the six controllers",
            ),
            DeclareLaunchArgument(
                "duration", default_value="10.0",
                description="seconds the limit monitor watches /joint_states",
            ),
            DeclareLaunchArgument(
                "rate_hz", default_value="50.0", description="sweep publish rate",
            ),
            DeclareLaunchArgument(
                "period_s", default_value="10.0", description="sine sweep period",
            ),
            bringup,
            TimerAction(
                period=LaunchConfiguration("start_delay"),
                actions=[sweep, monitor],
            ),
        ]
    )

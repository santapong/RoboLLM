"""M3 bring-up: TALOS mock hardware (talos_moveit_config's mock_bringup, same
as `sweep` uses) + full-body webcam tracking (talos_mirror/track_node) --
the `track` verb of docker/ros2-arm. The robot stays PARKED: track_node
never commands a joint, it only publishes what it sees.

    ros2 launch talos_mirror track.launch.py synthetic:=true use_rviz:=false
    ros2 launch talos_mirror track.launch.py synthetic:=false preview:=true

Every track_node parameter is also a launch argument, per house convention,
forwarded as `-p name:=value` through ros_arguments (not a parameters dict)
so ROS's YAML type inference turns "30.0" into a double and "true" into a
bool -- a raw LaunchConfiguration substitution in `parameters=` would hand
the node a string and fail on the first float()/bool() call.

`start_delay` mirrors sweep.launch.py's reasoning: six controllers chained
on OnProcessExit take a while to finish activating, and track_node itself
does not touch any of them (it only needs robot_state_publisher's TF tree
to exist for `body_frame_id`'s parent link), so this is generous rather than
tuned tight.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default, description) -- forwarded to track_node as ROS parameters.
NODE_ARGS = [
    ("synthetic", "true", "deterministic full-body motion generator instead of the camera"),
    ("rate_hz", "30.0", "synthetic-mode publish tick rate"),
    ("sweep_period_s", "16.0", "synthetic full-body motion cycle length"),
    ("camera", "/dev/video0", "video capture device (camera mode only)"),
    ("pose_model_path", "/opt/models/pose_landmarker_full.task",
     "mediapipe pose_landmarker .task model file"),
    ("min_visibility", "0.5", "per-limb visibility gate"),
    ("preview", "false", "annotated webcam window, MIRRORED for the human (X11, camera mode only)"),
    ("body_frame_id", "camera_link", "TF parent for the human/* frames"),
]


def generate_launch_description():
    pkg = get_package_share_directory("talos_mirror")
    moveit_pkg = get_package_share_directory("talos_moveit_config")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "mock_bringup.launch.py")
        ),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
    )

    ros_args = []
    for name, _, _ in NODE_ARGS:
        ros_args += ["-p", [name, ":=", LaunchConfiguration(name)]]

    # mediapipe lives in the ros2-talos:jazzy image's /opt/mpvenv, not the
    # system python ament's console_scripts are shebanged to. Set the prefix
    # unconditionally (synthetic mode does not need it, but the venv is
    # --system-site-packages so rclpy still imports fine either way, and one
    # interpreter for both modes means switching synthetic:=false never
    # silently changes which python is running the node).
    mpvenv = "/opt/mpvenv/bin/python"
    prefix = mpvenv if os.path.exists(mpvenv) else None

    track = Node(
        package="talos_mirror",
        executable="track_node",
        name="talos_track",
        output="screen",
        prefix=prefix,
        ros_arguments=ros_args,
    )

    # Where the webcam sits relative to the robot -- placement for RViz only.
    # human/* markers live in this frame; nothing computes a robot command
    # from it in this milestone, so this transform is not a calibration you
    # must get right before the demo works (same note as humanoid_mirror's
    # mirror.launch.py).
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_link_tf",
        arguments=["--x", "1.2", "--y", "0.0", "--z", "1.2",
                   "--yaw", "3.14159", "--pitch", "0.0", "--roll", "0.0",
                   "--frame-id", "base_link", "--child-frame-id", "camera_link"],
        output="log",
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
            *[
                DeclareLaunchArgument(name, default_value=default, description=desc)
                for name, default, desc in NODE_ARGS
            ],
            bringup,
            camera_tf,
            TimerAction(period=LaunchConfiguration("start_delay"), actions=[track]),
        ]
    )

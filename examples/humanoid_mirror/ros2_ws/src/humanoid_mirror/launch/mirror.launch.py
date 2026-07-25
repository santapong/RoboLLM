"""One-command bring-up: FFW humanoid + the mirror node.

    ros2-arm mirror synthetic                  scripted whole-body sweep, no camera
    ros2-arm mirror synthetic rviz:=false      headless (what mirror_accept uses)

Camera mode is M4. Until then mirror_node raises NotImplementedError for
synthetic:=false rather than pretending, so a missing webcam can never be
mistaken for a missing feature.

Every node parameter is also a launch argument, per house convention. They are
forwarded as `-p name:=value` through ros_arguments rather than as a parameters
dict, so ROS's YAML type inference turns "50.0" into a double and "true" into a
bool — passing LaunchConfiguration objects straight into `parameters=` would
hand the node strings and fail on the first float() call.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default, description) — forwarded to mirror_node as ROS parameters.
NODE_ARGS = [
    ("synthetic", "false", "scripted whole-body sweep instead of the camera (M2)"),
    ("rate_hz", "50.0", "control tick / command rate"),
    ("time_from_start", "0.06", "seconds-ahead stamp on each trajectory point"),
    ("max_joint_speed", "2.0", "rad/s slew ceiling (FFW's URDF limit is 4.8)"),
    ("limit_margin", "0.05", "stay this far off every joint limit"),
    ("min_cutoff", "1.0", "One-Euro min_cutoff for arm angles"),
    ("beta", "0.5", "One-Euro beta"),
    ("head_min_cutoff", "0.5", "slower One-Euro for the head (tiny travel)"),
    ("use_lift", "true", "also drive the prismatic lift"),
    ("loss_hold_sec", "2.0", "hold time after tracking loss before homing"),
    ("decay_speed", "0.5", "rad/s while decaying home"),
    ("sweep_period_s", "14.0", "synthetic sweep period"),
    ("latency_probe", "false", "log tick rate and clamp/slew hits every 3 s"),
    # --- M3 body tracking ---
    ("track_only", "false", "run vision + TF/markers with the robot PARKED (M3)"),
    ("camera", "/dev/video0", "video capture device"),
    ("pose_model_path", "/opt/models/pose_landmarker_full.task",
     "mediapipe pose_landmarker .task model file"),
    ("min_visibility", "0.5", "per-landmark visibility gate"),
    ("preview", "false", "annotated webcam window, MIRRORED for the human (X11)"),
    ("body_frame_id", "camera_link", "TF parent for the human/* frames"),
]


def generate_launch_description():
    pkg = get_package_share_directory("humanoid_mirror")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "mock_bringup.launch.py")
        ),
        launch_arguments={"rviz": LaunchConfiguration("rviz")}.items(),
    )

    ros_args = []
    for name, _, _ in NODE_ARGS:
        ros_args += ["-p", [name, ":=", LaunchConfiguration(name)]]

    # mediapipe lives in the Docker image's /opt/mpvenv, NOT in the system
    # python that ament console_scripts get shebanged to. Without this prefix
    # the node dies with ModuleNotFoundError: No module named 'mediapipe' the
    # moment tracking is switched on — while synthetic mode works fine, which
    # makes it look like a camera problem. Same fix hand_follow uses
    # (it runs its script through MPVENV_PYTHON explicitly).
    # The venv is --system-site-packages, so rclpy still imports from it and
    # this interpreter is correct for BOTH synthetic and camera modes.
    mpvenv = "/opt/mpvenv/bin/python"
    prefix = mpvenv if os.path.exists(mpvenv) else None

    mirror = Node(
        package="humanoid_mirror",
        executable="mirror_node",
        name="mirror_node",
        output="screen",
        prefix=prefix,
        ros_arguments=ros_args,
    )

    # Where the webcam sits relative to the robot. A placeholder: 1.2 m in
    # front of base_link at eye height, looking back at the robot. Only the
    # human/* markers live in this frame, and M4's retargeting is frame-
    # invariant (it copies ANGLES in the torso frame, not positions), so this
    # transform is for visual placement in RViz — not a calibration you must
    # get right before the demo works.
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
                "rviz", default_value="true", description="launch RViz2"
            ),
            # move_group + four controller spawners need to be up first. The
            # node also guards on "no /joint_states yet", so an early start is
            # harmless — just noisy.
            DeclareLaunchArgument(
                "start_delay",
                default_value="18.0",
                description="seconds to wait for move_group + controllers",
            ),
            *[
                DeclareLaunchArgument(name, default_value=default, description=desc)
                for name, default, desc in NODE_ARGS
            ],
            bringup,
            camera_tf,
            TimerAction(
                period=LaunchConfiguration("start_delay"), actions=[mirror]
            ),
        ]
    )

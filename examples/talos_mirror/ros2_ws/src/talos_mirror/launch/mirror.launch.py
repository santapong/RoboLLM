"""M4 bring-up: TALOS mock hardware + full-body tracking (talos_track) +
mirror_node, ALL IN ONE launch -- the `mirror [synthetic]` verb of
docker/ros2-arm.

    ros2 launch talos_mirror mirror.launch.py synthetic:=true use_rviz:=false
    ros2 launch talos_mirror mirror.launch.py synthetic:=false preview:=true

Three independent nodes, composed: mock_bringup (the robot), track_node
(vision -- unchanged from M3, still just publishes what it sees), and
mirror_node (subscribes /body/observation, commands the six controllers).
This is the decoupled architecture the task asked for ("mirror_node.py
(subscribes body obs...)") -- unlike humanoid_mirror, where tracking and
retargeting share one process, here they are three ROS nodes and the only
thing connecting the last two is the /body/observation topic track_node
already published for M3.

Every track_node/mirror_node parameter is also a launch argument, per house
convention, forwarded as `-p name:=value` through ros_arguments (not a
parameters dict) so ROS's YAML type inference turns "50.0" into a double and
"true" into a bool -- see track.launch.py / sweep.launch.py for the same
reasoning repeated at length.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default, description) -- forwarded to track_node.
TRACK_ARGS = [
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

# (name, default, description) -- forwarded to mirror_node.
MIRROR_ARGS = [
    ("mirror_rate_hz", "50.0", "control tick / command rate"),
    ("time_from_start", "0.06", "seconds-ahead stamp on each trajectory point"),
    ("max_joint_speed", "1.5", "rad/s slew ceiling"),
    ("limit_margin", "0.05", "stay this far off every joint limit"),
    ("min_cutoff", "1.0", "One-Euro min_cutoff for arm/leg angles"),
    ("beta", "0.5", "One-Euro beta"),
    ("head_min_cutoff", "0.5", "slower One-Euro for head+torso (small travel)"),
    ("loss_hold_sec", "2.0", "hold time after tracking loss before homing"),
    ("decay_speed", "0.4", "rad/s while decaying home"),
    ("mirror_mode", "mirror", '"mirror" (facing you) or "direct" (behind you)'),
    ("head_gain_yaw", "0.8", "human yaw -> robot head_2_joint gain"),
    ("head_gain_pitch", "0.6", "human pitch -> robot head_1_joint gain"),
    ("torso_gain_yaw", "0.5", "human torso yaw -> robot torso_1_joint gain"),
    ("torso_gain_pitch", "0.5", "human torso pitch -> robot torso_2_joint gain"),
    ("latency_probe", "false", "log tick rate and clamp/slew/limb summary every 3 s"),
    # c2-qp (M5) -- retarget mode and the QP layer's own tuning. Default is
    # "direct": the A/B judge REFUTED "qp tracks better" -- wrist position
    # regresses 0 -> 5.31 mm, the orientation metric has no independent
    # ground truth, and worst-case solve ticks (31-58 ms) can overrun this
    # node's own 50 Hz / 20 ms budget on this CPU. qp stays fully selectable
    # (mode:=qp) for anyone who wants to reproduce/re-measure that.
    ("mode", "direct",
     '"direct" (M4 only, default) or "qp" -- EXPERIMENTAL: refines wrist '
     "orientation via pink QP, slightly worse wrist position, may overrun "
     "50 Hz on this CPU"),
    ("landmark_min_cutoff", "1.0", "One-Euro min_cutoff on arm landmarks, BEFORE retargeting (qp mode only)"),
    ("landmark_beta", "0.5", "One-Euro beta on arm landmarks (qp mode only)"),
    ("qp_position_cost", "1.0", "pink FrameTask position weight (qp mode only)"),
    ("qp_orientation_cost", "0.5", "pink FrameTask orientation weight (qp mode only)"),
    ("qp_posture_cost", "0.01", "pink PostureTask weight, biased to the M4 solution (qp mode only)"),
    # Deadman (ported from humanoid_mirror): "enabled" sets the STARTING
    # state only -- true keeps auto-start for the mock demo; toggle live via
    # std_srvs/SetBool on /mirror_enable (false HOLDS every group where it is).
    ("enabled", "true", "deadman starting state; true auto-starts (mock demo default), "
     "toggle live via std_srvs/SetBool on /mirror_enable"),
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

    track_ros_args = []
    for name, _, _ in TRACK_ARGS:
        track_ros_args += ["-p", [name, ":=", LaunchConfiguration(name)]]

    # mediapipe lives in the ros2-talos:jazzy image's /opt/mpvenv, not the
    # system python ament's console_scripts are shebanged to. track_node
    # needs it (camera mode); mirror_node does not import mediapipe at all
    # (it only reads the already-parsed JSON /body/observation topic), but
    # running it under the same interpreter is harmless -- the venv is
    # --system-site-packages, so rclpy still imports fine either way, and
    # one interpreter for both nodes means there is only one python to
    # reason about in this launch.
    mpvenv = "/opt/mpvenv/bin/python"
    prefix = mpvenv if os.path.exists(mpvenv) else None

    track = Node(
        package="talos_mirror",
        executable="track_node",
        name="talos_track",
        output="screen",
        prefix=prefix,
        ros_arguments=track_ros_args,
    )

    mirror_ros_args = ["-p", ["rate_hz:=", LaunchConfiguration("mirror_rate_hz")]]
    for name, _, _ in MIRROR_ARGS[1:]:
        mirror_ros_args += ["-p", [name, ":=", LaunchConfiguration(name)]]

    mirror = Node(
        package="talos_mirror",
        executable="mirror_node",
        name="talos_mirror_node",
        output="screen",
        prefix=prefix,
        ros_arguments=mirror_ros_args,
    )

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
                description="seconds to wait for move_group + the six controllers "
                            "before track_node/mirror_node start",
            ),
            *[
                DeclareLaunchArgument(name, default_value=default, description=desc)
                for name, default, desc in TRACK_ARGS
            ],
            *[
                DeclareLaunchArgument(name, default_value=default, description=desc)
                for name, default, desc in MIRROR_ARGS
            ],
            bringup,
            camera_tf,
            TimerAction(period=LaunchConfiguration("start_delay"), actions=[track, mirror]),
        ]
    )

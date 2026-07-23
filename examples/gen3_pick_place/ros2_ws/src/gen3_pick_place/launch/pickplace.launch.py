"""One-command Gen3-lite hand-guided pick-and-place bring-up (U7).

Includes pickplace_demo.launch.py (mock-hardware ros2_control stack +
move_group [+ RViz on /monitored_planning_scene]) and, after the stack
settles, the hand_pick_place node — gesture-driven pick-and-place:
LEFT hand tracks, Closed_Fist = grip, Open_Palm = release.

  ros2-arm pickplace                     webcam + mediapipe gesture control
  ros2-arm pickplace synthetic           scripted pick-and-place (no camera)
  ros2-arm pickplace [synthetic] key:=value    any node parameter override
  ros2-arm pickplace rviz:=false         headless (no RViz window)

docs: pickplace-run.md (ros2_ws/docs/ in the dev ws, docs/ next to this example in RoboLLM) (bring-up, gesture cheat-sheet, tuning,
troubleshooting) and docs/pickplace-theory.md (why each mechanism exists).

The node runs under /opt/mpvenv/bin/python (the mediapipe venv baked into
ros2-arm:jazzy, --system-site-packages so rclpy resolves) straight from the
mounted source tree when available — edits take effect on the next launch
with no rebuild.  Native fallback: installed package + plain python3.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_SRC = "/ros2_ws/src/gen3_pick_place"
_MPVENV = "/opt/mpvenv/bin/python"
MP_PYTHON = _MPVENV if os.path.exists(_MPVENV) else "python3"


def generate_launch_description():
    # Prefer the live source tree (docker dev route: no rebuild needed for
    # node/launch edits); fall back to the installed share (native route).
    if os.path.isdir(_SRC):
        demo = os.path.join(_SRC, "launch", "pickplace_demo.launch.py")
        cwd = _SRC          # python -m resolves gen3_pick_place from cwd
    else:
        share = get_package_share_directory("gen3_pick_place")
        demo = os.path.join(share, "launch", "pickplace_demo.launch.py")
        cwd = None          # installed package resolves via PYTHONPATH

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo),
        launch_arguments={"rviz": LaunchConfiguration("rviz")}.items(),
    )

    # every hand_pick_place parameter is forwarded as a launch argument so
    # the documented `ros2-arm pickplace key:=value` tuning overrides work
    # (docs/pickplace-run.md "Tuning knobs").  Defaults mirror the node's.
    args = [
        ("synthetic", "false",
         "true: scripted full pick-and-place, no camera/mediapipe"),
        ("camera", "/dev/video0", "video capture device"),
        ("model_path", "/opt/models/gesture_recognizer.task",
         "mediapipe GestureRecognizer .task model file"),
        ("rate_hz", "20.0", "control tick / command rate"),
        ("time_from_start", "0.12", "seconds-ahead stamp on each traj point"),
        ("min_cutoff", "1.0", "One-Euro filter min cutoff (lower = smoother)"),
        ("beta", "0.5", "One-Euro filter beta (higher = less lag, more jitter)"),
        ("max_step_m", "0.03", "per-tick target slew clamp while tracking"),
        ("decay_step_m", "0.005", "per-tick slew while decaying to HOME"),
        ("loss_hold_sec", "2.0", "hold time after hand loss before homing"),
        ("max_joint_step_rad", "0.10", "per-tick joint-space delta clamp"),
        ("hand_conf", "0.6", "min handedness confidence for a Left hand"),
        ("acq_frames", "3", "consecutive frames before tracking (re)commits"),
        ("s_near", "0.30", "hand-size scale mapped to near end of X (depth)"),
        ("s_far", "0.10", "hand-size scale mapped to far end of X (depth)"),
        ("max_jump_rad", "0.5", "reject IK solutions jumping any joint more"),
        ("ik_fail_freeze", "5", "consecutive IK failures freezing the target"),
        ("latency_probe", "false",
         "true: log per-stage (capture/infer/IK/publish) timing every 3s"),
        ("preview", "false",
         "true: live webcam window with detections drawn (X11)"),
    ]
    declares = [DeclareLaunchArgument(n, default_value=d, description=desc)
                for n, d, desc in args]
    param_cmd = []
    for n, _, _ in args:
        param_cmd += ["-p", [n + ":=", LaunchConfiguration(n)]]

    # give ros2_control + move_group time to come up first — same 22 s margin
    # proven by handfollow.launch.py on this host; hand_pick_place's own
    # preflight() then waits (joint states, IK/FK services, box spawn)
    # before commanding anything.
    node = TimerAction(period=22.0, actions=[
        ExecuteProcess(
            cmd=[MP_PYTHON, "-m", "gen3_pick_place.hand_pick_place",
                 "--ros-args"] + param_cmd,
            cwd=cwd, output="screen"),
    ])

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Start RViz (RobotModel + PlanningScene on "
                    "/monitored_planning_scene)")
    return LaunchDescription([rviz_arg] + declares + [stack, node])

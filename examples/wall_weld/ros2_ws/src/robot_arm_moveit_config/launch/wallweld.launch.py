"""One-command wall-weld bring-up: the full MoveIt2 demo (RViz + mock
ros2_control) plus wall_weld.py — gesture-triggered autonomous serpentine
welding of a wall CollisionObject (U1).

  ros2-arm wallweld                     camera + GestureRecognizer: held
                                        CLOSED FIST welds the wall, OPEN
                                        PALM aborts; wall at the fixed
                                        default pose
  ros2-arm wallweld wall_mode:=marker   ArUco DICT_4X4_50 marker maps the
                                        wall pose (mapped placement — see
                                        wall_weld.py header)
  ros2-arm wallweld synthetic           no camera: scripted wall + auto
                                        fist through the identical path
  ros2-arm wallweld synthetic synthetic_abort_frac:=0.4
                                        + scripted mid-weld palm abort
  ros2-arm wallweld preview:=true       + annotated webcam window

wall_weld.py runs straight from its mounted src path under
/opt/mpvenv/bin/python (mediapipe venv baked into ros2-arm:jazzy), so edits
take effect on the next launch with no rebuild — same pattern as
handfollow.launch.py.
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_SRC_PY = "/ros2_ws/src/robot_arm_moveit_config/scripts/wall_weld.py"
WALL_WELD_PY = _SRC_PY if os.path.exists(_SRC_PY) else os.path.join(
    get_package_share_directory("robot_arm_moveit_config"),
    "scripts", "wall_weld.py")
_MPVENV = "/opt/mpvenv/bin/python"
MPVENV_PYTHON = _MPVENV if os.path.exists(_MPVENV) else "python3"

# (name, default, description) — every wall_weld.py parameter is forwarded
# as a launch argument so `ros2-arm wallweld key:=value` overrides work.
ARGS = [
    ("synthetic", "false", "true: scripted wall + auto-triggered weld, no camera"),
    ("preview", "false", "true: annotated webcam window (camera mode, X11)"),
    ("camera", "/dev/video0", "video capture device"),
    ("model_path", "/opt/models/gesture_recognizer.task",
     "mediapipe GestureRecognizer .task model"),
    ("rate_hz", "20.0", "control tick / streaming rate"),
    ("time_from_start", "0.12", "seconds-ahead stamp on each traj point"),
    ("wall_mode", "fixed", "'fixed' param pose | 'marker' ArUco-mapped pose"),
    ("wall_x", "0.30", "wall center x (fixed mode), base_link m"),
    ("wall_y", "0.0", "wall center y"),
    ("wall_z", "0.25", "wall center z"),
    ("wall_yaw", "0.0", "wall yaw rad (clamped to +-yaw_clamp_deg)"),
    ("wall_width", "0.35", "wall width m"),
    ("wall_height", "0.25", "wall height m"),
    ("wall_thick", "0.02", "wall thickness m"),
    ("row_step", "0.02", "vertical raster pass spacing m"),
    ("pass_step", "0.01", "waypoint spacing along a pass m"),
    ("margin", "0.02", "boundary margin from wall edges m"),
    ("standoff", "0.015", "torch tip standoff from the wall face m"),
    ("tool_back", "0.07", "tool0 offset from torch tip along wall normal m"),
    ("tool_up", "0.09", "tool0 offset from torch tip vertically m"),
    ("approach_dist", "0.06", "approach/retract extra clearance m"),
    ("bead_proud", "0.003", "bead drawn this far proud of the face m"),
    ("weld_speed", "0.05", "torch travel speed while welding m/s"),
    ("travel_speed", "0.08", "approach/retract travel speed m/s"),
    ("max_joint_step_rad", "0.10", "per-tick joint delta clamp"),
    ("ik_err_max_m", "0.004", "solve_track error that counts as an IK failure"),
    ("ik_fail_abort", "5", "consecutive IK failures that abort the weld"),
    ("home_x", "0.20", "idle hold tool0 x"),
    ("home_y", "0.0", "idle hold tool0 y"),
    ("home_z", "0.30", "idle hold tool0 z"),
    ("freeze_sec", "0.3", "freeze dwell after abort before homing"),
    ("settle_rad", "0.01", "joint-space settle threshold"),
    ("hand", "Any", "accepted handedness: Any | Left | Right"),
    ("hand_conf", "0.6", "min handedness confidence"),
    ("gesture_score", "0.6", "min gesture score for a committing frame"),
    ("n_fist", "5", "consecutive Closed_Fist frames to START"),
    ("n_palm", "3", "consecutive Open_Palm frames to ABORT"),
    ("hand_loss_abort_sec", "3.0", "hand loss this long mid-weld aborts"),
    ("marker_id", "-1", "required ArUco id (-1 = any 4x4_50 marker)"),
    ("wall_track", "false",
     "true: wall FOLLOWS the marker live while idle (lazy plan at fist)"),
    ("track_hz", "4.0", "max live re-pose rate while tracking"),
    ("track_alpha", "0.35", "EMA smoothing for the tracked pose (0-1)"),
    ("track_eps_m", "0.008", "min pose change to re-pose while tracking"),
    ("track_eps_yaw", "0.03", "min yaw change to re-pose while tracking"),
    ("aruco_every", "3", "run ArUco detection every Nth frame (idle only)"),
    ("capture_frames", "10", "consecutive stable detections to capture"),
    ("recapture_min_move_m", "0.02", "min pose change to re-capture"),
    ("recapture_min_yaw_rad", "0.09", "min yaw change to re-capture"),
    ("wall_x_min", "0.26", "marker-mapped wall x window min"),
    ("wall_x_max", "0.34", "marker-mapped wall x window max"),
    ("wall_y_min", "-0.10", "marker-mapped wall y window min"),
    ("wall_y_max", "0.10", "marker-mapped wall y window max"),
    ("wall_z_min", "0.20", "marker-mapped wall z window min"),
    ("wall_z_max", "0.32", "marker-mapped wall z window max"),
    ("yaw_clamp_deg", "20.0", "wall yaw clamp (both modes)"),
    ("marker_s_near", "0.30", "marker side/frame-height at nearest wall x"),
    ("marker_s_far", "0.08", "marker side/frame-height at farthest wall x"),
    ("grid_ny", "7", "reach precheck grid columns"),
    ("grid_nz", "5", "reach precheck grid rows"),
    ("ik_check_err_m", "0.003", "precheck max IK error per point"),
    ("shrink_factor", "0.85", "wall w/h shrink per failed precheck round"),
    ("shrink_max", "4", "max shrink rounds before rejecting the pose"),
    ("wall_min_w", "0.10", "smallest acceptable wall width"),
    ("wall_min_h", "0.06", "smallest acceptable wall height"),
    ("synth_start_sec", "2.0", "synthetic: inject fist this long after idle"),
    ("synthetic_abort_frac", "-1.0",
     "synthetic: inject palm at this weld progress (<0 = never)"),
]


def generate_launch_description():
    share = get_package_share_directory("robot_arm_moveit_config")
    # demo WITHOUT its MotionPlanning RViz — we bring our own config that
    # shows the wall (PlanningScene) AND the weld bead/sparks (MarkerArray
    # on /wall_weld_markers), which the stock moveit.rviz does not display.
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "demo.launch.py")),
        launch_arguments={"use_rviz": "false"}.items())
    rviz_src = "/ros2_ws/src/robot_arm_moveit_config/rviz/wallweld.rviz"
    rviz_cfg = rviz_src if os.path.exists(rviz_src) else os.path.join(
        share, "rviz", "wallweld.rviz")
    rviz_node = Node(package="rviz2", executable="rviz2",
                     arguments=["-d", rviz_cfg],
                     condition=IfCondition(LaunchConfiguration("rviz")))
    declares = [DeclareLaunchArgument(
                    "rviz", default_value="true",
                    description="launch RViz with the wall-weld view")]
    declares += [DeclareLaunchArgument(n, default_value=d, description=desc)
                 for n, d, desc in ARGS]
    param_cmd = []
    for n, _, _ in ARGS:
        param_cmd += ["-p", [n + ":=", LaunchConfiguration(n)]]
    # same 22 s bring-up margin proven by loop/handfollow launches on this
    # host; wall_weld.py's own preflight then waits for /joint_states +
    # /apply_planning_scene before doing anything.
    weld = TimerAction(period=22.0, actions=[
        ExecuteProcess(
            cmd=[MPVENV_PYTHON, WALL_WELD_PY, "--ros-args"] + param_cmd,
            output="screen"),
    ])
    return LaunchDescription(declares + [demo, rviz_node, weld])

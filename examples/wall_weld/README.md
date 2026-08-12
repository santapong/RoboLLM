# RoboLLM · Gesture-triggered wall-weld simulation

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Examples](../README.md) · [Technical notes](TECHNICAL.md) · [Runbook](docs/wallweld-run.md)

**Environment:** RViz simulation · CPU-only · no welding hardware. This is a
state-machine and planning lesson, not a real welding controller.

Show the webcam an ArUco marker to place (or **live-move**) a wall in the
MoveIt planning scene, then **make a fist**: the 6-DOF weld arm autonomously
rasters the entire wall face — serpentine passes, a growing bead line,
sparks at the torch. **Open palm aborts** mid-weld. This is gesture-triggered
*automation*, not hand-following: while idle the arm holds home.

```
/dev/video0 ─▶ MediaPipe GestureRecognizer (fist/palm, 21 landmarks, ~27 ms CPU)
            │            └▶ cv2.aruco DICT_4X4_50 ─▶ mapped wall pose
            │               (one-shot capture, or wall_track:=true = the wall
            │                FOLLOWS the marker live, EMA-smoothed, ≤4 Hz)
            ├ fist ×5 ─▶ raster planner (serpentine, 15 mm standoff, corner+grid
            │            reachability precheck, shrink-to-fit) ─▶ 20 Hz
            │            warm-start IK streaming ─▶ JointTrajectory ─▶ arm
            └ palm ×3 ─▶ abort: freeze → slewed return home, bead stays
```

| Gesture / input | Robot |
|---|---|
| ArUco marker (phone screen works) | wall placed where the marker maps; `wall_track:=true` = follows it in realtime |
| **fist, held ~5 frames** | plans + welds the whole wall (~72 s default size), bead + sparks visible |
| **open palm** (or hide hand >3 s mid-weld) | abort — partial bead persists |
| fist again | fresh weld |
| `/wall_reset` (Trigger srv) | respawn wall, clear bead |

## Run — Docker (the verified route)

```bash
cd examples/wall_weld
docker build -t ros2-arm:jazzy docker/     # skip if already built
./docker/ros2-arm wallweld synthetic       # scripted wall + auto weld, no camera
./docker/ros2-arm wallweld preview:=true   # live: fist to weld, palm to abort
./docker/ros2-arm wallweld wall_mode:=marker wall_track:=true preview:=true
                                           # realtime marker-tracked wall
```

Marker: display `ros2_ws/src/robot_arm_moveit_config/assets/wall_marker.png`
(DICT_4X4_50, id 0) on your phone. Docs: `docs/wallweld-run.md`.

## Verified numbers (i3-9100, CPU-only)

Synthetic full weld: **352/352 waypoints, bead 352/352, 20.00 Hz stream,
max joint step 0.025–0.04 rad** (limit 0.15), **101/101 sampled states
collision-valid (100%)** vs the wall at the default 15 mm standoff (5 mm
scored 88% — the torch collision body is thicker than its tip). Raster:
11 rows, 3.61 m path. Abort stops motion promptly; mid-weld `/wall_reset`
correctly refused. Adversarially reviewed: TOCTOU idle-gate race and a
degenerate-raster crash found and fixed before release.

## Honesty note on "the camera sees the wall"

Marker placement is a **mapped** pose (marker image position/size/tilt →
a clamped reachable window), not metric SLAM — a single webcam cannot
measure absolute distance to a featureless plane. For true wall surveying,
a depth camera (RealSense-class) is the upgrade path; the tracking
architecture takes a depth-derived plane without redesign.

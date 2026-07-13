# Live session playbook — the full LLM ↔ robot demo

A guided ~30-minute session on your desktop that exercises everything: Claude
sees through the camera, controls the world, drives the robot, builds a map,
navigates it, and moves an arm. Do the steps in order; each stage builds on the
previous one.

> Camera matters: launch with **waffle_pi** (the burger has no camera).

## Setup (2 terminals + Claude Code)
```bash
# Terminal 1 — simulator (keep it visible, you'll watch the robot)
TURTLEBOT3_MODEL=waffle_pi ~/Desktop/robot-llm-loop/sim/launch_turtlebot.sh

# Terminal 2 — browser dashboard (optional but fun: you watch telemetry live)
~/Desktop/robot-llm-loop/web/run-web.sh     # → http://localhost:8080
```
Then **restart Claude Code** in `~/Desktop/robot-llm-loop` so the `ros2` MCP
(22 tools) is fresh.

## Stage 1 — Claude wakes up (introspection)
Ask Claude:
- *"List the ROS 2 topics and tell me what robot this looks like."*
- *"Where is the robot right now, and what do the laser sectors say?"*

Expect: topic list with `/camera/image_raw` present, pose ≈ (−2.0, −0.5) in the
TB3 world, laser distances in all four directions.

## Stage 2 — Claude sees (vision)
- *"Take a camera picture and describe what you see."*

Expect: Claude calls `get_camera_image`, reads the JPEG, describes the TB3
world's pillars. **This is the moment the loop closes through pixels.**

## Stage 3 — Claude controls the world (new!)
- *"Spawn a red box half a meter in front of the robot, then take another
  picture — can you see it?"*
- *"Now spawn a blue sphere to its left and list all world objects."*

Expect: `spawn_object` + a camera shot with the red box visible in it.

## Stage 4 — Claude acts (the vision→action loop)
- *"Drive toward the red box and stop before you hit it."*
  (Claude should combine camera + `get_laser_scan` + `drive` — safe mode will
  also refuse forward motion closer than 0.35 m.)
- Or run the autonomous chaser while YOU move the box around in Gazebo:
  ```bash
  .venv/bin/python examples/ros2_py/10_color_follow.py
  ```

## Stage 5 — record the run (datasets)
- *"Record odometry, laser and camera while I drive for 20 seconds, then stop
  and tell me the bag size."* → then drive via the dashboard.
- *"Replay that bag."*

## Stage 6 — map & navigate (add Terminal 3)
```bash
~/Desktop/robot-llm-loop/sim/launch_slam.sh          # Terminal 3
```
- *"Drive a slow exploration pattern and tell me the map status every so often.
  Stop when it's mostly explored."* (Claude: `drive` + `get_map_status`)
- Save the map, then swap SLAM for Nav2:
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/map    # then Ctrl-C SLAM
  ~/Desktop/robot-llm-loop/sim/launch_nav2.sh        # set 2D Pose Estimate in RViz!
  ```
- *"Navigate to x=0.5, y=0.5 and confirm with the transform where you ended up."*
  (Claude: `navigate_to` + `get_transform("map", "base_link")`)

## Stage 7 — the arm (separate demo, no Gazebo needed)
```bash
~/Desktop/robot-llm-loop/sim/launch_moveit_panda.sh   # can replace Stages 1–6
```
- *"Read the arm's joint states, then move it to a ready pose."*
  (Claude: `get_joint_states` + `move_arm('panda_joint1=0, panda_joint2=-0.785,
  panda_joint3=0, panda_joint4=-2.356, panda_joint5=0, panda_joint6=1.571,
  panda_joint7=0.785')`)

## If something misbehaves
| Symptom | Fix |
|---------|-----|
| Claude's tools return "no /odom yet" | sim not up yet, or Claude Code started before the sim — restart Claude Code |
| `get_camera_image` errors | you launched burger — relaunch with `TURTLEBOT3_MODEL=waffle_pi` |
| `navigate_to` rejected | Nav2 needs the 2D Pose Estimate set in RViz first |
| spawn works but you can't see the object | it spawned behind a pillar — ask Claude to spawn at different x/y |
| robot won't drive forward | safe mode: something is < 0.35 m ahead (ask Claude to `set_safe_mode` off, or back up) |

Afterwards, tell Claude what worked and what felt clumsy — that feedback drives
the next round of tools.

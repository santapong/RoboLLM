# robot-llm-loop

Your first **LLM ↔ robotics loop**: Claude (via MCP) can introspect and drive a
ROS 2 robot, and separately drive FreeCAD to design parts / export URDF.

```
        you (natural language)
                │
                ▼
      ┌───────────────────┐      ros2 MCP        ┌──────────────────────┐
      │   Claude Code      │ ─────────────────▶  │  ros2_mcp_server.py  │
      │  (planner + coder) │                     │  (rclpy node)        │
      └───────────────────┘  ◀─────────────────  └──────────┬───────────┘
                │              tool results                  │ /cmd_vel, /odom,
                │  freecad MCP                               │ navigate_to_pose
                ▼                                            ▼
      ┌───────────────────┐                     ┌──────────────────────┐
      │  FreeCAD (CAD/URDF)│                     │ TurtleBot3 in Gazebo │
      └───────────────────┘                     └──────────────────────┘
```

## What's here
| Path | Purpose |
|------|---------|
| `ros2_mcp_server.py` | The bridge. 8 MCP tools — see below. **Edit this to add skills.** |
| `run-server.sh` | Launch wrapper — sources ROS 2 + venv, runs the server. Registered as the `ros2` MCP server. |
| `sim/launch_turtlebot.sh` | Starts TurtleBot3 in Gazebo (own terminal). |
| `assets/` | Outputs live here: `urdf/`, `cad/` (FreeCAD exports), `screenshots/`. |
| `setup/dev-setup.sh` | Full machine provisioner (copy of the one that built this box). |
| `docs/` | Notes as you learn. |
| `.venv/` | venv with `--system-site-packages` (sees ROS `rclpy`) + MCP SDK. Git-ignored. |

### MCP tools (in `ros2_mcp_server.py`)
| Tool | What it does |
|------|--------------|
| `list_topics` / `list_nodes` | Introspect the ROS graph |
| `get_robot_pose` | Robot x/y/orientation from `/odom` |
| `get_laser_scan` | Nearest obstacle front/left/right/back from `/scan` |
| `drive` / `stop` | Move via `/cmd_vel` (auto-stops, 10 s safety cap) |
| `navigate_to` | Nav2 goal to (x, y, yaw) |
| `run_ros2` | Run any `ros2` CLI command — the fast learn/introspect tool |

## Try the loop
1. **Start the sim** (own terminal, needs a display):
   ```bash
   ~/Desktop/robot-llm-loop/sim/launch_turtlebot.sh
   ```
2. **Restart Claude Code** (so it picks up the `ros2` MCP server), then just ask:
   - "List the ROS 2 topics."
   - "Where is the robot?"
   - "Drive forward for 3 seconds, then turn left."
   - (with Nav2 running) "Navigate to x=1.5, y=0.5."

Claude calls the MCP tools and the TurtleBot moves in Gazebo.

## CAD / URDF loop
The `freecad` MCP is also registered. Open FreeCAD → **MCP Addon** workbench →
**Start RPC Server**, then ask Claude things like:
- "In FreeCAD, create a 2-link robot arm and export it as URDF."
- "Make a 100×60×20 mm mounting bracket with four M4 holes."

## Extending (the learning part)
Add a new capability by adding a function to `ros2_mcp_server.py`:
```python
@mcp.tool()
def get_battery() -> float:
    """Read the latest battery percentage."""
    ...
```
Restart Claude Code and the new tool is available. Grow this file as you learn:
laser scan reads, camera snapshots, MoveIt arm goals, behavior sequencing, etc.

## MCP registration (already done)
```
claude mcp add --scope user ros2 -- ~/Desktop/robot-llm-loop/run-server.sh
```

## Version control & push to GitHub
This folder is a git repo. To push it:
```bash
gh auth login                 # one-time: authenticate GitHub (interactive)
gh repo create robot-llm-loop --private --source=. --remote=origin --push
```
After that, normal flow (or ask Claude to do it):
```bash
git add -A && git commit -m "..." && git push
```
`.venv/` and large binary assets are git-ignored; source, scripts, and small
assets are tracked.

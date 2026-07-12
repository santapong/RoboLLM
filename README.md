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
| File | Purpose |
|------|---------|
| `ros2_mcp_server.py` | The bridge. Exposes ROS 2 as MCP tools (`list_topics`, `list_nodes`, `get_robot_pose`, `drive`, `stop`, `navigate_to`). **Edit this to add skills.** |
| `run-server.sh` | Launch wrapper — sources ROS 2 + venv, runs the server. Registered with Claude Code as the `ros2` MCP server. |
| `sim/launch_turtlebot.sh` | Starts TurtleBot3 in Gazebo (run in its own terminal). |
| `.venv/` | Python venv with `--system-site-packages` (sees ROS `rclpy`) + the MCP SDK. |

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

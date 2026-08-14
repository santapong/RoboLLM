# RoboLLM · Package the ROS 2 MCP server

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Documentation](../docs/README.md) · [Architecture](../docs/ARCHITECTURE.md) · [Live session](../docs/live-session.md)

Two install formats, because Claude Desktop and Claude Code use different ones.

| Client | Format | This repo provides |
|--------|--------|--------------------|
| **Claude Code** (CLI / IDE) | `.mcp.json` or `claude mcp add` | `../.mcp.json` |
| **Claude Desktop** (Mac/Win app) | `.mcpb` bundle (drag-and-drop) | `dist/ros2-bridge.mcpb` (built here) |

> `.mcpb` bundles are a **Claude Desktop** feature — Claude Code does **not**
> install `.mcpb`. For Claude Code use the `.mcp.json` below.

Both run the same server; `rclpy` and the message packages come from your ROS 2
install (its `setup.bash` is sourced at launch), so a ROS 2 installation is
required on any machine that runs this.

## Claude Code (what you use)
You already have it registered at **user scope** (works everywhere):
```bash
claude mcp add --scope user ros2 -- /path/to/RoboLLM/scripts/launch/mcp.sh
```
The repo also ships `../.mcp.json` for **project scope** — anyone who opens this
folder in Claude Code gets the `ros2` server after a one-time approval prompt. It
points at `scripts/launch/mcp.sh` (which sources ROS 2 + the venv). Nothing else to do.

## Claude Desktop (the .mcpb you asked for)
Build the bundle:
```bash
mcpb/build_mcpb.sh          # -> mcpb/dist/ros2-bridge.mcpb  (~9 MB)
```
Then in Claude Desktop: **Settings → Extensions → Advanced → Install from file**
and pick `mcpb/dist/ros2-bridge.mcpb`. At install it asks for your **ROS 2
setup.bash** path (default `/opt/ros/jazzy/setup.bash`) and TurtleBot3 model.

What the bundle contains:
```
ros2-bridge.mcpb (zip)
├── manifest.json          # v0.4; command sources ROS then runs the server
└── server/
    ├── apps/mcp/entrypoint.py  apps/mcp/server.py
    ├── robollm/bridge.py  robollm/gazebo_world.py
    └── lib/                # vendored `mcp` SDK (rclpy comes from ROS at runtime)
```

The built `.mcpb` and `build/` staging are git-ignored (regenerable, ~9 MB). To
share the bundle, hand someone the file from `dist/` (or force-add it to git).

## Notes / limits
- **Linux only** as configured — the launch command uses `bash` + `source`. A
  Windows Desktop install would need a platform override in `manifest.json`.
- The vendored `mcp` wheel is built for this machine's Python (3.12, x86_64). If
  you move the bundle to a very different Python/arch, rebuild it there.
- Editing tools? Edit `../apps/mcp/server.py`, then rerun `build_mcpb.sh` (it
  copies the current server code in) and reinstall.

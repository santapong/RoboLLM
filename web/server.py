#!/usr/bin/env python3
"""
web/server.py — an easy browser dashboard for the ROS 2 robot.

Run it, open http://localhost:8080, and you get:
  - live pose + velocity
  - a mini lidar "radar" view
  - on-screen + WASD keyboard teleop (hold to move, release to stop)
  - the live topic list
  - one-click Nav2 goals

It reuses robot_bridge.py (the same code the MCP server uses), so the browser
and Claude drive the exact same robot. Uses a different ROS node name so both
can run at once.

Launch: web/run-web.sh   (sources ROS 2 + venv, then uvicorn)
"""
from __future__ import annotations

import asyncio
import os

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from robot_bridge import get_bridge

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")

app = FastAPI(title="robot-llm-loop dashboard")
bridge = get_bridge("claude_web_bridge")


class Vel(BaseModel):
    linear: float = 0.0
    angular: float = 0.0


class Goal(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0


@app.get("/")
def index() -> FileResponse:
    return FileResponse(os.path.join(STATIC, "index.html"))


@app.get("/api/topics")
def topics() -> list[dict[str, str]]:
    return [{"topic": n, "type": ",".join(t)}
            for n, t in bridge.get_topic_names_and_types()]


@app.post("/api/cmd")
def cmd(v: Vel) -> dict[str, str]:
    """Continuous teleop: set target velocity. The bridge's deadman watchdog
    auto-stops if the browser stops sending (~0.6 s)."""
    bridge.set_velocity(v.linear, v.angular)
    return {"ok": "set"}


@app.post("/api/stop")
def stop() -> dict[str, str]:
    bridge.stop()
    return {"ok": "stopped"}


@app.post("/api/nav")
def nav(g: Goal) -> dict[str, str]:
    return {"result": bridge.navigate_to(g.x, g.y, g.yaw_deg, timeout_s=1.0) or "sent"}


@app.websocket("/ws")
async def telemetry(ws: WebSocket) -> None:
    """Push pose + radar ~10x/s so the dashboard stays live."""
    await ws.accept()
    try:
        while True:
            await ws.send_json({"pose": bridge.pose(),
                                "laser": bridge.laser_summary(),
                                "radar": bridge.laser_radar(36)})
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        return
    except Exception:
        return


if os.path.isdir(STATIC):
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

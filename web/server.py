#!/usr/bin/env python3
"""web/server.py — an easy browser dashboard for the ROS 2 robot.

Run it, open http://localhost:8080, and you get live telemetry, a lidar radar,
the camera view, WASD/on-screen teleop (obstacle-safe), the topic list, and
one-click Nav2 goals. It reuses robot_bridge.py — the same code the MCP server
uses — so you and Claude drive the exact same robot.

Safety:
  - binds 127.0.0.1 by default (localhost only). To expose on your LAN, set
    HOST=0.0.0.0 AND a shared token: ROBOT_TOKEN=secret  (control endpoints then
    require ?token=secret; the page embeds it automatically when you load it
    from this server).

Launch: web/run-web.sh
"""
from __future__ import annotations

import asyncio
import os
import threading
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Query
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from robot_bridge import get_bridge

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")
TOKEN = os.environ.get("ROBOT_TOKEN", "")   # empty = no auth (fine for localhost)

app = FastAPI(title="robot-llm-loop dashboard")
bridge = get_bridge("claude_web_bridge")

# Camera: prefer the ROS /camera/image_raw feed (Gazebo sim or the Pi's
# run_camera.sh); fall back to a directly-attached USB webcam so the dashboard
# shows something live even with no ROS camera. WEBCAM=0 disables the fallback.
WEBCAM_ENABLED = os.environ.get("WEBCAM", "1") != "0"
WEBCAM_DEVICE = os.environ.get("WEBCAM_DEVICE", "/dev/video0")


class Webcam:
    """Lazy, thread-safe USB webcam grabber. Opens the device only on the first
    frame request and degrades to None (never raises) if it can't."""

    def __init__(self, device: str) -> None:
        self.device = device
        self._cap = None
        self._lock = threading.Lock()
        self.failed = False

    def _index(self):
        d = self.device
        if d.startswith("/dev/video"):
            try:
                return int(d.replace("/dev/video", ""))
            except ValueError:
                return d
        return int(d) if d.isdigit() else d

    def jpeg(self):
        if self.failed:
            return None
        with self._lock:
            import cv2
            if self._cap is None:
                cap = cv2.VideoCapture(self._index(), cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap.release()
                    self.failed = True
                    return None
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self._cap = cap
            ok, frame = self._cap.read()
            if not ok:
                return None
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return buf.tobytes() if ok else None


_webcam = Webcam(WEBCAM_DEVICE) if WEBCAM_ENABLED else None


def _camera_frame():
    """Latest frame as (jpeg_bytes, source). ROS camera wins; USB webcam is the
    fallback; (None, 'none') if neither has a frame."""
    jpg = bridge.camera_jpeg_bytes()
    if jpg is not None:
        return jpg, "ros"
    if _webcam is not None:
        jpg = _webcam.jpeg()
        if jpg is not None:
            return jpg, "webcam"
    return None, "none"


def _camera_source() -> str:
    """Cheap source label (no frame grab) for telemetry."""
    if bridge.camera_age_s() is not None:
        return "ros"
    return "webcam" if _webcam is not None and not _webcam.failed else "none"


def check(token: str) -> None:
    if TOKEN and token != TOKEN:
        raise HTTPException(status_code=401, detail="bad or missing token")


class Vel(BaseModel):
    linear: float = 0.0
    angular: float = 0.0


class Goal(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0


class Safe(BaseModel):
    enabled: bool = True
    min_obstacle_m: float = 0.35


class ArmCmd(BaseModel):
    angles: list[float]


class ArmEnable(BaseModel):
    on: bool = True


@app.get("/", response_class=HTMLResponse)
def index() -> HTMLResponse:
    # inject the token so the page's fetches authenticate automatically
    with open(os.path.join(STATIC, "index.html")) as f:
        html = f.read().replace("__TOKEN__", TOKEN)
    return HTMLResponse(html)


@app.get("/api/topics")
def topics() -> list[dict[str, Any]]:
    """Every topic with its type and live publisher/subscriber counts."""
    return bridge.topics_detail()


@app.post("/api/cmd")
def cmd(v: Vel, token: str = Query("")) -> dict[str, str]:
    check(token)
    bridge.set_velocity(v.linear, v.angular)
    return {"ok": "set"}


@app.post("/api/stop")
def stop(token: str = Query("")) -> dict[str, str]:
    check(token)
    bridge.stop()
    return {"ok": "stopped"}


@app.post("/api/nav")
def nav(g: Goal, token: str = Query("")) -> dict[str, str]:
    check(token)
    return {"result": bridge.navigate_to(g.x, g.y, g.yaw_deg, timeout_s=1.0) or "sent"}


@app.post("/api/safe")
def safe(s: Safe, token: str = Query("")) -> dict[str, str]:
    check(token)
    bridge.safe_mode = s.enabled
    bridge.min_obstacle_m = s.min_obstacle_m
    return {"safe_mode": str(bridge.safe_mode), "min_obstacle_m": str(bridge.min_obstacle_m)}


@app.post("/api/arm")
def arm(cmd: ArmCmd, token: str = Query("")) -> dict[str, Any]:
    """Send target angles (degrees) to the real DIY arm."""
    check(token)
    return bridge.command_arm(cmd.angles)


@app.post("/api/arm/home")
def arm_home(token: str = Query("")) -> dict[str, Any]:
    check(token)
    return bridge.arm_home()


@app.post("/api/arm/enable")
def arm_enable(e: ArmEnable, token: str = Query("")) -> dict[str, Any]:
    """Torque the arm on/off — off is the arm's e-stop (servos go limp)."""
    check(token)
    return bridge.arm_enable(e.on)


@app.get("/api/joints")
def joints() -> dict[str, Any]:
    return bridge.arm_joint_states()


@app.get("/api/arm/status")
def arm_status() -> dict[str, Any]:
    """Arm connection detail: serial port, firmware, baud, torque state."""
    return bridge.arm_status()


@app.get("/api/camera")
def camera() -> Response:
    """Latest camera frame as JPEG (204 if no camera / no image yet)."""
    jpg, source = _camera_frame()
    if jpg is None:
        return Response(status_code=204)
    return Response(content=jpg, media_type="image/jpeg",
                    headers={"X-Camera-Source": source})


@app.get("/api/camera/stream")
async def camera_stream() -> StreamingResponse:
    """Live MJPEG stream (multipart/x-mixed-replace) — a smooth feed instead of
    per-frame polling. Sourced from the ROS camera or the USB webcam fallback."""
    async def frames():
        loop = asyncio.get_event_loop()
        while True:
            jpg, _ = await loop.run_in_executor(None, _camera_frame)
            if jpg:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            await asyncio.sleep(0.066)   # ~15 fps
    return StreamingResponse(frames(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.websocket("/ws")
async def telemetry(ws: WebSocket) -> None:
    """Push pose + laser + radar ~10x/s so the dashboard stays live."""
    await ws.accept()
    try:
        while True:
            await ws.send_json({"pose": bridge.pose(),
                                "laser": bridge.laser_summary(),
                                "radar": bridge.laser_radar(36),
                                "arm": bridge.arm_joint_states(),
                                "arm_status": bridge.arm_status(),
                                "cam": {"source": _camera_source(), "age": bridge.camera_age_s()},
                                "safe": {"on": bridge.safe_mode, "min": bridge.min_obstacle_m}})
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        return
    except Exception:
        return


if os.path.isdir(STATIC):
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

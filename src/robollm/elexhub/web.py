"""Loopback-only experiment console; no robot is enabled on startup."""

from __future__ import annotations

import asyncio
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import asynccontextmanager
from pathlib import Path

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, HTMLResponse, Response
from pydantic import BaseModel, ConfigDict
from robo_arm_driver.elexhub_profile import hardware_gaps
from starlette.middleware.trustedhost import TrustedHostMiddleware

from .camera_setup import CameraSetup, camera_router
from .interaction import TaskExecutor, parse_instruction, select_pointed_object
from .model import mat_to_base


class PreviewRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    instruction: str
    destination: list[float]
    object_id: str | None = "red_box_0"


class ExecuteRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    preview_id: str
    workspace_clear: bool = False
    condition: str = "C"


class SelectionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    role: str
    pixel: list[float] | None = None


def create_app(arm, root, *, mode="sim", webcam=None, speech=None, camera_setup=None):
    root = Path(root)
    setup = camera_setup or CameraSetup(root)
    setup.enabled = setup.enabled and mode == "sim" and webcam is None
    sessions = root / "output/elexhub-sessions"
    sessions.mkdir(parents=True, exist_ok=True)
    logfile = sessions / f"session-{time.time_ns()}.jsonl"
    write_lock = threading.Lock()

    def record(entry):
        with write_lock, logfile.open("a") as stream:
            stream.write(json.dumps(entry, allow_nan=False) + "\n")

    task = TaskExecutor(
        arm, mode=mode, observe=webcam.observe if webcam else None, record=record
    )
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="elexhub-task")
    stopped = threading.Event()
    state = {
        "jpeg": None,
        "jpeg_at": 0.0,
        "future": None,
        "result": None,
        "error": None,
        "heartbeat": time.monotonic(),
        "selection": {},
    }
    api_lock = threading.RLock()

    def rendering():
        try:
            while not stopped.is_set():
                image = arm.render()
                ok, jpeg = cv2.imencode(
                    ".jpg",
                    cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 85],
                )
                if ok:
                    state.update(jpeg=jpeg.tobytes(), jpeg_at=time.monotonic())
                else:
                    state["jpeg"] = None
                if (
                    mode == "hardware"
                    and getattr(arm, "enabled", False)
                    and time.monotonic() - state["heartbeat"] > 0.6
                ):
                    task.abort()
                    state["error"] = "console connection lost; PWM disabled"
                stopped.wait(0.08)
        finally:
            state["jpeg"] = None
            arm.close()

    thread = threading.Thread(target=rendering, daemon=True)

    @asynccontextmanager
    async def lifespan(app):
        thread.start()
        yield
        task.abort()
        stopped.set()
        pool.shutdown(wait=True, cancel_futures=True)
        thread.join(timeout=3)
        if webcam:
            webcam.close()
        setup.close()

    app = FastAPI(title="Elexhub experiment console", lifespan=lifespan)
    app.add_middleware(
        TrustedHostMiddleware, allowed_hosts=["127.0.0.1", "localhost", "testserver"]
    )
    app.state.task = task
    app.state.runtime = state
    app.state.camera_setup = setup
    app.include_router(camera_router(setup))

    @app.middleware("http")
    async def local_mutations(request: Request, call_next):
        # Browsers from other origins must never submit motion commands.
        if request.method == "POST":
            origin = request.headers.get("origin")
            expected = str(request.base_url).rstrip("/")
            if origin and origin != expected:
                return Response("cross-origin command refused", status_code=403)
        response = await call_next(request)
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; frame-ancestors 'none'"
        )
        return response

    @app.exception_handler(ValueError)
    async def rejected(request, exc):
        return Response(
            json.dumps({"detail": str(exc)}),
            status_code=400,
            media_type="application/json",
        )

    @app.get("/")
    def home():
        return HTMLResponse((Path(__file__).parent / "console.html").read_text())

    @app.get("/model")
    def model_view():
        return FileResponse(
            root / "assets/elexhub/provisional/model.html", media_type="text/html"
        )

    @app.get("/api/state")
    def status():
        obs = webcam.public_observation() if webcam else None
        return {
            "mode": mode,
            "stage": task.stage,
            "busy": task.busy,
            "result": state["result"],
            "error": state["error"],
            "arm": arm.observation(),
            "camera": obs,
            "selection": state["selection"],
            "speech_ready": speech is not None,
            "hardware_enabled": bool(getattr(arm, "enabled", False)),
            "hardware_gaps": hardware_gaps(arm.config.profile_data),
            "events": task.history[-8:],
        }

    @app.get("/api/frame.jpg")
    def frame(camera: bool = False):
        if camera:
            jpg = webcam.frame() if webcam else None
        else:
            jpg = state["jpeg"] if time.monotonic() - state["jpeg_at"] <= 1.0 else None
        if jpg is None:
            # An absent feed is an ordinary empty state, never another source.
            return Response(status_code=204)
        return Response(jpg, media_type="image/jpeg")

    @app.post("/api/heartbeat")
    def heartbeat():
        state["heartbeat"] = time.monotonic()
        if mode == "hardware":
            arm.keepalive()
        return {"ok": True}

    @app.post("/api/selection")
    def selection(body: SelectionRequest):
        if not webcam:
            raise ValueError(
                "use the grid in simulation; webcam selection requires calibrated capture"
            )
        if body.role not in {"source", "destination"}:
            raise ValueError("unknown selection role")
        obs = webcam.observe()
        xy = (
            webcam.pixel(body.pixel)
            if body.pixel is not None
            else obs.get("pointing_mat_m")
        )
        if xy is None or time.monotonic() - obs.get("captured_at_monotonic", 0) > 0.30:
            raise ValueError(
                "one visible pointing hand and a fresh mat reference are required"
            )
        value = (
            select_pointed_object(obs.get("objects", []), xy)
            if body.role == "source"
            else xy
        )
        state["selection"][body.role] = value
        state["selection"]["captured_at"] = time.monotonic()
        return {"role": body.role, "value": value}

    @app.post("/api/preview")
    def preview(body: PreviewRequest):
        with api_lock:
            if state["future"] and not state["future"].done():
                raise ValueError("task is busy")
            source = None
            calibration_id = "simulation"
            if webcam:
                obs = webcam.observe()
                calibration_id = webcam.camera.id
                matching = [
                    o for o in obs.get("objects", []) if o["id"] == body.object_id
                ]
                if body.instruction.lower().startswith(("move", "put")):
                    if len(matching) != 1:
                        raise ValueError("selected object is missing or ambiguous")
                    # Hardware uses the measured base transform; shadow uses the preview profile.
                    if mode == "hardware":
                        source = (
                            webcam.camera.base @ np.r_[matching[0]["position_mat_m"], 1]
                        )[:3]
                    else:
                        source = mat_to_base(
                            arm.config,
                            matching[0]["position_mat_m"][:2],
                            matching[0]["position_mat_m"][2],
                        )
            intent = parse_instruction(
                body.instruction,
                body.destination,
                object_id=body.object_id,
                calibration_id=calibration_id,
            )
            state["error"] = state["result"] = None
            return task.preview(intent, source_base=source)

    @app.post("/api/execute")
    def execute(body: ExecuteRequest):
        with api_lock:
            if state["future"] and not state["future"].done():
                raise ValueError("task is busy")
            if not body.workspace_clear:
                raise ValueError("confirm the workspace is clear")
            if not task.previewed or task.previewed[0] != body.preview_id:
                raise ValueError("create a fresh preview first")
            if mode == "shadow":
                raise ValueError(
                    "shadow mode records perception and previews; motion is disabled"
                )
            state["result"] = state["error"] = None

            def work():
                try:
                    state["result"] = task.execute(
                        body.preview_id,
                        workspace_clear=body.workspace_clear,
                        condition=body.condition,
                    )
                except Exception as exc:  # noqa: BLE001 - task already aborts on every failure
                    state["error"] = str(exc)

            state["future"] = pool.submit(work)
            return {"accepted": True, "mode": mode}

    @app.post("/api/abort")
    def abort():
        task.abort()
        return {
            "aborted": True,
            "servo_power": "use the physical cutoff to disconnect power",
        }

    @app.post("/api/reset")
    def reset():
        with api_lock:
            if mode != "sim":
                raise ValueError("reset is available only in simulation")
            if state["future"] and not state["future"].done():
                raise ValueError("wait for the current task to stop")
            arm.reset()
            task.previewed = None
            task.stage = "IDLE"
            state["result"] = state["error"] = None
            return {"reset": True}

    @app.post("/api/enable")
    def enable():
        if mode != "hardware":
            raise ValueError("there is no physical controller in this mode")
        if task.busy:
            raise ValueError("task is busy")
        arm.enable()
        state["heartbeat"] = time.monotonic()
        return {"enabled": True}

    @app.post("/api/speech")
    async def transcribe(request: Request):
        if speech is None:
            raise ValueError(
                "offline speech model is not configured; type the instruction"
            )
        data = bytearray()
        async for part in request.stream():
            data.extend(part)
            if len(data) > 2_000_000:
                raise HTTPException(413, "recording too large")
        text = await asyncio.to_thread(speech.transcribe, bytes(data))
        return {"text": text, "requires_preview": True}

    artifacts = {
        "cad": root / "assets/elexhub/provisional/elexhub.FCStd",
        "step": root / "assets/elexhub/provisional/elexhub.step",
        "glb": root / "assets/elexhub/provisional/elexhub.glb",
        "sheets": root / "output/pdf/elexhub-reference-sheets.pdf",
        "wiring-pdf": root / "output/pdf/elexhub-wiring-and-measurements.pdf",
        "wiring": root / "docs/physical-arm/elexhub/wiring.html",
    }

    @app.get("/artifact/{name}")
    def artifact(name: str):
        if name not in artifacts or not artifacts[name].exists():
            raise HTTPException(404)
        return FileResponse(artifacts[name])

    return app

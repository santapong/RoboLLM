"""Sim server: the bed as a ZeroMQ REQ/REP service (SDD §6.4, phase P5, question Q-A).

Runs on the Pi. One outstanding request at a time; physics advances only inside a ``step`` request, so a
policy on another machine sees the same lockstep semantics as the in-process evaluator. Wire format is
msgpack dictionaries; the camera frame travels as raw uint8 bytes with its shape.

Commands (request → reply):
    reset       {"cmd": "reset", "spec": EpisodeSpec dict, "variation": str, "camera_azimuth_deg": f, "camera_translation_m": [3]}
                → {"obs": obs, "t": 0}
    observation {"cmd": "observation", "render": bool}                     → {"obs": obs}
    step        {"cmd": "step", "action": [7 floats], "render": bool}       → {"obs": obs, "error_m", "min_error_m", "success", "progress",
                                                                              "decision": {"ok", "code", "depth"}, "executed": [7], "safety": {...}, "done", "t"}
    query       {"cmd": "query"}                                            → {"commanded_ee": {"pos": [3], "rot": [9]}, "target": [3], "error_m", "min_error_m", "safety": {...}, "t"}
    info        {"cmd": "info"}                                             → host, versions, limits, fps, max_frames, home_rot [9], scene camera pose
    close       {"cmd": "close"}                                            → {"ok": true}
A request that fails validation returns {"error": code, "message": str} and does not advance the episode.

    MUJOCO_GL=egl .venv/bin/python sim/vla-bed/sim_server.py --bind tcp://100.74.8.82:5555
"""

from __future__ import annotations

import argparse
import platform
import socket
import sys
import time
from pathlib import Path

import numpy as np

BED_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BED_DIR))
from families import EpisodeSpec  # noqa: E402

DEFAULT_BIND = "tcp://*:5555"
PROTOCOL = "robollm.vla-bed.sim-server.v1"


def encode_obs(obs: dict | None) -> dict | None:
    """Every `observation.images.<name>` key travels as raw uint8 bytes + shape (None when not rendered)."""
    if obs is None:
        return None
    images = {}
    for key, image in obs.items():
        if key.startswith("observation.images."):
            images[key.split(".")[-1]] = None if image is None else {"bytes": np.ascontiguousarray(image, dtype=np.uint8).tobytes(), "shape": list(image.shape)}
    return {
        "images": images,
        "state": np.asarray(obs["observation.state"], dtype=np.float32).tolist(),
        "camera_lag_ms": np.asarray(obs.get("observation.camera_lag_ms", [0.0]), dtype=np.float32).tolist(),
    }


def decode_obs(o: dict | None) -> dict | None:
    if o is None:
        return None
    out = {
        "observation.state": np.asarray(o["state"], dtype=np.float32),
        "observation.camera_lag_ms": np.asarray(o.get("camera_lag_ms", [0.0]), dtype=np.float32),
    }
    for name, img in o.get("images", {}).items():
        out[f"observation.images.{name}"] = None if img is None else np.frombuffer(img["bytes"], dtype=np.uint8).reshape(img["shape"]).copy()
    out.setdefault("observation.images.front", None)
    return out


def safety_dict(safety) -> dict:
    return {"safe": bool(safety.safe), "rejections": dict(safety.rejections), "measured": dict(safety.measured), "worst_depth": float(safety.worst_depth)}


class SimServerHandler:
    """The request handler, separable from the socket so it can be driven in-process by tests."""

    def __init__(self, env=None):
        if env is None:
            from env import BedEnv
            env = BedEnv(render=True)
        self.env = env
        self.t = 0
        self.closed = False

    def handle(self, req: dict) -> dict:
        try:
            cmd = req.get("cmd") if isinstance(req, dict) else None
            if cmd == "reset":
                return self._reset(req)
            if cmd == "observation":
                return {"obs": encode_obs(self.env.observation(render=bool(req.get("render", True))))}
            if cmd == "step":
                return self._step(req)
            if cmd == "query":
                return self._query()
            if cmd == "info":
                return self._info()
            if cmd == "close":
                self.closed = True
                return {"ok": True}
            return {"error": "bad_request", "message": f"unknown cmd {cmd!r}"}
        except Exception as e:  # never let one bad request kill the server
            return {"error": type(e).__name__, "message": str(e)}

    def _reset(self, req: dict) -> dict:
        spec = req["spec"]
        es = EpisodeSpec(int(spec["seed"]), str(spec["split"]), str(spec["family"]), int(spec["cell"]), tuple(float(x) for x in spec["target"]), tuple(float(x) for x in spec["initial_q"]))
        obs = self.env.reset(es, req.get("variation", "nominal"), float(req.get("camera_azimuth_deg", 0.0)), tuple(req.get("camera_translation_m", (0.0, 0.0, 0.0))))
        self.t = 0
        return {"obs": encode_obs(obs), "t": 0}

    def _step(self, req: dict) -> dict:
        action = np.asarray(req["action"], dtype=np.float64)
        if action.shape != (7,):
            return {"error": "bad_action", "message": f"action must have 7 floats, got shape {action.shape}"}
        r = self.env.step(action, render=bool(req.get("render", False)))
        self.t += 1
        return {"obs": encode_obs(r.observation), "error_m": float(r.error_m), "min_error_m": float(self.env.min_error_m), "success": bool(r.success), "progress": float(r.progress),
                "decision": {"ok": bool(r.decision.ok), "code": str(r.decision.code), "depth": float(r.decision.depth)},
                "executed": np.asarray(r.executed, dtype=np.float32).tolist(), "safety": safety_dict(self.env.safety), "done": bool(r.success), "t": self.t}

    def _query(self) -> dict:
        pos, rot = self.env.commanded_ee
        return {"commanded_ee": {"pos": np.asarray(pos, dtype=np.float64).tolist(), "rot": np.asarray(rot, dtype=np.float64).reshape(-1).tolist()}, "target": np.asarray(self.env.target, dtype=np.float64).tolist(),
                "error_m": float(self.env.error_m), "min_error_m": float(self.env.min_error_m), "safety": safety_dict(self.env.safety), "t": self.t}

    def _info(self) -> dict:
        import mujoco
        from env import FPS, MAX_FRAMES
        from safety import RPY_STEP_LIMIT_RAD, XYZ_STEP_LIMIT_M
        return {"protocol": PROTOCOL, "host": socket.gethostname(), "machine": platform.machine(), "python": platform.python_version(), "mujoco": mujoco.__version__,
                "fps": FPS, "max_frames": MAX_FRAMES, "limits": {"xyz_step_m": XYZ_STEP_LIMIT_M, "rpy_step_rad": RPY_STEP_LIMIT_RAD},
                "home_rot": np.asarray(self.env.controller.home_rot, dtype=np.float64).reshape(-1).tolist(), "image_shape": [self.env.height, self.env.width, 3]}


def serve(bind: str = DEFAULT_BIND, persistent: bool = False) -> int:
    import msgpack
    import zmq

    handler = SimServerHandler()
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(bind)
    print(f"sim server {PROTOCOL} on {bind} (host {socket.gethostname()})", flush=True)
    n, t0 = 0, time.perf_counter()
    try:
        while True:
            raw = sock.recv()
            try:
                req = msgpack.unpackb(raw, raw=False)
            except Exception as e:
                sock.send(msgpack.packb({"error": "bad_msgpack", "message": str(e)}, use_bin_type=True))
                continue
            reply = handler.handle(req)
            sock.send(msgpack.packb(reply, use_bin_type=True))
            n += 1
            if handler.closed:
                if persistent:
                    handler.closed = False
                    continue
                break
    finally:
        print(f"served {n} requests in {time.perf_counter() - t0:.1f} s", flush=True)
        handler.env.close()
        sock.close(0)
        ctx.term()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default=DEFAULT_BIND)
    ap.add_argument("--persistent", action="store_true", help="keep serving after a close request")
    args = ap.parse_args()
    return serve(args.bind, args.persistent)


if __name__ == "__main__":
    raise SystemExit(main())

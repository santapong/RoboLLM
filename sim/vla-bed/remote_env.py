"""RemoteEnv: the BedEnv surface the evaluator uses, served over ZeroMQ by sim_server.py (SDD §6.4, P5).

Exposes reset / observation / step / commanded_ee / target / error_m / min_error_m / safety / controller.home_rot /
close with the same shapes as BedEnv, so evaluate.py's episode loop and policies run unchanged with
``--env zmq://host:5555``. Every call is one REQ/REP round trip; the wire time is accumulated in ``wire_s``.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import numpy as np

BED_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BED_DIR))
from env import StepResult  # noqa: E402
from safety import Decision  # noqa: E402
from sim_server import decode_obs  # noqa: E402


class RemoteError(RuntimeError):
    pass


@dataclass
class RemoteSafety:
    rejections: dict = field(default_factory=dict)
    measured: dict = field(default_factory=dict)
    worst_depth: float = 0.0

    @property
    def safe(self) -> bool:
        return not self.rejections and not self.measured

    def update(self, d: dict) -> None:
        self.rejections = dict(d.get("rejections", {}))
        self.measured = dict(d.get("measured", {}))
        self.worst_depth = float(d.get("worst_depth", 0.0))


class ZmqTransport:
    """send(request dict) → reply dict over a REQ socket; one outstanding request at a time."""

    def __init__(self, url: str, timeout_s: float = 120.0):
        import msgpack
        import zmq
        self._msgpack = msgpack
        self.url = url.replace("zmq://", "tcp://", 1) if url.startswith("zmq://") else url
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.RCVTIMEO, int(timeout_s * 1000))
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(self.url)

    def __call__(self, req: dict) -> dict:
        self.sock.send(self._msgpack.packb(req, use_bin_type=True))
        return self._msgpack.unpackb(self.sock.recv(), raw=False)

    def close(self) -> None:
        self.sock.close(0)
        self.ctx.term()


class RemoteEnv:
    def __init__(self, transport, render: bool = True):
        self.transport = transport
        self.render = render
        self.safety = RemoteSafety()
        self.spec = None
        self.frame = 0
        self._error_m = float("nan")
        self.min_error_m = float("inf")
        self.wire_s = 0.0
        self.requests = 0
        info = self._send({"cmd": "info"})
        self.info = info
        self.controller = SimpleNamespace(home_rot=np.asarray(info["home_rot"], dtype=np.float64).reshape(3, 3))
        self.height, self.width = int(info["image_shape"][0]), int(info["image_shape"][1])

    def _send(self, req: dict) -> dict:
        t0 = time.perf_counter()
        reply = self.transport(req)
        self.wire_s += time.perf_counter() - t0
        self.requests += 1
        if isinstance(reply, dict) and "error" in reply:
            raise RemoteError(f"{reply['error']}: {reply.get('message', '')}")
        return reply

    def reset(self, spec, variation: str = "nominal", camera_azimuth_deg: float = 0.0, camera_translation_m=(0.0, 0.0, 0.0)) -> dict:
        self.spec = spec
        r = self._send({"cmd": "reset", "spec": spec.to_dict(), "variation": variation, "camera_azimuth_deg": float(camera_azimuth_deg), "camera_translation_m": [float(x) for x in camera_translation_m]})
        self.frame = 0
        self.safety = RemoteSafety()
        q = self._send({"cmd": "query"})
        self._error_m, self.min_error_m = float(q["error_m"]), float(q["min_error_m"])
        return decode_obs(r["obs"])

    def observation(self, render: bool = True) -> dict:
        return decode_obs(self._send({"cmd": "observation", "render": bool(render)})["obs"])

    def step(self, action, render: bool = True) -> StepResult:
        r = self._send({"cmd": "step", "action": [float(x) for x in np.asarray(action, dtype=np.float64).reshape(-1)], "render": bool(render)})
        self.frame = int(r["t"])
        self._error_m, self.min_error_m = float(r["error_m"]), float(r["min_error_m"])
        self.safety.update(r["safety"])
        d = r["decision"]
        return StepResult(decode_obs(r["obs"]), float(r["error_m"]), bool(r["success"]), float(r["progress"]), Decision(bool(d["ok"]), str(d["code"]), float(d["depth"])), np.asarray(r["executed"], dtype=np.float32))

    @property
    def commanded_ee(self):
        q = self._send({"cmd": "query"})
        return np.asarray(q["commanded_ee"]["pos"], dtype=np.float64), np.asarray(q["commanded_ee"]["rot"], dtype=np.float64).reshape(3, 3)

    @property
    def target(self) -> np.ndarray:
        return np.asarray(self._send({"cmd": "query"})["target"], dtype=np.float64)

    @property
    def error_m(self) -> float:
        return self._error_m

    def close(self) -> None:
        try:
            self._send({"cmd": "close"})
        finally:
            if hasattr(self.transport, "close"):
                self.transport.close()

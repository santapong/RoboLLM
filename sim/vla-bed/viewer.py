"""Browser viewer for the bed: mjviser in library mode, our loop owns the physics.

Binds to one address only (SDD §4 network rules). On the Pi pass the tailnet
address, never 0.0.0.0::

    nice -n 10 python viewer.py --host 100.x.y.z --port 8090

Then open ``http://100.x.y.z:8090`` on the workstation. Viewer-owned mode
(default) lets the browser pause, resume, and reset; P2+ recording runs use
``--hold`` semantics from their own loops and never share this process.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import mujoco
import viser
from mjviser import Viewer

sys.path.insert(0, str(Path(__file__).resolve().parent / "scene"))
import build_scene  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="address to bind; tailnet IP on the Pi, 127.0.0.1 locally")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--gripper", type=float, default=0.0, help="fingers_actuator ctrl 0..255")
    args = parser.parse_args()

    model = build_scene.load_model()
    data = mujoco.MjData(model)
    build_scene.set_home(model, data, gripper_ctrl=args.gripper)

    def reset_fn(m: mujoco.MjModel, d: mujoco.MjData) -> None:
        build_scene.set_home(m, d, gripper_ctrl=args.gripper)

    def step_fn(m: mujoco.MjModel, d: mujoco.MjData) -> None:
        mujoco.mj_step(m, d)

    server = viser.ViserServer(host=args.host, port=args.port, label="RoboLLM · UR5e VLA bed")
    print(f"viewer: http://{args.host}:{args.port}  (pid {os.getpid()})", flush=True)
    viewer = Viewer(model, data, step_fn=step_fn, reset_fn=reset_fn, server=server)
    viewer.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

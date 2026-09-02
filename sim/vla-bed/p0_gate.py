"""Phase 0 gate G0 (SDD §8): can this machine host the bed?

Evidence written to ``results/p0/<host>/``:

- ``frame.png`` — the 224×224 front-camera render at the home pose
- ``bench.json`` — cold start, physics steps/s, render fps, machine facts, verdict

PASS requires a non-black frame that shows the scene (brightness and contrast
thresholds, plus red target pixels present) and a successful physics loop.
Run with ``MUJOCO_GL=egl`` (default) or ``MUJOCO_GL=osmesa`` as the fallback.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import socket
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

T_IMPORT = time.perf_counter()
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent / "scene"))
import build_scene  # noqa: E402

MIN_MEAN = 8.0  # 0..255; a black frame is ~0
MIN_STD = 12.0
MIN_RED_PIXELS = 20


def cpu_model() -> str:
    try:
        lines = Path("/proc/cpuinfo").read_text().splitlines()
    except OSError:
        return platform.processor() or "unknown"
    # x86 exposes "model name"; the Pi exposes "Model\t: Raspberry Pi 5 ..." (capital M)
    for key in ("model name", "Model", "Hardware"):
        for line in lines:
            if line.startswith(key) and ":" in line:
                return line.split(":", 1)[1].strip()
    return platform.processor() or "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=None, help="default results/p0/<hostname>")
    parser.add_argument("--steps", type=int, default=5000, help="physics steps for the throughput number")
    parser.add_argument("--frames", type=int, default=60, help="renders for the fps number")
    args = parser.parse_args()

    host = socket.gethostname()
    out = args.out or (Path(__file__).resolve().parent / "results" / "p0" / host)
    out.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    model = build_scene.load_model()
    data = mujoco.MjData(model)
    build_scene.set_home(model, data)
    cold_start_s = time.perf_counter() - T_IMPORT
    build_s = time.perf_counter() - t0

    h, w = build_scene.IMAGE_HW
    renderer = mujoco.Renderer(model, height=h, width=w)
    try:
        t1 = time.perf_counter()
        renderer.update_scene(data, camera=build_scene.CAMERA_NAME)
        frame = renderer.render().copy()
        first_render_s = time.perf_counter() - t1
        Image.fromarray(frame).save(out / "frame.png")

        t2 = time.perf_counter()
        for _ in range(args.frames):
            renderer.update_scene(data, camera=build_scene.CAMERA_NAME)
            renderer.render()
        render_fps = args.frames / (time.perf_counter() - t2)
    finally:
        renderer.close()

    t3 = time.perf_counter()
    for _ in range(args.steps):
        mujoco.mj_step(model, data)
    steps_per_s = args.steps / (time.perf_counter() - t3)
    finite = bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
    ee = data.site(build_scene.EE_SITE).xpos.copy()

    mean = float(frame.mean())
    std = float(frame.std())
    red = int(np.sum((frame[..., 0] > 150) & (frame[..., 1] < 90) & (frame[..., 2] < 90)))
    frame_ok = mean > MIN_MEAN and std > MIN_STD and red >= MIN_RED_PIXELS
    verdict = "PASS" if (frame_ok and finite) else "FAIL"

    bench = {
        "schema": "robollm.vla-bed.p0-gate.v1",
        "verdict": verdict,
        "host": {
            "hostname": host,
            "machine": platform.machine(),
            "cpu": cpu_model(),
            "os": platform.platform(),
            "python": platform.python_version(),
            "mujoco": mujoco.__version__,
            "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
        },
        "model": {
            "name": "robollm_vla_bed_ur5e_2f85",
            "nq": int(model.nq),
            "nu": int(model.nu),
            "nbody": int(model.nbody),
            "ngeom": int(model.ngeom),
            "timestep_s": float(model.opt.timestep),
            "menagerie_commit": build_scene.MENAGERIE_COMMIT,
        },
        "frame": {
            "shape": list(frame.shape),
            "mean": round(mean, 2),
            "std": round(std, 2),
            "red_target_pixels": red,
            "thresholds": {"mean": MIN_MEAN, "std": MIN_STD, "red_pixels": MIN_RED_PIXELS},
            "ok": frame_ok,
        },
        "timing": {
            "cold_start_s": round(cold_start_s, 3),
            "build_and_compile_s": round(build_s, 3),
            "first_render_s": round(first_render_s, 3),
            "render_fps_224": round(render_fps, 1),
            "physics_steps_per_s": round(steps_per_s, 0),
            "physics_realtime_factor": round(steps_per_s * float(model.opt.timestep), 1),
            "steps_measured": args.steps,
            "frames_measured": args.frames,
        },
        "physics": {"finite_after_steps": finite, "ee_pos_after_steps": [round(float(x), 4) for x in ee]},
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    (out / "bench.json").write_text(json.dumps(bench, indent=2) + "\n")
    print(json.dumps(bench, indent=2))
    print(f"G0 {verdict} on {host} ({platform.machine()}, MUJOCO_GL={os.environ.get('MUJOCO_GL')}) → {out}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

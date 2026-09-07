"""Render review views of the exported model; no device or training access."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "ros2/robo_arm_driver")]

import cv2
import mujoco
import numpy as np
from robo_arm_driver.config import load_arm_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model-dir", type=Path, default=ROOT / "assets/elexhub/provisional"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = load_arm_config(ROOT / "ros2/robo_arm_driver/config/elexhub.sim.yaml")
    descriptor = json.loads((args.model_dir / "cad-description.json").read_text())
    if descriptor["profile_hash"] != config.profile_hash:
        raise ValueError("model/profile mismatch; regenerate the model first")
    model = mujoco.MjModel.from_xml_path(str(args.model_dir / "elexhub.xml"))
    data = mujoco.MjData(model)
    data.qpos[: config.njoints] = [
        joint.raw_to_rad(joint.home_deg) for joint in config.joints
    ]
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth = 1440
    model.vis.global_.offheight = 1080
    bounds = np.asarray(
        [
            solid["world_mesh_bounds_m"]
            for part in descriptor["parts"]
            for solid in part["solids"]
        ]
    )
    low, high = bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)
    camera = mujoco.MjvCamera()
    camera.lookat = (low + high) / 2
    camera.distance = float(np.linalg.norm(high - low) * 1.3)
    option = mujoco.MjvOption()
    option.geomgroup[:] = [0, 1, 0, 0, 0, 0]
    option.sitegroup[:] = 0
    args.output.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=1080, width=1440) as renderer:
        for name, azimuth, elevation in [
            ("angle", 135, -20),
            ("front", 180, -5),
            ("side", 90, -5),
        ]:
            camera.azimuth, camera.elevation = azimuth, elevation
            renderer.update_scene(data, camera=camera, scene_option=option)
            pixels = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(args.output / f"{name}.png"), pixels)
        renderer.update_scene(data, camera="webcam")
        cv2.imwrite(
            str(args.output / "workspace.png"),
            cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR),
        )
    manifest = {
        "appearance_revision": descriptor["appearance"]["revision"],
        "profile_hash": config.profile_hash,
        "geometry_status": "provisional",
        "physical_evidence": False,
        "rendered_views": ["angle", "front", "side", "workspace"],
        "artifact_sha256": {
            name: hashlib.sha256((args.model_dir / name).read_bytes()).hexdigest()
            for name in [
                "elexhub.xml",
                "elexhub.FCStd",
                "elexhub.step",
                "elexhub.glb",
                "cad-description.json",
            ]
        },
    }
    (args.output / "RENDER_MANIFEST.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()

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


def review_pose(config, values=None):
    """Choose a rendering pose without changing the profile or exported CAD."""
    angles = np.asarray(
        [joint.raw_to_rad(joint.home_deg) for joint in config.joints]
        if values is None
        else values,
        dtype=float,
    )
    if angles.shape != (config.njoints,) or not np.isfinite(angles).all():
        raise ValueError("review pose must contain five finite joint angles")
    for joint, angle in zip(config.joints, angles):
        if not joint.min_rad <= angle <= joint.max_rad:
            raise ValueError(f"review pose is outside {joint.name} joint limits")
    return angles


def visual_bounds(model, data, body_names=None):
    """Bound actual visual vertices at the current pose, including mesh offsets."""
    bounds = []
    for geom_id in np.flatnonzero(model.geom_group == 1):
        if (
            body_names is not None
            and model.body(int(model.geom_bodyid[geom_id])).name not in body_names
        ):
            continue
        mesh_id = model.geom_dataid[geom_id]
        start = model.mesh_vertadr[mesh_id]
        vertices = model.mesh_vert[start : start + model.mesh_vertnum[mesh_id]]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        world = vertices @ rotation.T + data.geom_xpos[geom_id]
        bounds.append([world.min(axis=0), world.max(axis=0)])
    if not bounds:
        raise ValueError("no visual meshes found for the requested review view")
    bounds = np.asarray(bounds)
    return np.stack([bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model-dir", type=Path, default=ROOT / "assets/elexhub/provisional"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--robot-only",
        action="store_true",
        help="render the robot companion without a workspace",
    )
    parser.add_argument(
        "--pose-rad",
        type=float,
        nargs=5,
        metavar=("BASE", "SHOULDER", "ELBOW", "WRIST_PITCH", "WRIST_ROLL"),
        help="review-only joint angles in radians; default is the existing home pose",
    )
    args = parser.parse_args(argv)
    config = load_arm_config(ROOT / "ros2/robo_arm_driver/config/elexhub.sim.yaml")
    angles = review_pose(config, args.pose_rad)
    descriptor = json.loads((args.model_dir / "cad-description.json").read_text())
    if descriptor["profile_hash"] != config.profile_hash:
        raise ValueError("model/profile mismatch; regenerate the model first")
    model_name = "elexhub-robot.xml" if args.robot_only else "elexhub.xml"
    artifact_names = [
        model_name,
        "elexhub.FCStd",
        "elexhub.step",
        "elexhub.glb",
        "elexhub.urdf",
        "cad-description.json",
        "APPEARANCE_PROVENANCE.json",
    ]
    artifact_names += sorted(
        f"meshes/{record['name']}.obj"
        for records in descriptor["visual_meshes"].values()
        for record in records
    )
    artifact_hashes = {
        name: hashlib.sha256((args.model_dir / name).read_bytes()).hexdigest()
        for name in artifact_names
    }
    model = mujoco.MjModel.from_xml_path(str(args.model_dir / model_name))
    data = mujoco.MjData(model)
    data.qpos[: config.njoints] = angles
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth = 1440
    model.vis.global_.offheight = 1080
    low, high = visual_bounds(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat = (low + high) / 2
    camera.distance = float(np.linalg.norm(high - low) * 1.3)
    option = mujoco.MjvOption()
    option.geomgroup[:] = [0, 1, 0, 0, 0, 0]
    option.sitegroup[:] = 0
    views = [
        ("angle", 135, -20),
        ("front", 180, -5),
        ("side", 90, -5),
    ]
    if args.robot_only:
        views += [("top", 180, -89), ("back", 0, -5), ("claw", 135, -20)]
    rendered_views = []
    view_cameras = {}
    args.output.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=1080, width=1440) as renderer:
        for name, azimuth, elevation in views:
            if name == "claw":
                claw_low, claw_high = visual_bounds(
                    model, data, {"tool", "finger_left", "finger_right"}
                )
                camera.lookat = (claw_low + claw_high) / 2
                camera.distance = float(np.linalg.norm(claw_high - claw_low) * 1.5)
            camera.azimuth, camera.elevation = azimuth, elevation
            renderer.update_scene(data, camera=camera, scene_option=option)
            pixels = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(args.output / f"{name}.png"), pixels):
                raise OSError(f"could not save {name} review image")
            rendered_views.append(name)
            view_cameras[name] = {
                "look_at_m": camera.lookat.tolist(),
                "distance_m": camera.distance,
                "azimuth_deg": azimuth,
                "elevation_deg": elevation,
            }
        if not args.robot_only:
            renderer.update_scene(data, camera="webcam")
            if not cv2.imwrite(
                str(args.output / "workspace.png"),
                cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR),
            ):
                raise OSError("could not save workspace review image")
            rendered_views.append("workspace")
            view_cameras["workspace"] = {"named_camera": "webcam"}
    manifest = {
        "appearance_revision": descriptor["appearance"]["revision"],
        "profile_hash": config.profile_hash,
        "geometry_status": descriptor["status"],
        "physical_evidence": False,
        "robot_only": args.robot_only,
        "model_file": model_name,
        "pose": {
            "source": "review-only angles"
            if args.pose_rad is not None
            else "profile home",
            "joint_names": list(config.joint_names),
            "joint_positions_rad": angles.tolist(),
            "gripper": "open; existing sliding-jaw approximation",
            "canonical_home_changed": False,
            "static_exports_pose": "profile home",
        },
        "owner_photo_reference": descriptor["appearance"].get("owner_photo_reference"),
        "visual_bounds_m": [low.tolist(), high.tolist()],
        "rendered_views": rendered_views,
        "view_cameras": view_cameras,
        "artifact_sha256": artifact_hashes,
        "render_sha256": {
            name + ".png": hashlib.sha256(
                (args.output / f"{name}.png").read_bytes()
            ).hexdigest()
            for name in rendered_views
        },
    }
    (args.output / "RENDER_MANIFEST.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()

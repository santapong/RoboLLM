"""Verify exported geometry, run bounded reaches and render the original model."""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from simulation import HumanoidSimulation


def verify():
    meshes = [trimesh.load(path, force="mesh") for path in sorted((HERE / "meshes").glob("*.obj"))]
    if not meshes or not all(mesh.is_watertight and mesh.volume > 0 for mesh in meshes):
        raise ValueError("Each exported visual mesh must be closed with positive volume")
    glb = trimesh.load(HERE / "concept01.glb", force="scene")
    if not 1.4 < glb.extents[1] < 1.8 or len(glb.geometry) < 20:
        raise ValueError("Portable glTF must load as a complete Y-up humanoid")
    output = HERE / "renders"
    output.mkdir(exist_ok=True)
    sim = HumanoidSimulation(model_name="concept01", width=960, height=960)
    rows = []
    try:
        for arm in ("right", "left"):
            for offset in ([0, 0, 0], [.04, 0, .08]):
                sim.reset()
                target = np.array(sim.workspace[arm]["default"]) + offset
                command = {"mode": "reach", "arm": arm, "target": target.tolist()}
                max_violation = 0.0
                for step in range(125):
                    state = sim.step(command if step == 0 else None)
                    max_violation = max(max_violation, state["safety"]["joint_limit_violation_rad"])
                    if not state["safety"]["finite"] or state["safety"]["fault"]:
                        raise RuntimeError("Concept dynamics became invalid")
                rows.append({"arm": arm, "target_m": target.tolist(), "seconds": 5,
                             "final_error_m": state["error_m"], "max_joint_limit_violation_rad": max_violation,
                             "safety": state["safety"]})
        sim.reset()
        for _ in range(25):
            sim.step()
        # Model renders show the authored reference pose rather than an IK
        # target. The stand remains visible to disclose the supported base.
        marker = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, "target_sphere")
        sim.model.geom_rgba[marker, 3] = 0
        # MuJoCo 3.10 reads the projection from model.vis.global, not the
        # similarly named MjvCamera flag, when deriving the free-camera frustum.
        sim.model.vis.global_.orthographic = 1
        sim.model.vis.global_.fovy = 1.90
        camera = mujoco.MjvCamera()
        camera.orthographic = 1
        camera.lookat[:] = [0, 0, .84]
        camera.distance = 3
        options = mujoco.MjvOption()
        options.geomgroup[3] = 0
        renderer = mujoco.Renderer(sim.model, height=960, width=960)
        views = {}
        try:
            for name, azimuth, elevation in (("front", 180, 0), ("side", 90, 0), ("rear", 0, 0), ("three-quarter", 145, -10)):
                camera.azimuth, camera.elevation = azimuth, elevation
                renderer.update_scene(sim.data, camera=camera, scene_option=options)
                if not renderer.scene.camera[0].orthographic:
                    raise RuntimeError("Reference view did not use orthographic projection")
                path = output / f"{name}.png"
                Image.fromarray(renderer.render()).save(path)
                views[name] = {"azimuth": azimuth, "elevation": elevation, "orthographic_vertical_size_m": 1.9,
                               "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        finally:
            renderer.close()
        report = {"method": "authored image-guided procedural model; estimated physics", "mujoco_version": mujoco.__version__,
                  "model_sha256": hashlib.sha256((HERE / "model.xml").read_bytes()).hexdigest(),
                  "glb_sha256": hashlib.sha256((HERE / "concept01.glb").read_bytes()).hexdigest(),
                  "mesh_assets_checked": len(meshes), "all_meshes_watertight_positive_volume": True,
                  "gltf_geometry_count": len(glb.geometry), "gltf_y_up_bounds_m": glb.bounds.tolist(),
                  "joints": sim.model.njnt, "actuators": sim.model.nu,
                  "reaches": rows, "renders": views,
                  "limitations": ["Authored approximation, not photogrammetry or a learned image-to-3D conversion", "Dimensions/inertia/masses/gains are estimated", "Supported pelvis; no walking or physical validation", "Self-collision disabled; fixed finger geometry"]}
        (HERE / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return report
    finally:
        sim.close()


if __name__ == "__main__":
    verify()

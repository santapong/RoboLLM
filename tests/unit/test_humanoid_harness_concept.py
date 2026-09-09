"""Independent checks of the saved original 3D model and its dynamics."""

import sys
from pathlib import Path

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")
pytest.importorskip("mujoco")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples/humanoid_harness"))
from simulation import HumanoidSimulation

CONCEPT = ROOT / "examples/humanoid_harness/models/concept01"


def test_exported_visual_parts_are_closed_solids():
    paths = list((CONCEPT / "meshes").glob("*.obj"))
    assert len(paths) >= 20
    for path in paths:
        mesh = trimesh.load(path, force="mesh")
        assert mesh.is_watertight and mesh.volume > 0, path.name
        assert np.isfinite(mesh.vertices).all()


def test_portable_glb_has_full_humanoid_and_named_body_hierarchy():
    scene = trimesh.load(CONCEPT / "concept01.glb", force="scene")
    assert 1.4 < scene.extents[1] < 1.8  # glTF export is Y-up, metres.
    assert .6 < scene.extents[2] < 1.0
    assert len(scene.geometry) >= 20
    assert {"pelvis", "head", "left_wrist_yaw_link", "right_wrist_yaw_link"} <= set(scene.graph.nodes)


@pytest.mark.parametrize("arm", ["right", "left"])
def test_original_model_reaches_with_estimated_physics(arm):
    sim = HumanoidSimulation(model_name="concept01")
    try:
        assert sim.model.njnt == sim.model.nu == 29
        assert sim.source_kind == "original-procedural"
        target = sim.workspace[arm]["default"]
        for step in range(125):
            state = sim.step({"mode": "reach", "arm": arm, "target": target} if step == 0 else None)
            assert state["safety"]["finite"]
            assert state["safety"]["fault"] is None
            assert state["safety"]["joint_limit_violation_rad"] < 1e-6
        assert state["error_m"] < .035
    finally:
        sim.close()

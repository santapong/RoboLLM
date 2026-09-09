"""Multiple-embodiment checks and the original-MJCF integration contract."""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from examples.humanoid_harness import simulation
from examples.humanoid_harness.fetch_model import REVISION, model_assets
from examples.humanoid_harness.simulation import (
    ARM_JOINTS,
    WORKSPACE,
    HumanoidSimulation,
)


@pytest.fixture
def h1():
    if not (model_assets("h1") / "provenance.json").exists():
        pytest.skip("Run fetch_model.py --model h1 for the pinned H1 assets")
    sim = HumanoidSimulation(model_name="h1", width=320, height=240)
    yield sim
    sim.close()


@pytest.mark.parametrize("arm", ["left", "right"])
def test_h1_reaches_with_four_joint_arms_and_original_force_limits(h1, arm):
    pelvis = h1.data.body("pelvis").xpos.copy()
    initial = h1.step({"mode": "reach", "arm": arm, "target": h1.workspace[arm]["default"]})
    for _ in range(124):
        state = h1.step()
        assert state["safety"]["joint_limit_violation_rad"] < 0.001
        assert state["safety"]["fault"] is None
    assert h1.model.nq == h1.model.nv == h1.model.nu == 19
    assert len(h1._arms[arm]) == 4
    assert state["error_m"] < 0.035
    assert state["error_m"] < initial["error_m"] / 5
    assert state["source_revision"] == REVISION
    assert state["source_kind"] == "upstream-robot"
    assert state["model_id"] == "h1"
    np.testing.assert_array_equal(h1.data.body("pelvis").xpos, pelvis)
    np.testing.assert_array_equal(h1.model.actuator(f"{arm}_shoulder_pitch").forcerange, [-40, 40])
    np.testing.assert_array_equal(h1.model.actuator(f"{arm}_elbow").forcerange, [-18, 18])
    np.testing.assert_allclose(h1.model.joint(f"{arm}_elbow").range, [-1.25, 2.61])


@pytest.mark.parametrize("command", [{"mode": "demo"}, {"mode": "pose", "pose": "wave"}, {"mode": "pose", "pose": "arms_up"}])
def test_h1_scripted_modes_work_without_wrist_joints(h1, command):
    h1.step(command)
    for _ in range(124):
        state = h1.step()
        assert state["safety"]["finite"]
        assert state["safety"]["fault"] is None
        assert state["safety"]["joint_limit_violation_rad"] < 0.001


@pytest.fixture
def original_xml(tmp_path):
    """An independently described robot; no copied upstream model assets."""
    root = ET.Element("mujoco", model="original-test")
    ET.SubElement(root, "compiler", angle="radian")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", range="-2.8 2.8", limited="true", armature="0.03")
    ET.SubElement(defaults, "geom", type="sphere", size="0.015", mass="0.05", contype="0", conaffinity="0")
    world = ET.SubElement(root, "worldbody")
    pelvis = ET.SubElement(world, "body", name="pelvis", pos="0 0 0.82")
    ET.SubElement(pelvis, "geom", size="0.10", mass="5")
    actuators = ET.SubElement(root, "actuator")
    for arm, sign in (("left", 1), ("right", -1)):
        parent = pelvis
        for index, part in enumerate(ARM_JOINTS):
            pos = f"0 {sign * 0.2} 0.4" if index == 0 else "0 0 -0.06"
            parent = ET.SubElement(parent, "body", name=f"{arm}_{part}_link", pos=pos)
            ET.SubElement(parent, "geom")
            name = f"{arm}_{part}_joint"
            ET.SubElement(parent, "joint", name=name, axis="0 1 0")
            ET.SubElement(actuators, "position", name=name, joint=name, kp="100", kv="10", ctrllimited="true", ctrlrange="-2.8 2.8", forcelimited="true", forcerange="-10 10")
        ET.SubElement(parent, "site", name=f"{arm}_hand_tip", pos="0.001 0.002 -0.08")
    path = tmp_path / "model.xml"
    ET.ElementTree(root).write(path)
    return path


def test_custom_xml_keeps_own_identity_sites_home_and_support_height(original_xml):
    metadata = {"id": "test-concept", "label": "Original test concept", "source_revision": "design-test",
                "source_kind": "original-procedural", "home": {"left_elbow_joint": 0.1}, "workspace": WORKSPACE}
    sim = HumanoidSimulation(custom_model_path=original_xml, custom_metadata=metadata)
    try:
        state = sim.state()
        assert state["model_id"] == "test-concept"
        assert state["source_kind"] == "original-procedural"
        assert state["source_revision"] == "design-test"
        assert "Unitree" not in state["model"]
        assert state["joints"]["left_elbow_joint"] == pytest.approx(0.1)
        np.testing.assert_allclose(sim.model.site("left_hand_tip").pos, [0.001, 0.002, -0.08])
        assert sim.model.geom("pelvis_support").pos[2] == pytest.approx((0.05 + 0.83) / 2)
        assert len(state["model_sha256"]) == 64
    finally:
        sim.close()


def test_custom_registry_resolves_reference_and_rejects_identity_collision(original_xml, monkeypatch):
    registry = original_xml.parent / "registry"
    directory = registry / "test-concept"
    directory.mkdir(parents=True)
    (directory / "model.xml").write_bytes(original_xml.read_bytes())
    metadata = {"id": "test-concept", "label": "Original concept", "source_kind": "original-procedural",
                "workspace": WORKSPACE, "reference_image": "turnaround.png"}
    descriptor = directory / "model.json"
    descriptor.write_text(json.dumps(metadata))
    monkeypatch.setattr(simulation, "MODELS_DIRECTORY", registry)
    item = next(item for item in simulation.list_models() if item["id"] == metadata["id"])
    assert item["available"]
    assert item["reference_image"] == str(directory / "turnaround.png")
    descriptor.write_text(json.dumps({**metadata, "id": "g1"}))
    with pytest.raises(ValueError, match="not replace an official model"):
        simulation.list_models()


def test_custom_motor_actuators_fail_closed(original_xml):
    root = ET.parse(original_xml).getroot()
    actuator = root.find("actuator")[0]
    actuator.tag = "motor"
    del actuator.attrib["kp"]
    del actuator.attrib["kv"]
    ET.ElementTree(root).write(original_xml)
    with pytest.raises(ValueError, match="position servo"):
        HumanoidSimulation(custom_model_path=original_xml)

"""Compose the UR5e + Robotiq 2F-85 red-target scene from unmodified upstream files.

Route M overlay (SDD §5, §12): the MuJoCo Menagerie files are loaded from a
git-ignored checkout at a pinned commit and composed in memory with MuJoCo's
``MjSpec`` API. Nothing upstream is edited or copied into this repository; our
additions (front camera, red target, prefixes) live only here.

Usage as a library::

    from build_scene import build_spec, load_model, HOME_QPOS
    model = load_model()                       # uses the default Menagerie path

Usage as a script (exports the compiled XML for inspection; the export is
git-ignored because it embeds upstream geometry)::

    python scene/build_scene.py --export scene/ur5e_red_target.generated.xml
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import mujoco
import numpy as np

BED_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MENAGERIE = BED_DIR / "assets" / "mujoco_menagerie"
MENAGERIE_COMMIT = "e4049d0a3bfd58d2a3081614e6777d4007e3f86a"  # 2026-09-01, SDD §11

ARM_SCENE = "universal_robots_ur5e/scene.xml"  # ur5e.xml + floor, lights, skybox
GRIPPER = "robotiq_2f85/2f85.xml"
GRIPPER_PREFIX = "2f85/"

# UR5e "home" keyframe from ur5e.xml, restated here so the composed model does
# not depend on how attach() treats child keyframes.
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
ARM_ACTUATORS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
GRIPPER_ACTUATOR = GRIPPER_PREFIX + "fingers_actuator"  # ctrl 0..255
EE_SITE = GRIPPER_PREFIX + "pinch"  # fingertip frame; the bed's end-effector

CAMERA_NAME = "front"
WRIST_CAMERA_NAME = "wrist"  # recipe v6: on wrist_3_link, looking along the tool axis (+y, where the 2F-85 mounts at y = 0.107)
WRIST_CAMERA_POS = np.array([0.0, 0.03, 0.10])  # 0.10 m off-axis: looks past the gripper body at the fingertips
WRIST_CAMERA_TILT_DEG = 21.0  # optical axis meets the tool axis ≈ 0.26 m ahead, at the pinch site
WRIST_CAMERA_FOVY = 70.0
CAMERA_POS = np.array([1.35, -1.25, 0.95])
CAMERA_LOOKAT = np.array([0.05, -0.35, 0.30])
IMAGE_HW = (224, 224)

TARGET_NAME = "red_target"
TARGET_POS_NOMINAL = np.array([0.45, -0.35, 0.25])  # P1 randomises per goal family
TARGET_RADIUS = 0.02


def _lookat_xyaxes(pos: np.ndarray, lookat: np.ndarray) -> np.ndarray:
    """MuJoCo camera looks down -z; return the 6-vector xyaxes for a look-at."""
    forward = lookat - pos
    forward /= np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return np.concatenate([right, up])


def menagerie_dir() -> Path:
    override = os.environ.get("VLA_BED_MENAGERIE")
    return Path(override).expanduser() if override else DEFAULT_MENAGERIE


def build_spec(menagerie: Path | None = None) -> mujoco.MjSpec:
    root = menagerie or menagerie_dir()
    arm_path = root / ARM_SCENE
    gripper_path = root / GRIPPER
    for p in (arm_path, gripper_path):
        if not p.exists():
            raise FileNotFoundError(
                f"{p} missing — run scripts/pi_setup.sh or clone MuJoCo Menagerie at "
                f"{MENAGERIE_COMMIT} into {root}"
            )

    spec = mujoco.MjSpec.from_file(str(arm_path))
    spec.modelname = "robollm_vla_bed_ur5e_2f85"

    # The 2F-85 model asks for an elliptic friction cone and impratio 10 (its
    # pads need them to hold objects); adopt them before attaching so the
    # attach does not silently keep the arm scene's defaults.
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0

    # Gripper on the flange, upstream file untouched.
    gripper = mujoco.MjSpec.from_file(str(gripper_path))
    spec.attach(gripper, prefix=GRIPPER_PREFIX, site=spec.site("attachment_site"))

    # Fixed front camera: identical intrinsics in every episode (SDD §6.1).
    spec.worldbody.add_camera(
        name=CAMERA_NAME,
        pos=CAMERA_POS,
        xyaxes=_lookat_xyaxes(CAMERA_POS, CAMERA_LOOKAT),
        fovy=42.0,
    )
    # Wrist camera (recipe v6): MuJoCo cameras look down −z; x_cam = link x, forward = +y tilted toward −z by the tilt.
    s, c = np.sin(np.radians(WRIST_CAMERA_TILT_DEG)), np.cos(np.radians(WRIST_CAMERA_TILT_DEG))
    spec.body("wrist_3_link").add_camera(name=WRIST_CAMERA_NAME, pos=WRIST_CAMERA_POS, xyaxes=[1.0, 0.0, 0.0, 0.0, s, c], fovy=WRIST_CAMERA_FOVY)
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, 640)
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, 480)

    # Red target: visual only, no collision, mocap so P1 can move it per family.
    target = spec.worldbody.add_body(name=TARGET_NAME, pos=TARGET_POS_NOMINAL, mocap=True)
    target.add_geom(
        name=TARGET_NAME + "_geom",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[TARGET_RADIUS, 0, 0],
        rgba=[0.92, 0.10, 0.10, 1.0],
        contype=0,
        conaffinity=0,
        group=0,
    )
    target.add_site(name=TARGET_NAME + "_site", size=[0.005, 0, 0], rgba=[0, 0, 0, 0])
    return spec


def load_model(menagerie: Path | None = None) -> mujoco.MjModel:
    return build_spec(menagerie).compile()


def set_home(model: mujoco.MjModel, data: mujoco.MjData, gripper_ctrl: float = 0.0) -> None:
    for name, q in zip(ARM_JOINTS, HOME_QPOS):
        data.qpos[model.joint(name).qposadr[0]] = q
    data.qvel[:] = 0.0
    for name, q in zip(ARM_ACTUATORS, HOME_QPOS):
        data.ctrl[model.actuator(name).id] = q
    data.ctrl[model.actuator(GRIPPER_ACTUATOR).id] = gripper_ctrl
    mujoco.mj_forward(model, data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--menagerie", type=Path, default=None)
    parser.add_argument("--export", type=Path, default=None, help="write compiled MJCF (git-ignored)")
    args = parser.parse_args()
    spec = build_spec(args.menagerie)
    model = spec.compile()
    print(
        f"model {spec.modelname}: nq={model.nq} nu={model.nu} nbody={model.nbody} "
        f"ngeom={model.ngeom} cameras={[model.camera(i).name for i in range(model.ncam)]}"
    )
    if args.export:
        args.export.write_text(spec.to_xml())
        print(f"exported {args.export}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

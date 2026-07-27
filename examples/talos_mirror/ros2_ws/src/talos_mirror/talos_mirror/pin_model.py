"""TALOS pinocchio model loading + small FK helpers shared by qp_retarget.py
(the live QP node path) and tools/qp_ab_harness.py (the offline A/B replay,
which needs the SAME model to compute independent metrics for BOTH modes).

pinocchio lives in /opt/mpvenv only -- importing this module under system
python raises ModuleNotFoundError, same failure mode as importing mediapipe
under the wrong interpreter (see body_track.py's module docstring / the
house numpy law). Import it lazily at call sites that might run under
system python (mirror_node.py's direct mode must never need this module).

THE EXPANDED URDF: talos_moveit_config/config/talos.urdf.xacro is a xacro
file (macros, $(find) package lookups) -- pinocchio cannot parse xacro
directly, only plain URDF XML. This module expands it with the `xacro` CLI
(installed in ros2-talos:jazzy) at STARTUP inside the container, exactly as
MoveItConfigsBuilder does internally for move_group (same source file, same
default args -- test:=false, use_capsule_collision:=false -- so this is the
identical kinematic model MoveIt itself plans against, not a re-derivation).
`ros2 pkg prefix`/ament_index_python resolve the share directory so this
works from either the built ros2_ws install/ or a colcon-built container;
TALOS_URDF_XACRO/TALOS_URDF_XML env vars override the lookup (used by
tools/qp_ab_harness.py to avoid re-running xacro on every replay call, and
by tests that want a pinned URDF snapshot).
"""

import os
import subprocess

import numpy as np
import pinocchio as pin

from talos_mirror.talos_config import ALL_JOINTS, TIP_LINKS

# Robot-side wrist tip frame per human/robot side -- same TIP_LINKS
# talos_config already uses for the SRDF group definitions (arm_l/arm_r),
# so the QP layer's Cartesian targets attach to exactly the link MoveIt's
# own /compute_fk would report for that group's tip.
WRIST_FRAME = {"left": TIP_LINKS["arm_l"], "right": TIP_LINKS["arm_r"]}
TORSO_FRAME = "torso_2_link"


def _expand_xacro():
    """Run the `xacro` CLI against talos_moveit_config's own URDF xacro,
    resolved via ament_index_python (works from install/ inside the built
    container). Raises RuntimeError with a clear diagnosis on failure --
    the two likely causes are "talos_mirror's workspace was never colcon
    built" (see docker/ros2-arm's `talos`/`sweep` guards) and "xacro is not
    on PATH" (it should always be, from /opt/ros/jazzy/setup.bash).
    """
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

    try:
        share = get_package_share_directory("talos_moveit_config")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "talos_moveit_config not found by ament_index -- source "
            "install/setup.bash first (colcon build --packages-select "
            "talos_moveit_config talos_description ... ; see docker/ros2-arm)"
        ) from exc

    xacro_path = os.path.join(share, "config", "talos.urdf.xacro")
    if not os.path.exists(xacro_path):
        raise RuntimeError(f"expected xacro file missing: {xacro_path}")

    try:
        result = subprocess.run(
            ["xacro", xacro_path], capture_output=True, text=True, check=True, timeout=30,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "`xacro` is not on PATH -- source /opt/ros/jazzy/setup.bash first"
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"xacro expansion of {xacro_path} failed:\n{exc.stderr}") from exc
    return result.stdout


def load_urdf_xml():
    """Expanded TALOS URDF XML text. TALOS_URDF_XML (a path to an already-
    expanded URDF file) short-circuits the xacro run entirely -- used by the
    A/B harness to expand once and reuse across many replay() calls, and by
    anything that wants a pinned snapshot rather than "whatever xacro
    produces right now"."""
    cached = os.environ.get("TALOS_URDF_XML")
    if cached and os.path.exists(cached):
        with open(cached, "r") as f:
            return f.read()
    return _expand_xacro()


def _rotation_between(u, v):
    """Rotation matrix taking unit vector u onto unit vector v by the
    shortest arc (Rodrigues' formula). Handles the near-parallel (identity)
    and near-antiparallel (180 deg about an arbitrary perpendicular axis --
    the twist about that axis is unobservable either way, same "genuinely
    unobservable DOF, park it" reasoning retarget.py already applies to
    ankle roll / wrist palm twist) cases explicitly rather than dividing by
    a near-zero sin(angle)."""
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-12 or nv < 1e-12:
        return np.eye(3)
    u, v = u / nu, v / nv
    c = float(np.dot(u, v))
    if c > 1.0 - 1e-10:
        return np.eye(3)
    if c < -1.0 + 1e-10:
        axis = np.cross(u, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(u, [0.0, 1.0, 0.0])
        axis = axis / np.linalg.norm(axis)
        return _axis_angle(axis, np.pi)
    axis = np.cross(u, v)
    s = np.linalg.norm(axis)
    axis = axis / s
    angle = np.arccos(np.clip(c, -1.0, 1.0))
    return _axis_angle(axis, angle)


def _axis_angle(axis, angle):
    k = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def rotation_angle(r):
    """Geodesic angle (radians) of a rotation matrix, via the standard
    trace formula -- used both by qp_retarget's own residual report and by
    the A/B harness's independent orientation-error metric."""
    c = (np.trace(r) - 1.0) * 0.5
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


class TalosPinModel:
    """Loads the expanded TALOS URDF into pinocchio once and exposes the
    handful of FK operations qp_retarget.py and the A/B harness both need,
    keyed by TALOS's OWN joint names (talos_config.ALL_JOINTS) rather than
    pinocchio's internal joint index order (leg_l, leg_r, torso, arm_l,
    arm_r, head -- NOT the ALL_JOINTS order) so no caller has to know or
    care what that order is.
    """

    def __init__(self, urdf_xml=None):
        self.model = pin.buildModelFromXML(urdf_xml or load_urdf_xml())
        self.data = self.model.createData()
        assert self.model.nq == self.model.nv == len(ALL_JOINTS), (
            f"unexpected model shape: nq={self.model.nq} nv={self.model.nv} "
            f"expected {len(ALL_JOINTS)} (all-revolute, fixed-base pelvis -- "
            f"see talos_config/retarget.py's PINNED base_link note)"
        )
        self.idx_q, self.idx_v = {}, {}
        for name in ALL_JOINTS:
            jid = self.model.getJointId(name)
            self.idx_q[name] = self.model.joints[jid].idx_q
            self.idx_v[name] = self.model.joints[jid].idx_v

        self.wrist_frame_id = {}
        for side, frame_name in WRIST_FRAME.items():
            fid = self.model.getFrameId(frame_name)
            if fid >= self.model.nframes:
                raise RuntimeError(f"frame {frame_name!r} not found in the URDF")
            self.wrist_frame_id[side] = fid
        self.torso_frame_id = self.model.getFrameId(TORSO_FRAME)

        self.arm_joint_names = {
            "left": [f"arm_left_{k}_joint" for k in range(1, 8)],
            "right": [f"arm_right_{k}_joint" for k in range(1, 8)],
        }
        self.joint_side = {}
        for side, names in self.arm_joint_names.items():
            for n in names:
                self.joint_side[n] = side

    def neutral(self):
        return pin.neutral(self.model)

    def q_from_dict(self, d, base=None):
        q = np.array(base, dtype=float) if base is not None else self.neutral()
        for name, idx in self.idx_q.items():
            if name in d:
                q[idx] = float(d[name])
        return q

    def dict_from_q(self, q):
        return {name: float(q[idx]) for name, idx in self.idx_q.items()}

    def fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def wrist_pose(self, q, side):
        """pin.SE3 of the wrist tip frame in the world/base_link frame for
        joint vector `q`. Runs its own FK pass -- callers needing several
        poses off the SAME q should call self.fk(q) once and read
        self.data.oMf[...] directly instead (qp_retarget.py does this)."""
        self.fk(q)
        return self.data.oMf[self.wrist_frame_id[side]].copy()

    def build_wrist_target(self, m4_q, forearm_dir_torso, side):
        """SE3 Cartesian target for one wrist: POSITION = the M4 direct-copy
        solution's own FK (m4_q has that side's shoulder/elbow already
        direction-matched via retarget.solve_arm, wrist 5/6/7 parked at 0 --
        this is exactly the FK-verified-to-0.00-deg M4 baseline, so the QP
        starts from a position that is already correct and only has to fix
        orientation: "refines rather than replaces"). ORIENTATION = the
        minimal rotation that takes the M4 wrist frame's local -Z axis onto
        the mirrored forearm direction (`forearm_dir_torso`, a unit vector
        in the TORSO frame -- exactly what retarget.solve_arm's `b` already
        is) expressed in world via the torso frame's own FK rotation.

        -Z (not +Z) is a documented CHOICE, EMPIRICALLY calibrated, not a
        measured fact from any datasheet: MediaPipe Pose has no hand/palm
        landmarks (same gap that parks wrist joints 5-7 at 0 in M4), so
        which local axis "should" point along the forearm is a convention
        this module has to pick, and the A/B harness's metric reuses the
        SAME convention. Swept q1-4 across each joint's full range at
        wrist=0 and compared each of the tool frame's three local axes (in
        world) against retarget.arm_dirs' own forearm output (an
        independent code path) for that same q: local -Z tracks it to
        within ~4-5 deg across the ENTIRE swept envelope (a fixed residual
        of the wrist's own offset geometry, not a per-pose error), +Z is
        ~176 deg off (antiparallel) EVERYWHERE, and local Y stays
        near-exactly perpendicular (~90 deg) everywhere -- i.e. this is a
        FIXED structural fact of the URDF's wrist-tool chain, not motion-
        dependent, so -Z is the only one of the three that makes the QP's
        job "close a small gap" rather than "chase an antipodal target
        through a +-0.68 rad joint7 range forever" (found and fixed during
        development: +Z made the QP saturate the narrow wrist joint limits
        on nearly every tick of the A/B session while barely closing the
        orientation error at all).

        Roll about that axis (palm twist) is left as whatever the minimal
        rotation gives, i.e. unconstrained/free -- there is no landmark
        this milestone tracks that could pin it down, the same "hold,
        don't invent" reasoning retarget.py already applies to ankle roll.
        """
        self.fk(m4_q)
        oMf = self.data.oMf[self.wrist_frame_id[side]]
        r_torso = self.data.oMf[self.torso_frame_id].rotation
        forearm_world = r_torso @ np.asarray(forearm_dir_torso, dtype=float)
        if np.linalg.norm(forearm_world) < 1e-9:
            return pin.SE3(oMf.rotation.copy(), oMf.translation.copy())
        z_local_world = oMf.rotation @ np.array([0.0, 0.0, -1.0])
        r_delta = _rotation_between(z_local_world, forearm_world)
        r_target = r_delta @ oMf.rotation
        return pin.SE3(r_target, oMf.translation.copy())

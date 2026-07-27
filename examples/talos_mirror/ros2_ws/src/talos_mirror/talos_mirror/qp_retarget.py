"""c2-qp / M5: pink task-space differential-IK layer that REFINES the M4
direct-copy arm solution into a real Cartesian wrist target (position +
orientation), instead of replacing it.

WHY A QP LAYER AT ALL (vs. more closed-form math like retarget.py's
solve_arm): retarget.py's solve_arm matches DIRECTIONS only (shoulder->elbow
and elbow->wrist unit vectors) and leaves wrist joints 5/6/7 permanently
parked at 0.0, because MediaPipe Pose carries no hand/palm landmarks to
target an orientation with. The QP layer's job is narrow and specific: given
the SAME mirrored forearm direction retarget.py already computes, ALSO
orient the wrist tool frame along it -- a genuine 3-DOF (wrist joints 5/6/7)
Cartesian orientation problem layered on top of a position that is already
exactly right (see pin_model.build_wrist_target's docstring for why the
position target is just M4's own FK).

PER-TICK PIPELINE (see qp_pose_source.QPMirrorPoseSource for where this
plugs into mirror_node):
  1. retarget.retarget() runs exactly as M4 does (direct-copy solve for
     every group, INCLUDING both arms 1-4 + wrist 5-7=0) -- this IS the
     posture-bias source, unchanged.
  2. For each ARM whose gate is currently on, pin_model.build_wrist_target
     turns that arm's mirrored forearm direction into a full SE3 Cartesian
     target.
  3. ONE pink QP solve (FrameTask per gated wrist + a PostureTask biased to
     the M4 q) produces a joint VELOCITY; only that GATED side's WRIST
     joints (5/6/7) are allowed to move -- shoulder/elbow (1-4), legs,
     head, torso, and any ungated arm are all zeroed before integration, so
     they track M4 byte-for-byte, never even the QP's own regularization
     pull (see __init__'s docstring for why 1-4 are LOCKED, not just
     posture-biased) -- and integrated one tick (dt) via pink's own
     Configuration.integrate (handles wrap-safe joint addition --
     irrelevant here since every TALOS joint is a plain revolute, but it is
     the correct/idiomatic call).
  4. Warm start: the SOLVER OBJECT keeps no q of its own between ticks for
     joints 1-4 (those are reseeded from the CURRENT tick's M4 solution
     every time and then locked, per (3)); only wrist joints 5-7 warm-start
     from the PREVIOUS tick's QP result, because those are the only joints
     the QP is actually solving for and orientation changes comparatively
     slowly tick to tick.

Joint-limit + velocity-limit constraints: pink.limits.ConfigurationLimit and
VelocityLimit, built directly from the SAME pinocchio model (whose
<limit lower/upper/velocity> values are the expanded URDF's own, i.e. the
identical numbers docs/joints.json (position) and joint_limits.MEASURED
(position + velocity) record -- joints.json itself carries no velocity
field, see joint_limits.py's own header for where MEASURED's velocity
column comes from). mirror_node's existing clamp()/slew() pipeline
(joint_limits.py) still runs downstream on whatever this module returns, as
an identical backstop to the one M4 already has -- this module's own limits
are the QP's INTERNAL constraint, not a replacement for that backstop.

solver="quadprog" -- the only solver installed (docker/Dockerfile's build-
layer verify step already proves solve_ik(..., solver="quadprog") works
against this exact image).

Import this module ONLY under /opt/mpvenv/bin/python (see pin_model.py's
header) -- mirror.launch.py already launches both track_node and
mirror_node with prefix=/opt/mpvenv/bin/python unconditionally, so mode:=qp
needs no launch change; a bare `python3` run will fail with
ModuleNotFoundError on `pink`, the same tell the house law calls out for
mediapipe ("synthetic/direct mode works, camera/pink mode dies").
"""

import math
import time

import numpy as np
import pinocchio as pin
import pink
from pink.limits import ConfigurationLimit, VelocityLimit
from pink.tasks import FrameTask, PostureTask

from talos_mirror.pin_model import WRIST_FRAME, TalosPinModel, rotation_angle
from talos_mirror.talos_config import ALL_JOINTS

SIDES = ("left", "right")


class WristQPSolver:
    def __init__(
        self,
        urdf_xml=None,
        position_cost=1.0,
        orientation_cost=0.5,
        posture_cost=1e-2,
        solver="quadprog",
        damping=1e-8,
    ):
        self.pm = TalosPinModel(urdf_xml)
        self.solver = solver
        self.damping = damping

        self.frame_tasks = {
            side: FrameTask(
                WRIST_FRAME[side],
                position_cost=position_cost,
                orientation_cost=orientation_cost,
            )
            for side in SIDES
        }
        self.posture_task = PostureTask(cost=posture_cost)
        self.limits = [ConfigurationLimit(self.pm.model), VelocityLimit(self.pm.model)]

        # The QP is only ever allowed to MOVE wrist joints 5/6/7 (roll,
        # pitch, yaw) -- shoulder+elbow (1-4) are locked to the M4 seed
        # exactly, same treatment as legs/head/torso, not just lightly
        # posture-regularized. This is what makes "refines rather than
        # replaces" literally true rather than aspirational: M4's 1-4
        # solve already reproduces the mirrored arm DIRECTION exactly (the
        # FK-verified baseline), so the Cartesian POSITION target
        # (pin_model.build_wrist_target's `oMf.translation`, which is
        # exactly FK(m4_q)) is already correct before the QP runs a single
        # iteration; the only real degrees of freedom the wrist frame's
        # ORIENTATION needs are 5/6/7, and 3 free joints against a 3-DOF
        # orientation residual is a well-posed (generically exactly
        # determined) problem, not a redundant one that would otherwise let
        # the QP quietly walk the shoulder away from a position that was
        # already right (measured during development: leaving 1-4 free
        # under only a WEAK posture cost let a large orientation target
        # drag the shoulder/elbow chain and reintroduced several cm of
        # position error the M4 baseline had already eliminated -- locking
        # 1-4 removes that failure mode structurally instead of by tuning
        # posture_cost up until it happens to be big enough).
        self._wrist_joint_names = {
            side: self.pm.arm_joint_names[side][4:7] for side in SIDES
        }

        # Warm-start memory: only wrist joints 5/6/7 persist between ticks
        # (see module docstring). None until the first solve.
        self._warm_wrist = None

    def reset(self):
        """Drop warm-start memory -- call when tracking is lost/reacquired
        so a stale wrist orientation from minutes ago never seeds a fresh
        solve (mirroring mirror_node's own loss_hold_sec -> HOME behaviour)."""
        self._warm_wrist = None

    def solve(self, m4_targets, forearm_dirs, dt):
        """m4_targets: {joint_name: rad}, retarget()'s output for THIS tick
        -- ONLY the joints backed by a currently-gated limb (retarget.py's
        own contract: "targets ... contains ONLY the joints backed by a
        currently-gated limb; everything else is the caller's ... to hold").
        NOT a full 30-joint dict -- a limb with its gate off (legs, at a
        desk, the common case) is simply ABSENT, same as direct mode sees.
        forearm_dirs: {"left": unit vector or None, "right": unit vector or
        None} in the TORSO frame, None for a side whose arm gate is
        currently off (held, not target-chased -- see retarget.py's
        per-limb gating). dt: seconds since the previous solve (mirror_node's
        measured tick period).

        Returns (out, info): `out` has exactly the SAME KEYS m4_targets had
        -- a limb whose gate is off this tick is ABSENT from m4_targets (see
        retarget.retarget's own contract) and stays absent from `out` too,
        so mirror_node's `if j not in targets: continue` holds it exactly
        like direct mode, instead of this module inventing a value for a
        joint retarget() never touched. Legs/head/torso and any non-gated
        arm that IS present are copied VERBATIM from m4_targets (byte-
        identical, not just posture-regularized close); gated arms present
        get their wrist joints (5/6/7) replaced by the QP result, 1-4
        untouched. `info` carries solve_time_s and, per gated side, the
        residual Cartesian error of the COMMANDED q against its own target
        (a sanity signal, not the A/B harness's independent metric -- see
        tools/qp_ab_harness.py for that).
        """
        t0 = time.perf_counter()
        m4_q = self.pm.q_from_dict(m4_targets)

        active = [s for s in SIDES if forearm_dirs.get(s) is not None]

        q_seed = m4_q.copy()
        if self._warm_wrist is not None:
            for side in SIDES:
                for name in self.pm.arm_joint_names[side][4:7]:  # joints 5,6,7
                    if name in self._warm_wrist:
                        q_seed[self.pm.idx_q[name]] = self._warm_wrist[name]

        self.posture_task.set_target(m4_q)
        tasks = [self.posture_task]
        targets_se3 = {}
        for side in active:
            se3 = self.pm.build_wrist_target(m4_q, forearm_dirs[side], side)
            self.frame_tasks[side].set_target(se3)
            tasks.append(self.frame_tasks[side])
            targets_se3[side] = se3

        info = {"solve_time_s": 0.0, "active_sides": list(active)}
        if not active:
            # No gated arm this tick -- nothing for the QP to do; hand M4
            # back untouched rather than running a QP with only a posture
            # task (that would just be an expensive no-op).
            info["solve_time_s"] = time.perf_counter() - t0
            return dict(m4_targets), info

        configuration = pink.Configuration(self.pm.model, self.pm.data, q_seed)
        velocity = pink.solve_ik(
            configuration, tasks, dt, solver=self.solver, damping=self.damping,
            limits=self.limits,
        )
        free_wrist_names = {n for s in active for n in self._wrist_joint_names[s]}
        v = np.array(velocity, dtype=float)
        for name in ALL_JOINTS:
            if name not in free_wrist_names:
                v[self.pm.idx_v[name]] = 0.0  # everything but the active wrists: LOCKED at M4

        q_next = configuration.integrate(v, dt)
        q_next_dict = self.pm.dict_from_q(q_next)

        # Base = m4_targets VERBATIM, not ALL_JOINTS -- m4_targets already
        # carries exactly the "only what is currently gated in" contract
        # (retarget.retarget's own docstring), so copying it wholesale is
        # what makes a gated-out limb (legs at a desk, most days) stay
        # ABSENT from `out` too, instead of a KeyError (m4_targets has no
        # entry for it) or a fabricated value (ALL_JOINTS has an entry for
        # it regardless of gating). Only the active sides' wrist joints
        # 5/6/7 -- always present in m4_targets whenever that side is
        # active, since an active side is by definition a GATED-IN arm and
        # retarget() parks 5/6/7 at 0.0 for every gated-in arm -- get
        # overwritten with the QP result.
        out = dict(m4_targets)
        for name in free_wrist_names:
            out[name] = q_next_dict[name]

        self._warm_wrist = {
            name: out[name]
            for side in active
            for name in self.pm.arm_joint_names[side][4:7]
        }

        self.pm.fk(q_next)
        for side in active:
            oMf = self.pm.data.oMf[self.pm.wrist_frame_id[side]]
            target = targets_se3[side]
            info[f"pos_err_{side}_m"] = float(
                np.linalg.norm(oMf.translation - target.translation)
            )
            info[f"rot_err_{side}_deg"] = math.degrees(
                rotation_angle(oMf.rotation.T @ target.rotation)
            )

        info["solve_time_s"] = time.perf_counter() - t0
        return out, info

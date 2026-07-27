"""qp_pose_source -- c2-qp (M5): QPMirrorPoseSource. Wraps retarget.retarget()
(unchanged -- still produces the M4 direct-copy solution for every group)
with two additions:

  1. One-Euro filtering on the LANDMARK INPUTS (shoulder/elbow/wrist, both
     sides) BEFORE retargeting -- a requirement specific to this milestone,
     distinct from mirror_node's existing per-OUTPUT-JOINT-ANGLE One-Euro
     filters (joint_limits.py's docstring explains why M4 filters angles,
     not landmarks: filtering landmarks then computing angles lets noise
     through the acos/atan2 nonlinearity). Both run here: THIS module's
     landmark filter smooths the RAW 3D points the QP's Cartesian target is
     built from, so a noisy source never hands the QP a jittery SE3 target
     frame to frame; mirror_node's existing angle-level filter still runs
     downstream on whatever this source returns, exactly as it already does
     for legs/head/torso and for direct mode. Belt and suspenders, not a
     replacement -- see mirror_node.py's docstring.
  2. qp_retarget.WristQPSolver, refining the M4 arm solution into a
     Cartesian-target-tracking one; legs/head/torso pass through verbatim
     (see WristQPSolver.solve's docstring for "verbatim, not just close").

Ported structure from retarget.MirrorPoseSource -- same node-attribute
contract (node.observation, node.mirror_mode, node.q_cmd, node.margin, the
four head/torso gains, node._lock), plus node.qp_solver (a
qp_retarget.WristQPSolver, constructed once by mirror_node.__init__ only
when mode=="qp") and node.rate_hz (used as the dt fallback on the very
first tick, before two observations exist to measure a real interval from).

Imports pink transitively (via qp_retarget, imported by mirror_node only in
qp mode) -- this module itself imports nothing from pink/pinocchio directly,
only talos_mirror's own body_track/joint_limits/retarget, so it is safe to
import under either interpreter; what actually requires /opt/mpvenv is
qp_retarget.WristQPSolver, constructed by mirror_node before this source is
ever used.
"""

from talos_mirror.body_track import (
    LEFT_ELBOW,
    LEFT_SHOULDER,
    LEFT_WRIST,
    RIGHT_ELBOW,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
)
from talos_mirror.joint_limits import OneEuro
from talos_mirror.retarget import retarget

_ARM_LANDMARK_IDS = (
    LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST,
    RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST,
)


class LandmarkFilter:
    """Three independent One-Euro filters (x, y, z) per landmark index --
    the standard way to filter a 3D point stream: components are filtered
    independently (a diagonal filter), never jointly. The OneEuro class
    itself is joint_limits.py's copy (itself copied from hand_follow.py,
    ported per the house self-contained-example rule); this class just
    fans it out over landmark index x component."""

    def __init__(self, indices, min_cutoff=1.0, beta=0.5):
        self._f = {
            (i, c): OneEuro(min_cutoff=min_cutoff, beta=beta)
            for i in indices
            for c in range(3)
        }

    def reset(self):
        for f in self._f.values():
            f.reset()

    def __call__(self, landmarks, t):
        """landmarks: {str(index): [x, y, z], ...} (the wire-format dict
        retarget._as_observation normalises to, in the TORSO-relative body
        axes body_track.py already uses). Returns a NEW dict, same shape,
        with every landmark index this filter tracks passed through its
        three per-component filters -- landmarks outside that set (legs,
        head/ears/nose, feet) pass through UNTOUCHED, since only the arm
        QP target needs them pre-filtered; every other group still runs
        M4's own (already-verified) direct-copy math on the raw values."""
        out = dict(landmarks)
        for (i, c), f in self._f.items():
            key = str(i)
            if key not in landmarks:
                continue
            v = list(landmarks[key])
            v[c] = f(float(v[c]), t)
            out[key] = v
        return out


class QPMirrorPoseSource:
    """mirror_node's pose source when mode=="qp". Same read(t) contract as
    retarget.MirrorPoseSource (returns a full joint-target dict or None),
    so mirror_node's _tick() needs no branching beyond which source object
    it was constructed with."""

    name = "body_observation_qp"

    def __init__(self, node):
        self.node = node
        self.filter = LandmarkFilter(
            _ARM_LANDMARK_IDS,
            min_cutoff=node.landmark_min_cutoff,
            beta=node.landmark_beta,
        )
        self._last_t = None

    def start(self):
        return None

    def stop(self):
        return None

    def read(self, t):
        with self.node._lock:
            obs = self.node.observation
        # obs is the wire-format dict track_node publishes (json.loads of
        # /body/observation) -- {"tracked": bool, "gates", "landmarks",
        # "basis", ...} -- retarget() normalises this shape itself via its
        # own _as_observation(); filtering happens BEFORE that call, on the
        # SAME wire-format dict, not on an already-normalised one (an
        # already-normalised dict has no "tracked" key and would silently
        # look untracked to retarget()'s own re-normalisation -- this is
        # why filtering is done here rather than by calling
        # retarget._as_observation ourselves first).
        if not obs or not obs.get("tracked") or obs.get("basis") is None \
                or obs.get("landmarks") is None:
            # Tracking lost/never acquired -- drop warm-start state so a
            # stale wrist orientation from before the dropout never seeds
            # the next real solve, mirroring mirror_node's own
            # loss_hold_sec -> HOME behaviour for the rest of the body.
            self.node.qp_solver.reset()
            self.filter.reset()
            self._last_t = None
            self.node.retarget_info = {"skipped": {"body": "not tracked"}}
            self.node.qp_info = {}
            return None

        obs = dict(obs)
        obs["landmarks"] = self.filter(obs["landmarks"], t)

        m4_targets, info = retarget(
            obs,
            mirror=self.node.mirror_mode,
            prev=self.node.q_cmd,
            margin=self.node.margin,
            head_gain_yaw=self.node.head_gain_yaw,
            head_gain_pitch=self.node.head_gain_pitch,
            torso_gain_yaw=self.node.torso_gain_yaw,
            torso_gain_pitch=self.node.torso_gain_pitch,
        )
        self.node.retarget_info = info
        if not m4_targets:
            self.node.qp_info = {}
            return None

        dt = (t - self._last_t) if self._last_t is not None else (1.0 / self.node.rate_hz)
        dt = max(dt, 1e-3)
        self._last_t = t

        out, qp_info = self.node.qp_solver.solve(m4_targets, info["forearm_dir"], dt)
        self.node.qp_info = qp_info
        return out

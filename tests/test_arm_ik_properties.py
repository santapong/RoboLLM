"""Hypothesis property suite over the canonical wall_weld arm_ik.py (pure math).

Properties (never assert joint equality — position-only 5-DOF IK is
multi-branch; assert Cartesian pose error instead):

* FK self-agreement  — the matrix FK (`fk`) and the scalar FK (`fk_tool0`) must
  agree on the tool0 point, and the tool z-axis is a unit vector.
* clamp / reachability invariants + idempotence — `clamp_target` projects any
  point onto the [R_MIN, R_MAX] shell and is a fixed point on its own output.
* solve_track reachable-target error — a target built by FK of a reachable pose
  is reached to sub-cm Cartesian error (unless the branch-jump flag is raised).
* jump-flag consistency — `info["jump"]` is True iff the returned joints really
  moved more than the jump limit from q_prev.

arm_ik is put on sys.path by tests/conftest.py.
"""
import math

import arm_ik
from hypothesis import assume, given, settings
from hypothesis import strategies as st

# solve_track is an iterative pure-python solver: on a slow shared CI runner
# a single example can trip hypothesis's default 200 ms deadline and flake.
# Correctness is bounded by the assertions, not wall time — disable deadlines.
settings.register_profile("ci", deadline=None)
settings.load_profile("ci")

_TORCH = arm_ik.TORCH
_SHOULDER_Z = arm_ik._SHOULDER_Z
_R_MAX = arm_ik._R_MAX
_R_MIN = arm_ik._R_MIN
_JUMP = arm_ik._JUMP_LIMIT

# free joints for the general FK checks
_free = st.floats(min_value=-3.0, max_value=3.0,
                  allow_nan=False, allow_infinity=False)
_joints6 = st.lists(_free, min_size=6, max_size=6)

# moderate, in-limit joints for building genuinely reachable targets
_track_joint = st.floats(min_value=-1.2, max_value=1.2,
                         allow_nan=False, allow_infinity=False)
_track6 = st.lists(_track_joint, min_size=6, max_size=6)

_NOMINAL = arm_ik.NOMINAL
# The solver regularizes hard toward NOMINAL, so its real operating regime is a
# warm start already near NOMINAL posture (the frame-to-frame teleop state).
# Deviations up to +-0.4 rad per joint span a wide working envelope.
_near_nom = st.lists(
    st.floats(min_value=-0.4, max_value=0.4,
              allow_nan=False, allow_infinity=False),
    min_size=6, max_size=6)
# a few-mm target step between camera frames
_step_mm = st.lists(
    st.floats(min_value=-0.005, max_value=0.005,
              allow_nan=False, allow_infinity=False),
    min_size=3, max_size=3)


def _shell_dist(p):
    return math.sqrt(p[0] ** 2 + p[1] ** 2 + (p[2] - _SHOULDER_Z) ** 2)


@given(_joints6)
def test_fk_self_agreement(q):
    tip, z = arm_ik.fk(q)
    # tool0 recovered from the matrix FK must equal the independent scalar FK
    tool0 = [tip[i] - _TORCH * z[i] for i in range(3)]
    px, py, pz = arm_ik.fk_tool0(q)
    assert math.isclose(tool0[0], px, abs_tol=1e-9)
    assert math.isclose(tool0[1], py, abs_tol=1e-9)
    assert math.isclose(tool0[2], pz, abs_tol=1e-9)
    # the reported tool z-axis is a unit vector
    assert math.isclose(math.sqrt(sum(c * c for c in z)), 1.0, abs_tol=1e-9)


@given(st.lists(st.floats(min_value=-1.0, max_value=1.0,
                          allow_nan=False, allow_infinity=False),
                min_size=3, max_size=3))
def test_clamp_reachability_and_idempotence(t):
    c1, was1 = arm_ik.clamp_target(t)
    d1 = _shell_dist(c1)
    assert _R_MIN - 1e-9 <= d1 <= _R_MAX + 1e-9
    # a point already inside the shell is passed through unflagged
    d0 = _shell_dist(t)
    if _R_MIN <= d0 <= _R_MAX:
        assert was1 is False
        assert math.isclose(c1[0], t[0], abs_tol=1e-12)
        assert math.isclose(c1[1], t[1], abs_tol=1e-12)
        assert math.isclose(c1[2], t[2], abs_tol=1e-12)
    # idempotence: clamping the clamped point does not move it
    c2, _ = arm_ik.clamp_target(c1)
    for a, b in zip(c1, c2):
        assert math.isclose(a, b, abs_tol=1e-9)


@given(_near_nom, _step_mm)
def test_solve_track_reaches_reachable_target(dev, step):
    # warm start = previous frame's solution, near NOMINAL posture
    q_prev = [_NOMINAL[i] + dev[i] for i in range(6)]
    # target = where the tool0 is now, nudged a few mm (the next camera frame)
    here = arm_ik.fk_tool0(q_prev)
    target = [here[i] + step[i] for i in range(3)]
    # keep it strictly inside the reachable shell so the clamp is a no-op and
    # "reachable" genuinely holds
    assume(_R_MIN + 1e-3 <= _shell_dist(target) <= _R_MAX - 1e-3)
    q, info = arm_ik.solve_track(target, q_prev)
    px, py, pz = arm_ik.fk_tool0(q)
    err = math.sqrt((px - target[0]) ** 2
                    + (py - target[1]) ** 2
                    + (pz - target[2]) ** 2)
    # the reported error is the true Cartesian pose error (never joint equality)
    assert math.isclose(err, info["err_m"], rel_tol=1e-6, abs_tol=1e-9)
    # in its operating regime the solver reaches sub-millimeter and never needs
    # a branch flip
    assert info["jump"] is False
    assert err < 1e-3


@given(_track6, st.lists(st.floats(min_value=-2.0, max_value=2.0,
                                   allow_nan=False, allow_infinity=False),
                         min_size=6, max_size=6))
def test_jump_flag_consistency(q_true, q_prev):
    target = list(arm_ik.fk_tool0(q_true))
    assume(_R_MIN + 1e-3 <= _shell_dist(target) <= _R_MAX - 1e-3)
    q, info = arm_ik.solve_track(target, q_prev)
    actual_jump = any(abs(q[i] - q_prev[i]) > _JUMP for i in range(6))
    assert info["jump"] == actual_jump

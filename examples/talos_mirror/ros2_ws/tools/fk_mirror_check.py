#!/usr/bin/env python3
"""INDEPENDENT adversarial verification of the M4 retarget vs MoveIt /compute_fk.

This is NOT retarget_bench. retarget_bench's discriminating cases assert on
arm_dirs()/leg_dirs() -- the SOLVER'S OWN forward kinematics -- so a solver
whose internal FK agrees with its own solve (self-consistent) but whose
geometry model is WRONG vs the real URDF would pass it. This project has
shipped exactly that failure once (internally exact to 0.27 deg, 31 deg wrong
vs /compute_fk).

Here the ONLY things trusted are:
  1. retarget()  -- the code UNDER TEST (imported from live source).
  2. MoveIt /compute_fk -- the INDEPENDENT oracle (the real URDF's kinematics).
arm_dirs()/leg_dirs() are never called. The loop is fully end-to-end:

    synthetic body observation
        -> retarget()                      (real code path, real mirror law)
        -> 30-joint command vector
        -> /compute_fk                     (independent kinematics)
        -> cartesian tip / segment positions
        -> assert against THE MIRROR LAW    (forward=+x, correct side, knee bend)

Two classes of check per pose:
  (A) SEMANTIC (mirror-law): the wrist/foot/head tip moved the correct forward
      direction (+x), to the correct SIDE of the room (Δy from rest has the
      sign THE MIRROR LAW demands), knee bent forward by the right amount,
      plausible magnitude.
  (B) QUANTITATIVE (wrong-model tripwire): the cartesian segment direction
      /compute_fk reports for the robot limb matches the direction the human
      pose (after the mirror law) demanded, to within the solver's own
      residual. A 31-deg-wrong model fails (B) even while staying internally
      self-consistent.

Every discriminating pose is also re-run through retarget()'s mirror_fn hook
with DELIBERATELY WRONG formulas (defined locally here, not imported), and the
harness must SEE its own assertions flip to FAIL -- proof the checks discriminate.

Side check uses Δy FROM THE HOME POSE, not absolute tip y: a limb's absolute y
is dominated by its fixed shoulder/hip base offset, which is NOT what the
mirror law governs. The reach's LATERAL DISPLACEMENT is. (This distinction is
exactly what lets the check catch the swap-only mutation, which reaches to the
wrong side but from the same base.)

Frames: arms/head are queried in torso_2_link (the arm/head base, matching
retarget's torso-relative math); legs in base_link (the pinned pelvis). +x is
proven to be physical forward by an FK anchor probe (toe frame ahead of heel).

Run inside the running mock-bringup container on its DDS domain, e.g.:
  docker exec talosviewer bash -c 'source /opt/ros/jazzy/setup.bash;
      source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=99;
      python3 /ros2_ws/tools/fk_mirror_check.py'
Emits RESULT:{json} and exits non-zero on any failure.
"""

import json
import math
import sys

_ROOT = __file__.rsplit("/tools/", 1)[0]
sys.path.insert(0, _ROOT + "/src/talos_mirror")

# Interface (NOT under test): landmark index constants.
from talos_mirror.body_track import (  # noqa: E402
    LEFT_ANKLE, LEFT_EAR, LEFT_ELBOW, LEFT_FOOT_INDEX, LEFT_HEEL, LEFT_HIP,
    LEFT_KNEE, LEFT_SHOULDER, LEFT_WRIST, NOSE, RIGHT_ANKLE, RIGHT_EAR,
    RIGHT_ELBOW, RIGHT_FOOT_INDEX, RIGHT_HEEL, RIGHT_HIP, RIGHT_KNEE,
    RIGHT_SHOULDER, RIGHT_WRIST,
)
# UNDER TEST.
from talos_mirror.retarget import retarget, mirror_vec  # noqa: E402
from talos_mirror.talos_config import home_joint_state  # noqa: E402

import rclpy  # noqa: E402
from moveit_msgs.srv import GetPositionFK  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Header  # noqa: E402


# --------------------------------------------------------------- vector math
def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return (0.0, 0.0, 0.0) if n < 1e-12 else (a[0] / n, a[1] / n, a[2] / n)


def ang_deg(u, v):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(u), unit(v))))))


def quat_rotate(q, v):
    """Rotate vector v by quaternion q=(x,y,z,w). Independent of retarget."""
    x, y, z, w = q
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - z * w); r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w); r11 = 1 - 2 * (x * x + z * z); r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w); r21 = 2 * (y * z + x * w); r22 = 1 - 2 * (x * x + y * y)
    return (r00 * v[0] + r01 * v[1] + r02 * v[2],
            r10 * v[0] + r11 * v[1] + r12 * v[2],
            r20 * v[0] + r21 * v[1] + r22 * v[2])


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


# --------------------------------------------------------------- deliberately WRONG mirror formulas
# Defined HERE, locally -- not imported from retarget -- so the mutation test
# does not lean on the module's own provided mutants. Injected through the real
# retarget(mirror_fn=...) code path.
def mut_negate_x_too(v):
    """The historic bug: negate x as well as y. Forward becomes backward."""
    return (-v[0], -v[1], v[2])


def mut_swap_only(v):
    """Sides still swap, nothing is negated. Left/right of the room is wrong."""
    return (v[0], v[1], v[2])


def mut_negate_x_only(v):
    """A third, home-grown wrong formula: negate x, keep y. Both axes wrong."""
    return (-v[0], v[1], v[2])


MUTATIONS = {
    "negate_x_too": mut_negate_x_too,
    "swap_only": mut_swap_only,
    "negate_x_only": mut_negate_x_only,
}


# --------------------------------------------------------------- observation builder (independent)
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

# Relaxed standing pose, body frame (x fwd, y left, z up), identity torso basis.
NEUTRAL = {
    LEFT_SHOULDER: (0.0, 0.20, 0.50), RIGHT_SHOULDER: (0.0, -0.20, 0.50),
    LEFT_ELBOW: (0.0, 0.22, 0.22), LEFT_WRIST: (0.0, 0.22, -0.06),
    RIGHT_ELBOW: (0.0, -0.22, 0.22), RIGHT_WRIST: (0.0, -0.22, -0.06),
    LEFT_HIP: (0.0, 0.10, 0.0), RIGHT_HIP: (0.0, -0.10, 0.0),
    LEFT_KNEE: (0.0, 0.10, -0.42), LEFT_ANKLE: (0.0, 0.10, -0.84),
    LEFT_HEEL: (-0.06, 0.10, -0.88), LEFT_FOOT_INDEX: (0.14, 0.10, -0.88),
    RIGHT_KNEE: (0.0, -0.10, -0.42), RIGHT_ANKLE: (0.0, -0.10, -0.84),
    RIGHT_HEEL: (-0.06, -0.10, -0.88), RIGHT_FOOT_INDEX: (0.14, -0.10, -0.88),
    NOSE: (0.15, 0.0, 0.66), LEFT_EAR: (0.04, 0.08, 0.64), RIGHT_EAR: (0.04, -0.08, 0.64),
}


def build_obs(overrides, basis=None):
    p = dict(NEUTRAL)
    p.update(overrides)
    return {
        "t": 0.0, "tracked": True, "frame_source": "fkcheck",
        "gates": {g: True for g in ("arm_l", "arm_r", "leg_l", "leg_r", "head")},
        "landmarks": {str(i): list(v) for i, v in p.items()},
        "basis": [list(r) for r in (basis or IDENTITY)],
    }


HOME = home_joint_state()


def full_joint_vector(targets, pin_torso=True):
    """30-joint dict = HOME overridden by retarget targets. Torso pinned to 0
    so arm links (child of torso_2_link) inherit no torso rotation."""
    q = dict(HOME)
    q.update(targets)
    if pin_torso:
        q["torso_1_joint"] = 0.0
        q["torso_2_joint"] = 0.0
    return q


# --------------------------------------------------------------- FK client
class FK(Node):
    def __init__(self):
        super().__init__("fk_mirror_check")
        self.cli = self.create_client(GetPositionFK, "/compute_fk")

    def wait(self, timeout=25.0):
        return self.cli.wait_for_service(timeout_sec=timeout)

    def compute(self, joint_dict, links, frame_id):
        """{link: (pos_tuple, quat_tuple)} for `links` in `frame_id`."""
        req = GetPositionFK.Request()
        req.header = Header(frame_id=frame_id)
        req.fk_link_names = list(links)
        req.robot_state.joint_state = JointState(
            name=list(joint_dict.keys()), position=list(joint_dict.values())
        )
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"/compute_fk failed (error_code={code}) for {links} in {frame_id}")
        out = {}
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            out[name] = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        return out


FAILURES = []
RECORDS = []


def check(ok, label, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}{('  --  ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(label)
    return ok


# =========================================================================
# ANCHOR PROBE: prove +x is physical forward, and torso_2_link is x-aligned
# with base_link at home, from /compute_fk alone (retarget-independent).
# =========================================================================
def anchor_probe(fk):
    print("\nANCHOR PROBE (retarget-independent: establish robot forward = +x)")
    q = dict(HOME)
    links = ["left_foot_upper_left", "left_foot_lower_left", "left_sole_link", "torso_2_link"]
    r = fk.compute(q, links, "base_link")
    toe = r["left_foot_upper_left"][0]
    heel = r["left_foot_lower_left"][0]
    sole = r["left_sole_link"][0]
    check(toe[0] > heel[0] + 0.02,
          "physical forward is +x in base_link (toe frame ahead of heel frame)",
          f"toe.x={toe[0]:+.4f} heel.x={heel[0]:+.4f} sole.x={sole[0]:+.4f}")
    q_torso = r["torso_2_link"][1]
    tx = quat_rotate(q_torso, (1.0, 0.0, 0.0))
    ty = quat_rotate(q_torso, (0.0, 1.0, 0.0))
    check(tx[0] > 0.98 and ty[1] > 0.98,
          "torso_2_link axes are x-forward / y-left aligned with base_link at home",
          f"torso_2 local +x->{tuple(round(v,3) for v in tx)} +y->{tuple(round(v,3) for v in ty)}")
    return {"toe_x": round(toe[0], 4), "heel_x": round(heel[0], 4)}


# =========================================================================
# DISCRIMINATING POSES (forward/asymmetric only -- NO sideways raises).
# Laterals boosted so the SIDE check crisply separates the swap-only mutant.
# =========================================================================
ARM_CASES = [
    ("forward reach LEFT arm (diagonal fwd+own-left)", "left",
     {LEFT_ELBOW: (0.34, 0.42, 0.44), LEFT_WRIST: (0.58, 0.60, 0.40)}),
    ("forward reach RIGHT arm (diagonal fwd+own-right)", "right",
     {RIGHT_ELBOW: (0.34, -0.42, 0.44), RIGHT_WRIST: (0.58, -0.60, 0.40)}),
]

# plain forward step: leg stays nearly straight (knee flex small) -- the
# discriminators here are foot +x, correct side, shin forward.
LEG_CASES = [
    ("forward step LEFT leg (diagonal fwd+own-left)", "left",
     {LEFT_KNEE: (0.26, 0.26, -0.32), LEFT_ANKLE: (0.52, 0.26, -0.58),
      LEFT_HEEL: (0.46, 0.26, -0.62), LEFT_FOOT_INDEX: (0.64, 0.26, -0.62)}),
    ("forward step RIGHT leg (diagonal fwd+own-right)", "right",
     {RIGHT_KNEE: (0.26, -0.26, -0.32), RIGHT_ANKLE: (0.52, -0.26, -0.58),
      RIGHT_HEEL: (0.46, -0.26, -0.62), RIGHT_FOOT_INDEX: (0.64, -0.26, -0.62)}),
]

# MARCH / high knee: thigh lifted forward+up, shin hangs down -- genuinely
# flexes the knee, for the knee-bend-direction check.
MARCH_CASES = [
    ("march LEFT leg (knee lifted, shin down)", "left",
     {LEFT_KNEE: (0.34, 0.12, -0.08), LEFT_ANKLE: (0.22, 0.12, -0.48),
      LEFT_HEEL: (0.16, 0.12, -0.50), LEFT_FOOT_INDEX: (0.34, 0.12, -0.52)}),
    ("march RIGHT leg (knee lifted, shin down)", "right",
     {RIGHT_KNEE: (0.34, -0.12, -0.08), RIGHT_ANKLE: (0.22, -0.12, -0.48),
      RIGHT_HEEL: (0.16, -0.12, -0.50), RIGHT_FOOT_INDEX: (0.34, -0.12, -0.52)}),
]

ARM_SEG = {"left": ("arm_left_2_link", "arm_left_4_link", "arm_left_7_link"),
           "right": ("arm_right_2_link", "arm_right_4_link", "arm_right_7_link")}
ARM_TIP = {"left": "wrist_left_ft_tool_link", "right": "wrist_right_ft_tool_link"}
LEG_SEG = {"left": ("leg_left_1_link", "leg_left_4_link", "leg_left_6_link"),
           "right": ("leg_right_1_link", "leg_right_4_link", "leg_right_6_link")}
LEG_TIP = {"left": "left_sole_link", "right": "right_sole_link"}
KNEE_JOINT = {"left": "leg_left_4_joint", "right": "leg_right_4_joint"}

FWD_MIN = 0.10          # m, tip must move at least this far forward
SIDE_MIN = 0.05         # m, tip must move at least this far to the correct side
DIR_TOL_EXTRA = 2.0     # deg, allowed on top of the solver's own residual
KNEE_ANGLE_TOL = 8.0    # deg, robot cartesian knee flex vs human knee flex


def robot_side(human_side):
    return "right" if human_side == "left" else "left"


def want_dy_negative(rs):
    """Robot RIGHT limb should move to the robot's own right (Δy<0)."""
    return rs == "right"


def run_arm_case(fk, label, human_side, overrides, mirror_fn, home_tips):
    rs = robot_side(human_side)
    obs = build_obs(overrides)
    sh, el, wr = (overrides.get(k, NEUTRAL[k]) for k in
                  ((LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST) if human_side == "left"
                   else (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST)))
    a_int = mirror_fn(unit(sub(el, sh)))
    b_int = mirror_fn(unit(sub(wr, el)))

    targets, info = retarget(obs, mirror=True, mirror_fn=mirror_fn)
    err = info["err_deg"].get(f"arm_{rs}", 999.0)
    q = full_joint_vector(targets)

    b_up, b_el, b_wr = ARM_SEG[rs]
    r_t2 = fk.compute(q, [b_up, b_el, b_wr], "torso_2_link")
    ua = unit(sub(r_t2[b_el][0], r_t2[b_up][0]))
    fa = unit(sub(r_t2[b_wr][0], r_t2[b_el][0]))
    d_ua = ang_deg(ua, a_int)
    d_fa = ang_deg(fa, b_int)

    tip = ARM_TIP[rs]
    tip_pos = fk.compute(q, [tip], "base_link")[tip][0]
    dx = tip_pos[0] - home_tips[tip][0]
    dy = tip_pos[1] - home_tips[tip][1]

    fwd_ok = dx > FWD_MIN
    side_ok = (dy < -SIDE_MIN) if want_dy_negative(rs) else (dy > SIDE_MIN)
    dir_ok = (d_ua <= err + DIR_TOL_EXTRA) and (d_fa <= err + DIR_TOL_EXTRA)
    plausible = norm(sub(tip_pos, home_tips[tip])) < 1.2

    detail = (f"tip Δx={dx:+.3f} Δy={dy:+.3f} (want Δx>{FWD_MIN}, "
              f"Δy{'<0' if want_dy_negative(rs) else '>0'}) | seg dir err ua={d_ua:.2f} "
              f"fa={d_fa:.2f} (resid {err:.2f})")
    return {"label": label, "robot_side": rs, "solver_resid_deg": round(err, 3),
            "tip_dx": round(dx, 4), "tip_dy": round(dy, 4),
            "seg_ua_err_deg": round(d_ua, 3), "seg_fa_err_deg": round(d_fa, 3),
            "fwd_ok": fwd_ok, "side_ok": side_ok, "dir_ok": dir_ok, "plausible": plausible,
            "semantic_ok": fwd_ok and side_ok and plausible, "quant_ok": dir_ok,
            "detail": detail}


def run_leg_case(fk, label, human_side, overrides, mirror_fn, home_tips, check_knee_match=True):
    rs = robot_side(human_side)
    obs = build_obs(overrides)
    hip, knee, ankle = (overrides.get(k, NEUTRAL[k]) for k in
                        ((LEFT_HIP, LEFT_KNEE, LEFT_ANKLE) if human_side == "left"
                         else (RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE)))
    a_h = unit(sub(knee, hip))     # human thigh dir
    b_h = unit(sub(ankle, knee))   # human shin dir
    a_int = mirror_fn(a_h)
    b_int = mirror_fn(b_h)
    human_knee_deg = ang_deg(a_h, b_h)   # mirror preserves angles

    targets, info = retarget(obs, mirror=True, mirror_fn=mirror_fn)
    err = info["err_deg"].get(f"leg_{rs}", 999.0)
    q = full_joint_vector(targets)
    knee_ang = targets.get(KNEE_JOINT[rs], 0.0)

    b_hip, b_knee, b_ank = LEG_SEG[rs]
    tip = LEG_TIP[rs]
    r = fk.compute(q, [b_hip, b_knee, b_ank, tip], "base_link")
    ta = unit(sub(r[b_knee][0], r[b_hip][0]))   # robot thigh dir (cartesian)
    sa = unit(sub(r[b_ank][0], r[b_knee][0]))   # robot shin dir (cartesian)
    d_ta = ang_deg(ta, a_int)
    d_sa = ang_deg(sa, b_int)
    robot_knee_deg = ang_deg(ta, sa)

    tip_pos = r[tip][0]
    dx = tip_pos[0] - home_tips[tip][0]
    dy = tip_pos[1] - home_tips[tip][1]

    fwd_ok = dx > FWD_MIN
    side_ok = (dy < -SIDE_MIN) if want_dy_negative(rs) else (dy > SIDE_MIN)
    shin_fwd = sa[0] > 0.0
    dir_ok = (d_ta <= err + DIR_TOL_EXTRA) and (d_sa <= err + DIR_TOL_EXTRA)
    knee_match = abs(robot_knee_deg - human_knee_deg) <= KNEE_ANGLE_TOL
    plausible = norm(sub(tip_pos, home_tips[tip])) < 1.0

    semantic = fwd_ok and side_ok and shin_fwd and plausible
    if check_knee_match:
        semantic = semantic and knee_match

    detail = (f"foot Δx={dx:+.3f} Δy={dy:+.3f} shin.x={sa[0]:+.3f} | "
              f"knee flex robot={robot_knee_deg:.1f} human={human_knee_deg:.1f} deg | "
              f"seg dir err ta={d_ta:.2f} sa={d_sa:.2f} (resid {err:.2f})")
    return {"label": label, "robot_side": rs, "solver_resid_deg": round(err, 3),
            "foot_dx": round(dx, 4), "foot_dy": round(dy, 4), "shin_x": round(sa[0], 4),
            "knee_joint": round(knee_ang, 4),
            "robot_knee_deg": round(robot_knee_deg, 2), "human_knee_deg": round(human_knee_deg, 2),
            "seg_ta_err_deg": round(d_ta, 3), "seg_sa_err_deg": round(d_sa, 3),
            "fwd_ok": fwd_ok, "side_ok": side_ok, "shin_fwd": shin_fwd,
            "knee_match": knee_match, "dir_ok": dir_ok, "plausible": plausible,
            "semantic_ok": semantic, "quant_ok": dir_ok, "detail": detail}


def run_march_case(fk, label, human_side, overrides, mirror_fn, home_tips):
    """Knee-bend-direction check: the knee genuinely flexes; verify the robot's
    cartesian knee flex matches the human's (so the knee maps a human bend to a
    real robot bend, not a parked/clamped joint), the thigh leads FORWARD, and
    the foot ends up BELOW and BEHIND the knee (a physical forward knee fold)."""
    rs = robot_side(human_side)
    obs = build_obs(overrides)
    hip, knee, ankle = (overrides.get(k, NEUTRAL[k]) for k in
                        ((LEFT_HIP, LEFT_KNEE, LEFT_ANKLE) if human_side == "left"
                         else (RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE)))
    a_h = unit(sub(knee, hip)); b_h = unit(sub(ankle, knee))
    a_int = mirror_fn(a_h); b_int = mirror_fn(b_h)
    human_knee_deg = ang_deg(a_h, b_h)

    targets, info = retarget(obs, mirror=True, mirror_fn=mirror_fn)
    err = info["err_deg"].get(f"leg_{rs}", 999.0)
    knee_ang = targets.get(KNEE_JOINT[rs], 0.0)
    q = full_joint_vector(targets)

    b_hip, b_knee, b_ank = LEG_SEG[rs]
    r = fk.compute(q, [b_hip, b_knee, b_ank], "base_link")
    knee_pos, ank_pos = r[b_knee][0], r[b_ank][0]
    ta = unit(sub(knee_pos, r[b_hip][0]))
    sa = unit(sub(ank_pos, knee_pos))
    robot_knee_deg = ang_deg(ta, sa)
    d_ta = ang_deg(ta, a_int); d_sa = ang_deg(sa, b_int)

    knee_bent = knee_ang > 0.5                       # joint genuinely flexed
    knee_match = abs(robot_knee_deg - human_knee_deg) <= KNEE_ANGLE_TOL
    thigh_fwd = ta[0] > 0.1                           # thigh leads forward
    foot_below_knee = ank_pos[2] < knee_pos[2] - 0.1  # shin hangs down
    foot_behind_knee = ank_pos[0] < knee_pos[0]       # forward fold: foot behind knee
    semantic = knee_bent and knee_match and thigh_fwd and foot_below_knee and foot_behind_knee
    detail = (f"knee_joint={knee_ang:.3f} | knee flex robot={robot_knee_deg:.1f} "
              f"human={human_knee_deg:.1f} deg | thigh.x={ta[0]:+.3f} "
              f"foot vs knee: Δz={ank_pos[2]-knee_pos[2]:+.3f} Δx={ank_pos[0]-knee_pos[0]:+.3f} "
              f"| seg err ta={d_ta:.2f} sa={d_sa:.2f} (resid {err:.2f})")
    return {"label": label, "robot_side": rs, "knee_joint": round(knee_ang, 4),
            "robot_knee_deg": round(robot_knee_deg, 2), "human_knee_deg": round(human_knee_deg, 2),
            "solver_resid_deg": round(err, 3),
            "knee_bent": knee_bent, "knee_match": knee_match, "thigh_fwd": thigh_fwd,
            "foot_below_knee": foot_below_knee, "foot_behind_knee": foot_behind_knee,
            "semantic_ok": semantic, "detail": detail}


HEAD_TOL = 12.0   # deg, gaze window around THE MIRROR LAW's own reference gaze


def head_gaze(fk, mirror_fn, home_head):
    """Cartesian gaze direction of head_2_link (torso_2 frame) for the
    head-turn pose. gaze_local is calibrated at HOME (head faces +x), so this
    reads the ACTUAL orientation /compute_fk produces, not a joint value."""
    overrides = {NOSE: (0.34, 0.34, 0.60), LEFT_EAR: (0.05, 0.12, 0.62),
                 RIGHT_EAR: (0.20, -0.04, 0.63)}
    obs = build_obs(overrides)
    targets, _info = retarget(obs, mirror=True, mirror_fn=mirror_fn)
    q = full_joint_vector(targets)
    q_pose = fk.compute(q, ["head_2_link"], "torso_2_link")["head_2_link"][1]
    gaze_local = quat_rotate(quat_conj(home_head[1]), (1.0, 0.0, 0.0))
    return quat_rotate(q_pose, gaze_local)


def run_head_case(fk, mirror_fn, home_head, ref_gaze):
    """Human looks toward their OWN LEFT (and a little down). Mirror law: robot
    head gazes toward its OWN RIGHT (gaze.y < 0), forward (gaze.x > 0), and --
    the tight discriminator -- within HEAD_TOL of the mirror-law reference gaze.

    The window (not a bare sign check) is REQUIRED because the head-yaw joint
    clamps at +-75 deg: the negate-x bug over-rotates the yaw into that clamp,
    keeping the SIGN (still looks right) but changing the MAGNITUDE (gaze x
    drops 0.725->0.307). Only a tight window around the correct value flips.
    ref_gaze is the mirror_vec gaze; for the reference run itself the angle is
    0 by construction, so this never falsely fails the correct case."""
    gaze = head_gaze(fk, mirror_fn, home_head)
    yaw_ok = gaze[1] < -0.05
    fwd_ok = gaze[0] > 0.3
    off = ang_deg(gaze, ref_gaze) if ref_gaze is not None else 0.0
    close = off < HEAD_TOL
    detail = (f"gaze dir in torso_2={tuple(round(v,3) for v in gaze)} "
              f"(want y<0, x>0, within {HEAD_TOL:.0f} deg of mirror-law ref; off={off:.1f})")
    return {"label": "head turn (human looks own-left)", "gaze": [round(v, 4) for v in gaze],
            "yaw_ok": yaw_ok, "fwd_ok": fwd_ok, "gaze_off_ref_deg": round(off, 2),
            "semantic_ok": yaw_ok and fwd_ok and close, "detail": detail}


def run_asym_case(fk, home_tips):
    """ASYMMETRIC combination: human LEFT arm reaches forward WHILE human RIGHT
    leg steps forward. Mirror law: robot RIGHT arm forward + robot LEFT leg
    forward, each on the correct side, no cross-talk."""
    overrides = {
        LEFT_ELBOW: (0.34, 0.42, 0.44), LEFT_WRIST: (0.58, 0.60, 0.40),
        RIGHT_KNEE: (0.26, -0.26, -0.32), RIGHT_ANKLE: (0.52, -0.26, -0.58),
        RIGHT_HEEL: (0.46, -0.26, -0.62), RIGHT_FOOT_INDEX: (0.64, -0.26, -0.62),
    }
    obs = build_obs(overrides)
    targets, info = retarget(obs, mirror=True, mirror_fn=mirror_vec)
    q = full_joint_vector(targets)
    arm_tip, leg_tip = ARM_TIP["right"], LEG_TIP["left"]
    r = fk.compute(q, [arm_tip, leg_tip], "base_link")
    a_dx = r[arm_tip][0][0] - home_tips[arm_tip][0]
    a_dy = r[arm_tip][0][1] - home_tips[arm_tip][1]
    l_dx = r[leg_tip][0][0] - home_tips[leg_tip][0]
    l_dy = r[leg_tip][0][1] - home_tips[leg_tip][1]
    ok = a_dx > FWD_MIN and a_dy < -SIDE_MIN and l_dx > FWD_MIN and l_dy > SIDE_MIN
    detail = (f"robot RIGHT arm tip Δx={a_dx:+.3f} Δy={a_dy:+.3f} (want Δx>0,Δy<0); "
              f"robot LEFT foot Δx={l_dx:+.3f} Δy={l_dy:+.3f} (want Δx>0,Δy>0)")
    return {"label": "asymmetric: L arm reach + R leg step",
            "arm_dx": round(a_dx, 4), "arm_dy": round(a_dy, 4),
            "leg_dx": round(l_dx, 4), "leg_dy": round(l_dy, 4),
            "semantic_ok": ok, "detail": detail,
            "arm_resid": info["err_deg"].get("arm_right"),
            "leg_resid": info["err_deg"].get("leg_left")}


def main():
    rclpy.init()
    fk = FK()
    print("=" * 78)
    print("  INDEPENDENT M4 retarget vs MoveIt /compute_fk  (adversarial cartesian check)")
    print("=" * 78)
    if not fk.wait():
        check(False, "/compute_fk available", "is the mock bringup (talosviewer) running on this domain?")
        _finish()

    anchor = anchor_probe(fk)

    home_tip_links = list(ARM_TIP.values()) + list(LEG_TIP.values())
    home_tips = {k: v[0] for k, v in fk.compute(dict(HOME), home_tip_links, "base_link").items()}
    home_head = fk.compute(dict(HOME), ["head_2_link"], "torso_2_link")["head_2_link"]
    print("\nhome tips (base_link): " +
          "  ".join(f"{k}=({v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f})" for k, v in home_tips.items()))

    print("\nDISCRIMINATING POSES under THE MIRROR LAW (mirror_vec), checked vs /compute_fk")
    correct = {}
    for label, hs, ov in ARM_CASES:
        rec = run_arm_case(fk, label, hs, ov, mirror_vec, home_tips)
        correct[label] = rec; RECORDS.append(rec)
        check(rec["semantic_ok"], f"{label}: SEMANTIC (fwd +x, correct side, plausible)", rec["detail"])
        check(rec["quant_ok"], f"{label}: QUANTITATIVE (compute_fk seg dir == intended within resid)",
              f"ua_err={rec['seg_ua_err_deg']} fa_err={rec['seg_fa_err_deg']} vs resid {rec['solver_resid_deg']}")
    for label, hs, ov in LEG_CASES:
        rec = run_leg_case(fk, label, hs, ov, mirror_vec, home_tips)
        correct[label] = rec; RECORDS.append(rec)
        check(rec["semantic_ok"], f"{label}: SEMANTIC (foot +x, correct side, shin fwd, knee flex matches)", rec["detail"])
        check(rec["quant_ok"], f"{label}: QUANTITATIVE (compute_fk seg dir == intended within resid)",
              f"ta_err={rec['seg_ta_err_deg']} sa_err={rec['seg_sa_err_deg']} vs resid {rec['solver_resid_deg']}")
    for label, hs, ov in MARCH_CASES:
        rec = run_march_case(fk, label, hs, ov, mirror_vec, home_tips)
        correct[label] = rec; RECORDS.append(rec)
        check(rec["semantic_ok"], f"{label}: KNEE BEND (flexed, robot flex==human, thigh fwd, forward fold)", rec["detail"])

    ref_gaze = head_gaze(fk, mirror_vec, home_head)
    head_rec = run_head_case(fk, mirror_vec, home_head, ref_gaze)
    correct[head_rec["label"]] = head_rec; RECORDS.append(head_rec)
    check(head_rec["semantic_ok"], "head turn: gaze flips to robot's own right (cartesian)", head_rec["detail"])

    asym_rec = run_asym_case(fk, home_tips)
    RECORDS.append(asym_rec)
    check(asym_rec["semantic_ok"], "asymmetric combo: both limbs correct, no cross-talk", asym_rec["detail"])

    # ------------------------------------------------ MUTATION KILL (self-test of the harness)
    print("\nMUTATION KILL: inject WRONG formulas through retarget(mirror_fn=...); "
          "my OWN assertions MUST flip to FAIL on every discriminating pose")
    mut_tally = {m: {"killed": 0, "escaped": 0} for m in MUTATIONS}
    disc = ([("arm", l, hs, ov) for l, hs, ov in ARM_CASES] +
            [("leg", l, hs, ov) for l, hs, ov in LEG_CASES] +
            [("head", None, None, None)])
    for mname, mfn in MUTATIONS.items():
        for kind, label, hs, ov in disc:
            if kind == "arm":
                rec = run_arm_case(fk, label, hs, ov, mfn, home_tips)
            elif kind == "leg":
                rec = run_leg_case(fk, label, hs, ov, mfn, home_tips, check_knee_match=False)
            else:
                rec = run_head_case(fk, mfn, home_head, ref_gaze)
                label = rec["label"]
            base_ok = correct[label]["semantic_ok"]
            killed = base_ok and not rec["semantic_ok"]
            mut_tally[mname]["killed" if killed else "escaped"] += 1
            print(f"  [{'PASS' if killed else 'FAIL'}] {mname:14s} x {label[:36]:36s} -> "
                  f"{'CAUGHT' if killed else 'ESCAPED'}  ({rec['detail']})")
            if not killed:
                FAILURES.append(f"mutation {mname} ESCAPED on {label}")

    total_killed = sum(t["killed"] for t in mut_tally.values())
    total = sum(t["killed"] + t["escaped"] for t in mut_tally.values())
    check(total_killed == total,
          "every injected wrong formula is caught by MY cartesian checks on every discriminating pose",
          f"{total_killed}/{total} mutation-runs caught")

    _finish({"anchor": anchor, "mutation_tally": mut_tally,
             "home_tips": {k: [round(x, 4) for x in v] for k, v in home_tips.items()}})


def _finish(extra=None):
    print("\n" + "=" * 78)
    if FAILURES:
        print(f"  FAILED -- {len(FAILURES)} check(s):")
        for f in FAILURES:
            print(f"    - {f}")
    else:
        print("  ALL CHECKS PASSED")
    print("=" * 78)
    payload = {"passed": not FAILURES, "failures": FAILURES[:30], "records": RECORDS}
    if extra:
        payload.update(extra)
    print("RESULT:" + json.dumps(payload, default=str))
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(1 if FAILURES else 0)


def run_live():
    """Belt-and-suspenders: literally COMMAND the robot (publish the retargeted
    JointTrajectory to the six controllers), wait for the mock hardware to echo
    it back on /joint_states, then /compute_fk the ACTUAL reported state and
    re-check the mirror law. Confirms the whole command path, not just FK of the
    target angles. Needs the mock bringup running (talosviewer)."""
    import time
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from talos_mirror.talos_config import CONTROLLER_TOPICS, GROUP_JOINTS

    rclpy.init()
    node = Node("fk_mirror_live")
    fk = FK.__new__(FK)          # reuse compute(); attach a client on `node`
    Node.__init__(fk, "fk_mirror_live_fk")
    fk.cli = fk.create_client(GetPositionFK, "/compute_fk")
    fk.wait()

    state = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: state.update(zip(m.name, m.position)), 10)
    pubs = {g: node.create_publisher(JointTrajectory, t, 10) for g, t in CONTROLLER_TOPICS.items()}
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    home_tips = {k: v[0] for k, v in
                 fk.compute(dict(HOME), list(ARM_TIP.values()) + list(LEG_TIP.values()), "base_link").items()}

    print("=" * 78)
    print("  LIVE: command the six controllers, read /joint_states, /compute_fk the result")
    print("=" * 78)

    def command_and_settle(targets):
        q = full_joint_vector(targets)
        for _ in range(60):      # ~3 s; publish repeatedly, mock JTC interpolates
            for g, joints in GROUP_JOINTS.items():
                msg = JointTrajectory(joint_names=list(joints))
                pt = JointTrajectoryPoint(positions=[q[j] for j in joints])
                pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
                msg.points = [pt]
                pubs[g].publish(msg)
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.05)
        # settle error between commanded and reported
        err = max(abs(state.get(j, q[j]) - q[j]) for j in q) if state else 9.9
        return dict(state), err

    cases = [("forward reach LEFT arm -> robot RIGHT wrist", "left", "arm",
              {LEFT_ELBOW: (0.34, 0.42, 0.44), LEFT_WRIST: (0.58, 0.60, 0.40)}),
             ("forward step RIGHT leg -> robot LEFT foot", "right", "leg",
              {RIGHT_KNEE: (0.26, -0.26, -0.32), RIGHT_ANKLE: (0.52, -0.26, -0.58),
               RIGHT_HEEL: (0.46, -0.26, -0.62), RIGHT_FOOT_INDEX: (0.64, -0.26, -0.62)})]
    all_ok = True
    for label, hs, kind, ov in cases:
        rs = robot_side(hs)
        targets, _info = retarget(build_obs(ov), mirror=True)
        live_state, settle = command_and_settle(targets)
        tip = (ARM_TIP if kind == "arm" else LEG_TIP)[rs]
        pos = fk.compute(live_state, [tip], "base_link")[tip][0]
        dx = pos[0] - home_tips[tip][0]
        dy = pos[1] - home_tips[tip][1]
        want_neg = want_dy_negative(rs)
        ok = settle < 0.02 and dx > FWD_MIN and ((dy < -SIDE_MIN) if want_neg else (dy > SIDE_MIN))
        all_ok = all_ok and ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}: from LIVE /joint_states "
              f"tip Δx={dx:+.3f} Δy={dy:+.3f} (want Δx>{FWD_MIN}, "
              f"Δy{'<0' if want_neg else '>0'}); settle_err={settle:.4f} rad")

    # leave the robot at home
    command_and_settle({})
    node.destroy_node(); fk.destroy_node(); rclpy.shutdown()
    print("=" * 78)
    print("  LIVE " + ("PASSED" if all_ok else "FAILED"))
    print("=" * 78)
    print("RESULT:" + json.dumps({"live_passed": all_ok}))
    sys.exit(0 if all_ok else 1)


def selftest_model():
    """Prove the QUANTITATIVE check catches a wrong KINEMATIC MODEL -- the exact
    '31 deg wrong vs /compute_fk while internally self-consistent' failure the
    task warns about, which the mirror_fn mutations do NOT exercise.

    Corrupts retarget.ARM_D4 (the elbow offset the arm solver is built on).
    solve_arm stays self-consistent with its OWN wrong constant (residual ~0),
    but the angles it emits put the REAL URDF arm (queried via /compute_fk) in
    the wrong direction. A harness that only trusted the solver's self-FK would
    miss this; mine compares /compute_fk against the INTENDED human direction,
    so it must FAIL. Success of this self-test = that FAIL was observed."""
    import talos_mirror.retarget as rt

    rclpy.init()
    fk = FK()
    print("=" * 78)
    print("  SELF-TEST: corrupt kinematic model (ARM_D4), confirm /compute_fk check catches it")
    print("=" * 78)
    if not fk.wait():
        print("  /compute_fk unavailable"); rclpy.shutdown(); sys.exit(2)
    home_tips = {k: v[0] for k, v in
                 fk.compute(dict(HOME), list(ARM_TIP.values()), "base_link").items()}

    good = run_arm_case(fk, ARM_CASES[0][0], ARM_CASES[0][1], ARM_CASES[0][2], mirror_vec, home_tips)
    orig = rt.ARM_D4
    rt.ARM_D4 = (0.02, 0.12, -0.2782)   # bogus lateral offset on the elbow
    try:
        bad = run_arm_case(fk, ARM_CASES[0][0], ARM_CASES[0][1], ARM_CASES[0][2], mirror_vec, home_tips)
    finally:
        rt.ARM_D4 = orig

    print(f"  intact model : solver resid={good['solver_resid_deg']:.2f} deg, "
          f"/compute_fk seg-dir err ua={good['seg_ua_err_deg']:.2f} -> quant_ok={good['quant_ok']}")
    print(f"  CORRUPTED    : solver resid={bad['solver_resid_deg']:.2f} deg (still self-consistent!), "
          f"/compute_fk seg-dir err ua={bad['seg_ua_err_deg']:.2f} -> quant_ok={bad['quant_ok']}")
    caught = good["quant_ok"] and not bad["quant_ok"]
    print("=" * 78)
    if caught:
        print(f"  PASS: the QUANTITATIVE /compute_fk check CAUGHT the wrong model "
              f"(seg-dir error jumped {good['seg_ua_err_deg']:.2f} -> {bad['seg_ua_err_deg']:.2f} deg "
              f"while the solver's own residual stayed {bad['solver_resid_deg']:.2f} deg).")
    else:
        print("  FAIL: the harness did NOT catch a corrupted kinematic model -- it is not independent.")
    print("=" * 78)
    print("RESULT:" + json.dumps({"selftest_model_caught": caught,
                                  "intact_seg_err": good["seg_ua_err_deg"],
                                  "corrupted_seg_err": bad["seg_ua_err_deg"],
                                  "corrupted_solver_resid": bad["solver_resid_deg"]}))
    rclpy.shutdown()
    sys.exit(0 if caught else 1)


if __name__ == "__main__":
    if "--selftest-model" in sys.argv:
        selftest_model()
    elif "--live" in sys.argv:
        run_live()
    else:
        main()

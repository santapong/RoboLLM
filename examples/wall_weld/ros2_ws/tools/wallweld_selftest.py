#!/usr/bin/env python3
"""Offline (no ROS graph) selftest for wall_weld.py pure logic — run inside
the ros2-arm container under /opt/mpvenv/bin/python:

  /opt/mpvenv/bin/python /ros2_ws/tools/wallweld_selftest.py

Checks RasterPlanner.plan/precheck (incl. shrink + reject), FistPalmSM
hysteresis, select_hand same-index binding, _advance, and the venv cv2 5.0
ArUco class-API call sequence used by the marker mode."""
import math
import sys
import types

sys.path.insert(0, "/ros2_ws/src/robot_arm_moveit_config/scripts")
sys.path.insert(0, "/ros2_ws/tools")
import wall_weld as ww

FAILS = []


def check(name, cond, detail=""):
    print("{:8s} {} {}".format("OK" if cond else "FAIL", name, detail))
    if not cond:
        FAILS.append(name)


class FakeLog:
    def warning(self, msg, **kw):
        print("  [warn]", msg.splitlines()[0][:100])

    def info(self, msg, **kw):
        pass


def fake_node(**over):
    n = types.SimpleNamespace(
        row_step=0.02, pass_step=0.01, margin=0.02, standoff=0.005,
        tool_back=0.07, tool_up=0.09, approach_dist=0.06, bead_proud=0.003,
        home_x=0.20, home_y=0.0, home_z=0.30, grid_ny=7, grid_nz=5,
        ik_check_err_m=0.003, shrink_factor=0.85, shrink_max=4,
        wall_min_w=0.10, wall_min_h=0.06)
    n.get_logger = lambda: FakeLog()
    for k, v in over.items():
        setattr(n, k, v)
    return n


# ---- RasterPlanner.plan -------------------------------------------------
pl = ww.RasterPlanner(fake_node())
wall = ww.WallPose(0.30, 0.0, 0.25, 0.0, 0.35, 0.25, 0.02)
plan = pl.plan(wall)
check("plan.count", len(plan.tool0) == 352 and plan.n_rows == 11,
      "waypoints={} rows={}".format(len(plan.tool0), plan.n_rows))
check("plan.parallel", len(plan.tips) == len(plan.tool0) == len(plan.bead))
t0 = plan.tool0[0]
check("plan.first", abs(t0[0] - 0.215) < 1e-9 and abs(t0[1] + 0.155) < 1e-9
      and abs(t0[2] - 0.235) < 1e-9, str([round(v, 3) for v in t0]))
tipz = [abs(tp[0] - 0.285) < 1e-9 for tp in plan.tips]
check("plan.tips_on_standoff", all(tipz), "tip x == face-standoff for all")
check("plan.approach", abs(plan.approach[0] - 0.155) < 1e-9,
      str([round(v, 3) for v in plan.approach]))
check("plan.serpentine",
      plan.tool0[31][1] > plan.tool0[0][1] and
      abs(plan.tool0[32][1] - plan.tool0[31][1]) < 1e-9 and
      plan.tool0[63][1] < plan.tool0[32][1],
      "row0 L->R, row1 R->L, shared y at the turn")
check("plan.path_len", 3.3 < plan.path_len < 3.7,
      "{:.2f} m".format(plan.path_len))

# yawed wall: normal/tangent rotate, z untouched
wy = ww.WallPose(0.30, 0.0, 0.25, math.radians(15), 0.35, 0.25, 0.02)
ply = pl.plan(wy)
check("plan.yaw_normal",
      abs(wy.normal[0] + math.cos(math.radians(15))) < 1e-9
      and abs(ply.tool0[0][2] - 0.235) < 1e-9)

# ---- precheck: accept, shrink, reject -----------------------------------
ok, w2, rep = pl.precheck(wall)
check("precheck.default_ok", ok and w2.w == wall.w, rep[:80])
big = ww.WallPose(0.30, 0.0, 0.25, 0.0, 0.60, 0.45, 0.02)   # too big: shrink
ok, w2, rep = pl.precheck(big)
check("precheck.shrinks", ok and w2.w < big.w,
      "-> {:.2f}x{:.2f}".format(w2.w, w2.h))
far = ww.WallPose(0.55, 0.0, 0.25, 0.0, 0.35, 0.25, 0.02)   # unreachable
ok, w2, rep = pl.precheck(far)
check("precheck.rejects", not ok, rep[:90])

# ---- FistPalmSM ---------------------------------------------------------
sm = ww.FistPalmSM(5, 3, 0.6)
evs = [sm.update("Closed_Fist", 0.9) for _ in range(8)]
check("sm.fist_commit", evs[4] == "fist" and evs.count("fist") == 1,
      "fires once on frame 5, held fist never refires")
evs = [sm.update("Open_Palm", 0.9) for _ in range(4)]
check("sm.palm_commit", evs[2] == "palm" and evs.count("palm") == 1)
sm.update(None, 0.0)
evs = [sm.update("Closed_Fist", 0.9) for _ in range(5)]
check("sm.refist", evs[4] == "fist", "re-fist after a break re-fires")
sm2 = ww.FistPalmSM(5, 3, 0.6)
seq = ["Closed_Fist"] * 4 + [None] + ["Closed_Fist"] * 4
evs = [sm2.update(s, 0.9 if s else 0.0) for s in seq]
check("sm.consecutive", all(e is None for e in evs),
      "4+4 broken frames never commit (needs 5 consecutive)")
sm3 = ww.FistPalmSM(5, 3, 0.6)
evs = [sm3.update("Closed_Fist", 0.4) for _ in range(10)]
check("sm.low_score", all(e is None for e in evs))

# ---- select_hand: same-index binding ------------------------------------
class C:
    def __init__(self, name, score):
        self.category_name, self.score = name, score


class L:
    def __init__(self, x, y):
        self.x, self.y = x, y


def lms(scale):
    pts = [L(0.5, 0.5)] * 21
    pts = list(pts)
    pts[ww.MID_MCP] = L(0.5, 0.5 - scale)
    return pts


res = types.SimpleNamespace(
    handedness=[[C("Right", 0.9)], [C("Left", 0.95)]],
    gestures=[[C("Open_Palm", 0.8)], [C("Closed_Fist", 0.9)]],
    hand_landmarks=[lms(0.10), lms(0.25)])
got = ww.select_hand(res, "Left", 0.6, 4 / 3)
check("select.left_index", got is not None and got[0] == 1
      and got[4] == "Closed_Fist",
      "gesture read at the LEFT hand's index, not index 0")
got = ww.select_hand(res, "Any", 0.6, 4 / 3)
check("select.any_larger", got[0] == 1, "larger hand wins under Any")
res2 = types.SimpleNamespace(
    handedness=[[C("Left", 0.9)], [C("Left", 0.9)]],
    gestures=[[C("Closed_Fist", 0.9)], [C("Open_Palm", 0.9)]],
    hand_landmarks=[lms(0.30), lms(0.10)])
got = ww.select_hand(res2, "Left", 0.6, 4 / 3)
check("select.two_lefts", got[0] == 0 and got[4] == "Closed_Fist",
      "two Lefts -> larger wins, gesture follows the winner")
check("select.none", ww.select_hand(
    types.SimpleNamespace(handedness=[], gestures=[], hand_landmarks=[]),
    "Any", 0.6, 4 / 3) is None)

# ---- _advance -----------------------------------------------------------
pts = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.02, 0.0, 0.0)]
pos, idx = ww.WallWeld._advance((0.0, 0.0, 0.0), 0, 0.0025, pts)
check("advance.partial", idx == 0 and abs(pos[0] - 0.0025) < 1e-12)
pos, idx = ww.WallWeld._advance(pos, idx, 0.012, pts)
check("advance.crosses", idx == 1 and abs(pos[0] - 0.0145) < 1e-12)
pos, idx = ww.WallWeld._advance(pos, idx, 1.0, pts)
check("advance.clamps_end", idx == 2 and abs(pos[0] - 0.02) < 1e-12)

# ---- venv cv2 5.0 ArUco class API (the exact call sequence used) --------
try:
    import cv2
    import numpy as np
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    img = cv2.aruco.generateImageMarker(d, 7, 240)
    canvas = np.full((480, 640), 255, np.uint8)
    canvas[120:360, 200:440] = img
    bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(bgr)
    check("aruco.detect", ids is not None and int(ids.flatten()[0]) == 7
          and corners[0][0].shape == (4, 2),
          "cv2 {} id={} corners TL={}".format(
              cv2.__version__, ids.flatten()[0] if ids is not None else None,
              corners[0][0][0] if ids is not None else None))
    pts = corners[0][0]
    side = sum(math.dist(pts[i], pts[(i + 1) % 4]) for i in range(4)) / 4
    roll = math.atan2(float(pts[1][1] - pts[0][1]),
                      float(pts[1][0] - pts[0][0]))
    check("aruco.mapping_inputs", abs(side - 239) < 4 and abs(roll) < 0.02,
          "side {:.1f}px roll {:.3f} rad".format(side, roll))
except Exception as exc:  # noqa: BLE001
    check("aruco", False, str(exc))

print("SELFTEST", "PASS" if not FAILS else "FAIL: {}".format(FAILS))
sys.exit(0 if not FAILS else 1)

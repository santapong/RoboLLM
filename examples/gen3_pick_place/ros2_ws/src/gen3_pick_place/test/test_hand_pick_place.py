"""Unit checks for hand_pick_place pure logic (U6 acceptance c).

Covers: preflight joint reconciliation (gap 8), gesture-at-same-index
binding (gap 2), the proximity-gate gripper plan (gaps 5/9), the workspace
map, and the synthetic script's geometric guarantees.

Needs rclpy importable (hand_pick_place imports ROS msgs at module level) —
run inside the ros2-arm container:
    python3 -m pytest test/test_hand_pick_place.py -q
or standalone:  python3 test/test_hand_pick_place.py
"""
import math
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from gen3_pick_place.hand_pick_place import (  # noqa: E402
    GRIP_CLOSE_EMPTY, SYNTH_DROP, SYNTH_SCRIPT,
    gripper_plan, map_to_workspace, reconcile_joint_names, select_left_hand,
    synth_frame,
)
from gen3_pick_place.ik_client import (  # noqa: E402
    ARM_JOINTS, WORKSPACE_BOX, in_box,
)
from gen3_pick_place.scene_box import (  # noqa: E402
    D_ATTACH, GRIP_OPEN, GRIP_PINCH, SPAWN_XYZ,
)
from gen3_pick_place.gesture_sm import (  # noqa: E402
    STATE_GRIPPING, STATE_RELEASED,
)


# ---------------------------------------------------------------------------
# gap 8: preflight subset/reconcile

def test_reconcile_exact_arm_joints():
    assert reconcile_joint_names(list(ARM_JOINTS)) is None


def test_reconcile_tolerates_finger_extras():
    names = list(ARM_JOINTS) + [
        "right_finger_bottom_joint", "right_finger_tip_joint",
        "left_finger_bottom_joint", "left_finger_tip_joint"]
    assert reconcile_joint_names(names) is None


def test_reconcile_rejects_missing_arm_joint():
    names = [j for j in ARM_JOINTS if j != "joint_3"] + [
        "right_finger_bottom_joint"]
    err = reconcile_joint_names(names)
    assert err is not None and "joint_3" in err


def test_reconcile_rejects_wrong_robot():
    err = reconcile_joint_names(["joint1", "joint2", "joint3",
                                 "joint4", "joint5", "joint6"])
    assert err is not None


# ---------------------------------------------------------------------------
# gap 2: gesture read at the SAME result index as the selected LEFT hand

def _lm(x, y, z=0.0):
    return NS(x=x, y=y, z=z)


def _hand_landmarks(wrist=(0.5, 0.5), mcp=(0.5, 0.35)):
    """21 normalized landmarks; only WRIST(0) and MIDDLE_MCP(9) matter."""
    lms = [_lm(0.5, 0.5) for _ in range(21)]
    lms[0] = _lm(*wrist)
    lms[9] = _lm(*mcp)
    return lms


def _world_landmarks(tag):
    """21 'world' landmarks whose x encodes an identifying tag."""
    return [_lm(float(tag), 0.01 * i, 0.0) for i in range(21)]


def _cat(name, score):
    return NS(category_name=name, score=score)


def test_left_selected_not_index_zero_and_gesture_follows():
    """Right at index 0, Left at index 1: everything must come from index 1."""
    res = NS(
        handedness=[[_cat("Right", 0.95)], [_cat("Left", 0.90)]],
        gestures=[[_cat("Open_Palm", 0.88)], [_cat("Closed_Fist", 0.77)]],
        hand_landmarks=[_hand_landmarks(), _hand_landmarks(wrist=(0.3, 0.6))],
        hand_world_landmarks=[_world_landmarks(0), _world_landmarks(1)],
    )
    obs = select_left_hand(res, hand_conf=0.6)
    assert obs is not None and obs.index == 1
    assert obs.gesture_label == "Closed_Fist"        # gestures[1], not [0]
    assert abs(obs.gesture_score - 0.77) < 1e-9
    assert obs.world_landmarks[0].x == 1.0           # world_landmarks[1]
    assert obs.u == 0.3 and obs.v == 0.6             # hand_landmarks[1] wrist


def test_two_lefts_larger_wins_and_binding_holds():
    small = _hand_landmarks(wrist=(0.5, 0.5), mcp=(0.5, 0.44))   # small hand
    big = _hand_landmarks(wrist=(0.2, 0.5), mcp=(0.2, 0.30))     # closer hand
    res = NS(
        handedness=[[_cat("Left", 0.9)], [_cat("Left", 0.8)]],
        gestures=[[_cat("Open_Palm", 0.9)], [_cat("Closed_Fist", 0.8)]],
        hand_landmarks=[small, big],
        hand_world_landmarks=[_world_landmarks(0), _world_landmarks(1)],
    )
    obs = select_left_hand(res, hand_conf=0.6)
    assert obs.index == 1 and obs.gesture_label == "Closed_Fist"
    assert obs.world_landmarks[0].x == 1.0


def test_right_or_low_conf_left_rejected():
    res = NS(
        handedness=[[_cat("Right", 0.99)], [_cat("Left", 0.4)]],
        gestures=[[_cat("Closed_Fist", 0.9)], [_cat("Closed_Fist", 0.9)]],
        hand_landmarks=[_hand_landmarks(), _hand_landmarks()],
        hand_world_landmarks=[_world_landmarks(0), _world_landmarks(1)],
    )
    assert select_left_hand(res, hand_conf=0.6) is None


def test_missing_gesture_entry_yields_none_label():
    res = NS(
        handedness=[[_cat("Left", 0.9)]],
        gestures=[[]],                                # classifier gave nothing
        hand_landmarks=[_hand_landmarks()],
        hand_world_landmarks=[_world_landmarks(0)],
    )
    obs = select_left_hand(res, hand_conf=0.6)
    assert obs is not None and obs.gesture_label is None
    assert obs.gesture_score == 0.0


# ---------------------------------------------------------------------------
# gaps 5 + 9: proximity-gated gripper plan

def test_far_fist_closes_but_does_not_attach():
    action, tgt = gripper_plan(STATE_GRIPPING, attached=False, near_box=False)
    assert action == "none" and tgt == GRIP_CLOSE_EMPTY


def test_carrying_fist_to_box_attaches_then():
    """Gate is re-evaluated per tick: far fist -> close only; the SAME state
    with near_box True -> attach (then pinch)."""
    a1, _ = gripper_plan(STATE_GRIPPING, attached=False, near_box=False)
    a2, tgt = gripper_plan(STATE_GRIPPING, attached=False, near_box=True)
    assert a1 == "none" and a2 == "attach" and tgt == GRIP_PINCH


def test_attached_holds_pinch_regardless_of_gate():
    for near in (True, False):
        action, tgt = gripper_plan(STATE_GRIPPING, attached=True,
                                   near_box=near)
        assert action == "none" and tgt == GRIP_PINCH


def test_release_detaches_then_opens():
    action, tgt = gripper_plan(STATE_RELEASED, attached=True, near_box=True)
    assert action == "detach" and tgt == GRIP_OPEN


def test_released_stays_open():
    action, tgt = gripper_plan(STATE_RELEASED, attached=False, near_box=True)
    assert action == "none" and tgt == GRIP_OPEN


# ---------------------------------------------------------------------------
# workspace map + synthetic script geometry (gaps 10/16 tie-in)

def test_map_corners_stay_inside_box():
    for u, v, s in ((0.0, 0.0, 0.05), (1.0, 1.0, 0.5), (-1.0, 2.0, 0.2),
                    (0.5, 0.5, 0.2)):
        xyz = map_to_workspace(u, v, s, s_near=0.30, s_far=0.10)
        assert in_box(xyz), xyz


def test_map_directions():
    """u right -> +Y, v up -> +Z, closer (bigger s) -> +X."""
    left = map_to_workspace(0.0, 0.5, 0.2, 0.30, 0.10)
    right = map_to_workspace(1.0, 0.5, 0.2, 0.30, 0.10)
    up = map_to_workspace(0.5, 0.0, 0.2, 0.30, 0.10)
    down = map_to_workspace(0.5, 1.0, 0.2, 0.30, 0.10)
    near = map_to_workspace(0.5, 0.5, 0.30, 0.30, 0.10)
    far = map_to_workspace(0.5, 0.5, 0.10, 0.30, 0.10)
    assert right[1] > left[1] and up[2] > down[2] and near[0] > far[0]


def test_synth_script_geometry():
    """Script waypoints inside the tuned box; grasp dwell at BOX spawn;
    drop pose >= 0.15 m from spawn (acceptance a's distance gate)."""
    for (_t, xyz, _l, _s, _r) in SYNTH_SCRIPT:
        assert in_box(xyz), xyz
    # dense sample of the lerped path stays in the box too
    for i in range(0, 1800):
        xyz, _, _, _ = synth_frame(i * 0.01)
        assert in_box(xyz), (i * 0.01, xyz)
    # dwell at the spawn during the fist segment: target == spawn == box
    xyz, label, score, _ = synth_frame(5.5)
    assert label == "Closed_Fist" and math.dist(xyz, SPAWN_XYZ) < 1e-9
    assert math.dist(SYNTH_DROP, SPAWN_XYZ) >= 0.15
    # the fist segment target is within the attach gate by construction
    assert math.dist(xyz, SPAWN_XYZ) <= D_ATTACH


def _run_all():
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS", name)
            except AssertionError as exc:
                fails += 1
                print("FAIL", name, "--", exc)
    print("{} failures".format(fails))
    return fails


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)

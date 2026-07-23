"""Unit tests for gen3_pick_place.gesture_sm (pure python, no ROS).

Runs under pytest, and also standalone:  python3 test_gesture_sm.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from gen3_pick_place.gesture_sm import (  # noqa: E402
    EVENT_GRIP,
    EVENT_RELEASE,
    LABEL_FIST,
    LABEL_PALM,
    STATE_GRIPPING,
    STATE_RELEASED,
    GestureSM,
    GestureSMConfig,
    extension_ratios_from_world_landmarks,
)

DT30 = 1.0 / 30.0  # nominal camera frame period

OPEN_RATIOS = (1.8, 1.8, 1.8, 1.8)   # all four fingers clearly extended
FIST_RATIOS = (1.0, 1.0, 1.0, 1.0)   # all curled
TWO_OPEN_RATIOS = (1.8, 1.8, 1.0, 1.0)  # openness 2 — below release threshold
NONE_LABEL = None


def drive(sm, t0, frames, dt=DT30):
    """Feed a list of (label, score, ratios, target) frames; return outputs."""
    outs = []
    t = t0
    for (label, score, ratios, target) in frames:
        outs.append(sm.update(t, label, score, ratios, target))
        t += dt
    return outs, t


def make_gripping_sm(dt=DT30):
    """Return (sm, t) with a committed GRIP at time t (end of fist run)."""
    sm = GestureSM()
    t = 0.0
    # neutral tracked frames to fill the ring buffer
    for _ in range(10):
        sm.update(t, NONE_LABEL, 0.0, FIST_RATIOS, ("tgt", t))
        t += dt
    grip_seen = False
    for _ in range(sm.cfg.n_grip):
        out = sm.update(t, LABEL_FIST, 0.95, FIST_RATIOS, ("tgt", t))
        grip_seen = grip_seen or out.event == EVENT_GRIP
        t += dt
    assert grip_seen and sm.state == STATE_GRIPPING
    return sm, t


# ---------------------------------------------------------------------------
# 1. flicker rejection (grip)

def test_flicker_rejection_grip():
    sm = GestureSM()
    cfg = sm.cfg
    t = 0.0
    # n_grip - 1 confident fist frames, then a palm frame: must NOT grip
    for _ in range(cfg.n_grip - 1):
        out = sm.update(t, LABEL_FIST, 0.95, FIST_RATIOS)
        assert out.event is None
        t += DT30
    out = sm.update(t, LABEL_PALM, 0.95, OPEN_RATIOS)
    assert out.event is None
    assert sm.state == STATE_RELEASED


# 2. grip commits after exactly n_grip consecutive confident frames

def test_grip_commits_after_n_frames():
    sm = GestureSM()
    t = 0.0
    events = []
    for i in range(sm.cfg.n_grip):
        out = sm.update(t, LABEL_FIST, 0.95, FIST_RATIOS)
        events.append(out.event)
        t += DT30
    assert events[:-1] == [None] * (sm.cfg.n_grip - 1)
    assert events[-1] == EVENT_GRIP
    assert sm.state == STATE_GRIPPING


# 3. score-threshold edge for grip: 0.60 counts (>=), 0.599... never commits

def test_score_threshold_edge_grip():
    sm_low = GestureSM()
    t = 0.0
    for _ in range(20):
        out = sm_low.update(t, LABEL_FIST, 0.599, FIST_RATIOS)
        t += DT30
    assert sm_low.state == STATE_RELEASED  # sub-threshold never commits

    sm_edge = GestureSM()
    t = 0.0
    committed = False
    for _ in range(sm_edge.cfg.n_grip):
        out = sm_edge.update(t, LABEL_FIST, 0.60, FIST_RATIOS)
        committed = committed or out.event == EVENT_GRIP
        t += DT30
    assert committed and sm_edge.state == STATE_GRIPPING


# 4. asymmetric hysteresis: release (n_release) is stickier than grip (n_grip)

def test_release_stickier_than_grip():
    sm, t = make_gripping_sm()
    cfg = sm.cfg
    assert cfg.n_release > cfg.n_grip  # asymmetry is part of the contract
    events = []
    for _ in range(cfg.n_release):
        out = sm.update(t, LABEL_PALM, 0.95, OPEN_RATIOS)
        events.append(out.event)
        t += DT30
    assert events[: cfg.n_release - 1] == [None] * (cfg.n_release - 1)
    assert events[-1] == EVENT_RELEASE
    assert sm.state == STATE_RELEASED


# 5. tilted-palm release via SECONDARY landmark-openness fallback

def test_tilted_palm_release_via_fallback():
    sm, t = make_gripping_sm()
    cfg = sm.cfg
    # classifier degrades to None/0.0 on the tilted palm, but the metric
    # world landmarks still show 4 extended fingers
    released_at = None
    for i in range(40):
        out = sm.update(t, NONE_LABEL, 0.0, OPEN_RATIOS, ("tgt", t))
        if out.event == EVENT_RELEASE:
            released_at = i
            break
        t += DT30
    assert released_at is not None, "secondary fallback never released"
    assert sm.state == STATE_RELEASED
    # must respect the None-score dwell + n_secondary frames — not instant
    min_frames = cfg.n_secondary - 1
    assert released_at >= min_frames


# 6. loss of tracking NEVER releases (worst demo failure guard)

def test_tracking_loss_never_releases():
    sm, t = make_gripping_sm()
    for _ in range(100):  # > 3 s of total tracking loss while carrying
        out = sm.update(t, NONE_LABEL, 0.0, None, None)
        assert out.event is None
        t += DT30
    assert sm.state == STATE_GRIPPING


# 7. secondary fallback needs POSITIVE openness evidence (O >= 3)

def test_secondary_requires_positive_openness():
    sm, t = make_gripping_sm()
    for _ in range(100):
        out = sm.update(t, NONE_LABEL, 0.0, TWO_OPEN_RATIOS, ("tgt", t))
        assert out.event is None
        t += DT30
    assert sm.state == STATE_GRIPPING


# 8. transition latch back-dates the frozen target by ~150 ms

def test_transition_latch_backdates_150ms():
    cfg = GestureSMConfig()
    sm = GestureSM(cfg)
    dt = 0.01  # 100 fps so t_c - 150 ms falls exactly on a buffer entry
    t = 0.0
    # ramping target: value = frame index
    n_pre = 60
    for i in range(n_pre):
        sm.update(t, NONE_LABEL, 0.0, FIST_RATIOS, i)
        t += dt
    # candidate onset: first raw fist frame at t_c = t
    t_c = t
    out = sm.update(t_c, LABEL_FIST, 0.95, FIST_RATIOS, n_pre)
    assert out.position_hold
    expected_t = t_c - cfg.latch_backdate_s
    expected_index = round(expected_t / dt)  # entry appended at that time
    assert out.frozen_target == expected_index
    # frozen target must stay pinned while the candidate continues
    out2 = sm.update(t_c + dt, LABEL_FIST, 0.95, FIST_RATIOS, n_pre + 1)
    assert out2.position_hold and out2.frozen_target == expected_index


# 9. latch aborts on a one-frame flicker: unfreezes, no grip

def test_latch_abort_on_flicker():
    sm = GestureSM()
    t = 0.0
    for i in range(20):
        sm.update(t, NONE_LABEL, 0.0, OPEN_RATIOS, i)
        t += DT30
    out = sm.update(t, LABEL_FIST, 0.95, FIST_RATIOS, 100)  # single fist frame
    assert out.position_hold  # candidate onset freezes immediately
    t += DT30
    held_after = []
    for _ in range(sm.cfg.n_abort + 2):
        out = sm.update(t, LABEL_PALM, 0.95, OPEN_RATIOS, 101)
        held_after.append(out.position_hold)
        t += DT30
    assert held_after[-1] is False  # aborted and unfrozen
    assert sm.state == STATE_RELEASED
    assert sm.frozen_target is None


# 10. position_hold persists through commit + settle window, then clears

def test_latch_settle_after_commit():
    sm, t = make_gripping_sm()
    cfg = sm.cfg
    # commit just happened at (t - DT30); settle_until = commit_t + t_settle
    commit_t = t - DT30
    holds = []
    times = []
    for _ in range(20):
        out = sm.update(t, LABEL_FIST, 0.95, FIST_RATIOS, ("tgt", t))
        holds.append(out.position_hold)
        times.append(t)
        t += DT30
    for h, tt in zip(holds, times):
        if tt < commit_t + cfg.t_settle_s:
            assert h, "hold released before settle window elapsed (t=%.3f)" % tt
        else:
            assert not h, "hold stuck past settle window (t=%.3f)" % tt
    assert sm.state == STATE_GRIPPING  # steady fist: no spurious transitions


# 11. per-finger hysteresis dead band keeps previous state

def test_per_finger_hysteresis_band():
    band = (1.4, 1.4, 1.4, 1.4)  # between e_curled=1.25 and e_extended=1.55
    sm_a = GestureSM()
    out = sm_a.update(0.0, NONE_LABEL, 0.0, OPEN_RATIOS)
    assert out.openness == 4
    out = sm_a.update(DT30, NONE_LABEL, 0.0, band)
    assert out.openness == 4  # stays extended inside the band

    sm_b = GestureSM()
    out = sm_b.update(0.0, NONE_LABEL, 0.0, FIST_RATIOS)
    assert out.openness == 0
    out = sm_b.update(DT30, NONE_LABEL, 0.0, band)
    assert out.openness == 0  # stays curled inside the band


# 12. score-threshold edge for PRIMARY release: 0.59 never counts, 0.60 does

def test_score_threshold_edge_release():
    sm, t = make_gripping_sm()
    for _ in range(20):
        out = sm.update(t, LABEL_PALM, 0.59, FIST_RATIOS)
        assert out.event is None
        t += DT30
    assert sm.state == STATE_GRIPPING
    released = False
    for _ in range(sm.cfg.n_release):
        out = sm.update(t, LABEL_PALM, 0.60, OPEN_RATIOS)
        released = released or out.event == EVENT_RELEASE
        t += DT30
    assert released and sm.state == STATE_RELEASED


# 13. extension-ratio helper computes wrist-anchored 3-D ratios

def test_extension_ratio_helper():
    lm = [(0.0, 0.0, 0.0)] * 21
    # index finger: MCP at distance 1.0, TIP at 1.8  -> e = 1.8
    lm[5] = (0.0, 1.0, 0.0)
    lm[8] = (0.0, 1.8, 0.0)
    # middle: curled, TIP folded back near the wrist -> e = 0.5
    lm[9] = (1.0, 0.0, 0.0)
    lm[12] = (0.5, 0.0, 0.0)
    # ring: 3-D placement, MCP dist 2.0, TIP dist 3.0 -> e = 1.5
    lm[13] = (0.0, 0.0, 2.0)
    lm[16] = (0.0, 3.0, 0.0)
    # pinky: MCP dist 1.0 along z, TIP dist 2.0 -> e = 2.0
    lm[17] = (0.0, 0.0, 1.0)
    lm[20] = (0.0, 0.0, 2.0)
    e = extension_ratios_from_world_landmarks(lm)
    assert abs(e[0] - 1.8) < 1e-9
    assert abs(e[1] - 0.5) < 1e-9
    assert abs(e[2] - 1.5) < 1e-9
    assert abs(e[3] - 2.0) < 1e-9
    # object-with-attributes form also accepted
    class P:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z
    e2 = extension_ratios_from_world_landmarks([P(*p) for p in lm])
    assert e == e2


# 14. secondary fallback honors the None-score dwell: a still-confident fist
#     classifier blocks landmark-only release even with open landmarks

def test_secondary_blocked_by_confident_fist():
    sm, t = make_gripping_sm()
    for _ in range(100):
        # contradictory frame: classifier confidently says fist -> dwell
        # never elapses, secondary must not fire on the (noisy) landmarks
        out = sm.update(t, LABEL_FIST, 0.95, OPEN_RATIOS, ("tgt", t))
        assert out.event is None
        t += DT30
    assert sm.state == STATE_GRIPPING


# ---------------------------------------------------------------------------
# standalone runner (no pytest needed)

if __name__ == "__main__":
    import traceback

    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print("PASS  %s" % name)
        except Exception:
            failed += 1
            print("FAIL  %s" % name)
            traceback.print_exc()
    print("%d/%d passed" % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)

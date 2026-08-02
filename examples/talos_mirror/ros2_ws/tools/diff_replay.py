#!/usr/bin/env python3
"""loop-test DO-PHASE task A1 -- the no-regression instrument, built BEFORE
any optimization edits touch talos_mirror.

    diff_replay.py record-golden [--out GOLDEN.json] [--mode direct|qp]
    diff_replay.py compare       [--golden GOLDEN.json] [--mode direct|qp]
    diff_replay.py self-test     [--golden GOLDEN.json] [--mode direct|qp]

WHAT THIS IS, and how it differs from qp_ab_harness.py's existing record/
replay pair (read that file first -- this one extends its pattern rather
than duplicating it):

  qp_ab_harness.replay() calls talos_mirror.retarget.retarget() DIRECTLY,
  once per tick, with no filtering/clamping/slewing/gating-hold/deadman --
  it measures the retarget+QP LAYER in isolation. That is the right tool for
  an A/B of the solver, but it is NOT what ships: mirror_node.MirrorNode
  ._tick() wraps retarget() in OneEuro filtering (per OUTPUT joint), the
  safety clamp, the per-tick slew limit, per-limb "absent from targets this
  tick == HELD exactly where q_cmd has it" gating, and the /mirror_enable
  deadman (disabled -> freeze, re-enabled -> reseed + filter reset).

  REPAIR ROUND (this version): the previous version of this file modeled
  that state machine with a hand-written `TickSimulator` class -- a COPY of
  _tick()'s logic plus its own hard-coded copies of every mirror_node.py
  parameter default. The gate REFUTED that on a demonstrated defect: a
  comparator built from a copy cannot see edits to the ORIGINAL. Three
  independent mutation tests (deleting the per-limb gating hold, deleting
  the deadman freeze, deleting the One-Euro filter fan-out, changing
  mirror_node.py's/mirror.launch.py's own parameter defaults) all reported
  `pass=true` against that copy, because none of them touched TickSimulator
  itself. mirror_node.py was never imported.

  THIS version drives the ACTUAL shipped MirrorNode instead of a
  reimplementation: `RealNodeTicker` constructs a real
  `talos_mirror.mirror_node.MirrorNode` (rclpy Node) and calls its own
  `_tick()`, `_on_enable()`, `_on_observation()`, `_on_joint_state()`
  methods directly and synchronously -- the exact four entry points a real
  ROS subscription/service/timer would invoke, called by TICK INDEX (house
  law: never wall-clock arrival) instead of through rclpy's executor/timer,
  so replay stays deterministic. `time.monotonic()` (mirror_node._tick's own
  clock read, `t = time.monotonic() - self._t0`) is patched to a controlled
  fake clock so tick k always sees exactly `t = k / CONTROL_HZ`, never real
  wall time. No node parameter is copied or overridden here (other than
  `mode`, which mirror.launch.py itself also forwards as a launch arg) --
  every OTHER default (max_joint_speed, margin, the One-Euro constants, the
  four head/torso gains, ...) comes from mirror_node.py's own
  `declare_parameter` calls, read live off the real node object. A change to
  any of those, or to `_tick()`'s/`_publish()`'s own logic, now changes the
  replayed commanded-joint stream for real, because it IS mirror_node.py
  running, not a description of it.

  This needs rclpy, and mirror_node.py imports rclpy at module scope, so
  THIS FILE NOW REQUIRES an environment with rclpy importable for every
  subcommand (previously only `--mode qp` needed anything beyond bare
  python). `docker/ros2-arm diff-bench` already runs it under
  /opt/mpvenv/bin/python inside ros2-talos:jazzy after sourcing
  /opt/ros/jazzy/setup.bash + install/setup.bash, where rclpy is importable
  (--system-site-packages venv, same reason `body-accept --ros` works there
  -- see that tool's own comment in docker/ros2-arm). Plain host python with
  no ROS install will fail at `import rclpy` -- run it through the launcher.

  A second, narrower gap the same refutations named: this comparator's
  in-process node construction never goes through mirror.launch.py at all
  (that only happens via `ros2 launch`), so a launch-arg default drifting
  out of sync with mirror_node.py's own parameter default would still be
  invisible to a diff of two in-process runs. `check_launch_defaults()`
  below closes that specific gap separately: it parses mirror.launch.py's
  MIRROR_ARGS default strings (pure AST literal-eval, no ROS needed) and
  asserts each equals the live node's own declared default for that
  parameter, folded into `compare`'s pass/fail as `launch_defaults_ok`.

  Also flagged by the same round: house law's "six controller topics stay
  six" had no tripwire here. `check_controller_topics()` asserts
  CONTROLLER_TOPICS has exactly 6 entries, one per GROUP, and that
  GROUP_JOINTS partitions ALL_JOINTS -- folded in as `topics_ok`.

  Known, still-documented scope cut vs the live node: GATE_SCHEDULE (below)
  hides individual LIMBS but the torso's own gate (shoulders visible) is
  never hidden, so `retarget()` never returns an empty `targets` dict here
  and MirrorNode._tick's separate "whole body lost -> hold loss_hold_sec
  then decay to HOME" branch is never exercised by this fixture. That branch
  is independent of the per-limb-gating/deadman machinery this task asks
  for and is out of scope for A1 -- flagged here, not silently absent.

  Side effect of driving the real node in `--mode qp`: the previously
  reported "KNOWN --mode qp LIMITATION" (qp_retarget crashing with
  pink.exceptions.NotWithinConfigurationLimits when one arm is gated out
  while the other is gated in) is no longer an uncaught crash from THIS
  tool's own offline reimplementation -- mirror_node._tick()'s own
  `try/except Exception` around `self.source.read(t)` is now the code
  running, so the real node's real degrade-to-hold behavior is what gets
  measured, not an artifact of this tool having no equivalent guard. The
  underlying qp_retarget.py seeding bug (pin.neutral() = 0.0 seed for a
  gated-off limb's joints) is unchanged and still out of this file's scope.

RECORD-GOLDEN drives SyntheticBodyPoseSource's own deterministic generator
(pose_source.body_points, the SAME pure function of t track/synthetic uses)
through a scripted per-limb VISIBILITY schedule (GATE_SCHEDULE) that hides
and reveals each of the five gated limbs (arm_l, arm_r, leg_l, leg_r, head)
at known times -- one full gate-out/gate-in cycle per limb -- plus a
scripted /mirror_enable disable/enable cycle (ENABLE_SCHEDULE), applied
through the real `_on_enable()` service handler exactly as the live
std_srvs/SetBool call is (it is not part of /body/observation). The
observation stream is recorded at `--rate` Hz (default 30, matching
track_node's synthetic default) and delivered to the real node through its
own `_on_observation()` JSON callback; the tick simulator drives `_tick()`
at `--control-hz` Hz (default 50, mirror_node's own rate) with the SAME
hold-latest "current sample" rule qp_ab_harness.replay uses -- so what
changes between record rate and control rate is rate-DECOUPLING, never a
second clock. The resulting per-tick commanded-joint stream (q_cmd after
every tick, INCLUDING frozen/held ticks -- so a freeze shows up as
identical values repeated, not a gap) is what gets banked as the golden
fixture. The raw recorded observation session itself is NOT banked --
GATE_SCHEDULE/ENABLE_SCHEDULE/body_points are all pure functions of t
already committed as code, so `record-golden` regenerates byte-identical
input on every run (a `recipe_fingerprint` sha256 of the exact fixture
parameters -- NOT node parameters, which are no longer copied here, see
above -- is stored in the golden file's meta and re-checked by `compare`,
so silent drift in the FIXTURE recipe itself is caught rather than
producing a false diff against a session that quietly stopped matching).

COMPARE regenerates that same session, drives the REAL MirrorNode of
WHATEVER BUILD OF talos_mirror IS CURRENTLY CHECKED OUT (the point: run
this after checking out a different commit / applying an optimization
edit, never re-running record-golden first), and diffs the two
commanded-joint streams. Every correlation is by TICK INDEX --
`golden["q"][k]` against `candidate_q[k]`, for the SAME k -- never by
wall-clock arrival time (house law); tick-count identity
(len(candidate) == len(golden)) is itself a PASS/FAIL condition; both
replays are pure functions over the SAME frozen session + schedules (the
fake clock removes wall-time from the equation entirely), so this is not a
race with anything.

NUMERIC FIDELITY SPEC (the PASS/FAIL law this tool enforces):
  - Outside a "dedup hold" tick: PASS requires deviation <= 0.001 deg on
    every joint (BASE_ALLOWED_DEG below). This is not a general epsilon --
    both streams are still deterministic replays of the same inputs through
    the same code, so in principle ANY difference is a real behavior
    change -- but it exists specifically to admit the accepted T1-arm
    bisection change (_solve4's 24 -> 12 halving cut, see retarget.py),
    whose measured floor is 8.0214e-4 deg on arm_l/arm_r (ticks 96/398; see
    retarget.py's own comment for the re-derivation). 0.001 deg is three
    orders of magnitude below physical significance for this robot and sits
    just above that measured floor -- it is not a general tolerance
    widening, and anything at 0.01-deg scale or above still FAILS.
  - During a "dedup hold" tick (T2's coalesced-duplicate-command optimization
    -- NOT YET IMPLEMENTED anywhere in this codebase as of this task; the
    spec is defined here now, ahead of that work, exactly as DO-PHASE A1
    asks): max transient deviation <= 0.65 deg is tolerated. Because T2
    does not exist yet, DEDUP_HOLD_TICKS is currently the empty set for
    every run this tool can actually perform -- so today's effective bar is
    still "0 deg everywhere". A future task that implements T2 marks the
    ticks its own dedup logic holds through `--dedup-ticks FILE` (one tick
    index per line) and this spec starts applying to them for real, with no
    change to this tool's PASS/FAIL logic.
  - Gate transitions (per-limb HELD -> driven or driven -> HELD) and the
    deadman freeze/reseed must be TICK-EXACT: the tick index at which a
    limb's driven-state flips, and the tick indices where /mirror_enable is
    disabled, must be IDENTICAL between golden and candidate. A candidate
    that reproduces the right VALUES one tick early or late still FAILS.
  - `launch_defaults_ok` and `topics_ok` (see above) must both be true.

SELF-TEST proves this instrument fails for the right reason (loop-test's
own rule): it runs compare(golden, golden) -- must PASS, 0 deviation
everywhere, by construction -- then perturbs a COPY of that same candidate
stream (adds a fixed offset to one joint over a known tick range, the
cheapest stand-in for "a build that quietly regressed that joint") and
diffs the perturbed copy against golden -- must FAIL, with the reported
per-group max deviation landing on exactly the perturbed group and the
perturbed tick range. Exits nonzero if either half does not behave as
expected, so a broken comparator (one that PASSes a real perturbation) is
itself caught by running `self-test` (see the CLI's --exit-code contract
below).

REPAIR ROUND 2 (this version): the gate REFUTED round 1 on three demonstrated
defects (plus a runtime hazard and a source/install gap flagged alongside
them). Closed here:

  (a) PUBLISH-LAYER BLINDNESS. Round 1's `RealNodeTicker.tick()` returned
  only `dict(self.node.q_cmd)` -- an INTERNAL dict -- so a `_publish()`
  gutted to a no-op, or silently dropping one of the six controller topics,
  or coalescing/skipping publishes past the numeric spec, left q_cmd
  exactly right while the WIRE went dark, incomplete, or drifted more than
  spec -- and every one of those built `pass=True`. Every one of the six
  `self.node.pubs[group].publish` bound methods is now wrapped
  (`RealNodeTicker._wrap_publishers`) to capture, per publish() call: which
  topic, `msg.joint_names`, `msg.points[0].positions`. Per tick those hits
  fold into a CARRY-FORWARD "wire" vector (one float per joint, seeded at
  HOME, updated only for the joints of a group that actually published
  THIS tick -- a real JointTrajectory controller keeps chasing its last
  commanded point forever, so this reproduces that, not an idealisation).
  `compare()`'s numeric-fidelity gate now diffs the WIRE, not q_cmd -- the
  same 0/0.65 deg spec as round 1, measured on the artifact the spec was
  always actually about. q_cmd is still banked and still gates the
  deadman-freeze invariant (a purely internal contract), but no longer
  gates numeric fidelity on its own.

  (b) "SIX TOPICS STAY SIX" WAS CIRCULAR. The old `check_controller_topics`
  (kept, renamed `check_controller_topics_static`) reads only the
  CONTROLLER_TOPICS/GROUP_JOINTS dict literals it is meant to guard -- a
  `_publish()` that silently drops one topic's loop iteration passed it
  every time. `check_controller_topics_runtime()` reads the RECORDED
  stream instead: the six topic name strings the wrapped publishers
  actually report (`publisher.topic_name`) are asserted equal to
  `LITERAL_CONTROLLER_TOPICS` -- six hardcoded `/{group}_controller/
  joint_trajectory` strings, NOT derived from CONTROLLER_TOPICS, so a
  mutation to that dict cannot launder itself through here -- AND every
  tick with the deadman enabled, outside `--dedup-ticks`, is asserted to
  have published on ALL SIX (by `pub_mask`, a bitmask of which wrapped
  publishers actually fired that tick -- independent of retarget()'s
  driven_mask, which only reflects solver INTENT, not the wire). `topics_
  ok` is now `topics_static["ok"] and topics_runtime["ok"]`.

  (c) `--mode qp` FALSE-FAILED ON A PRISTINE TREE. `_ensure_rclpy_init`
  injects `-p mode:=qp` as a process-global override for a non-default
  `--mode qp` run; `check_launch_defaults()` then compared THAT injected
  value against mirror.launch.py's "direct" default and reported a
  mismatch on every `--mode qp` invocation regardless of tree state -- a
  fail-for-the-wrong-reason. `check_launch_defaults()` now takes
  `override_param` (the one parameter THIS run itself overrode, or None)
  and skips exactly that parameter's comparison; every other launch-arg/
  node-default pair is still asserted equal, so a real drift in `mode`'s
  OWN default is still caught by every `--mode direct` run (every
  subcommand's own default) -- only the run that knowingly overrides
  `mode` stops grading itself against the override it just injected.

  (d) TICK COUNT WAS TAUTOLOGICAL. `n_ticks` was `DURATION_S * CONTROL_HZ`,
  a module constant on both sides, so `tick_count_identical` could only
  ever fire through `recipe_fingerprint_ok` already failing first.
  `simulate()` now reads the LIVE node's own `rate_hz` parameter
  (`driver.node.rate_hz`, mirror_node.py's own `declare_parameter(
  "rate_hz", 50.0)`) AFTER construction and derives `dt`/`n_ticks` from
  THAT, not from CONTROL_HZ (which now only seeds the fixture's own
  session-recording-rate metadata, unchanged). A build whose `rate_hz`
  default drifted now genuinely produces a different tick count.
  `n_published_ticks` (count of ticks where `pub_mask != 0`, i.e. an
  actual publish batch happened) is banked and compared separately too --
  a second count of "how many times did the node actually command
  something", independent of the replay loop's own iteration count.

  Two more items from the same refutation round are closed OUTSIDE this
  file: RUNTIME HAZARD (diff-bench replaying through REAL publishers on
  whatever ROS_DOMAIN_ID a live stack might share) -- see docker/ros2-arm's
  diff-bench block, now pinned to its own domain; and SRC-VS-INSTALL GAP
  (this file always measures src/, a live stack always runs install/) --
  see `check_src_install_parity()` below, a loud pre-flight the CLI runs
  before every subcommand.

  RECIPE_VERSION bumped to A1-v3: the golden file's SHAPE changed (adds
  `wire_overrides`, `pub_mask`, `n_published_ticks`, `topic_names`) even
  though the session recipe itself did not, for the same reason A1-v1 ->
  A1-v2 bumped it -- an old golden should not silently compare as "the
  same recipe" against a mechanism it cannot describe.

REPAIR ROUND 3 (this version): the gate REFUTED round 2 on two demonstrated
defects. Closed here, no golden SHAPE change (both fixes read fields the
golden already recorded -- `meta.topic_names` and `pub_mask` -- so
RECIPE_VERSION stays A1-v3 and existing goldens keep comparing):

  (D1) "SIX TOPICS STAY SIX" WAS A SET COMPARISON, NOT A MAPPING. Round 2's
  `_check_topic_names_and_partition` compared `set(topic_names.values())`
  against `set(LITERAL_CONTROLLER_TOPICS.values())` -- the same six
  destination strings in ANY permutation across groups passed, so an
  arm_l<->arm_r controller cross-wiring (left-arm joint commands published
  to /arm_r_controller/joint_trajectory and vice versa) reported
  `topics_ok=true`. Fixed by comparing the per-group MAPPING as a dict, in
  two independent sub-conditions: `topic_names_literal_mapping_ok` (against
  the hardcoded LITERAL_CONTROLLER_TOPICS, as before, now dict-equality
  instead of set-equality) and the previously-missing
  `topic_names_golden_mapping_ok` (against the GOLDEN bank's own recorded
  `meta.topic_names` -- present in golden_commanded.json since A1-v3, but
  never read by `compare()` until now). See `_check_topic_names_and_
  partition`'s own docstring.

  (D2) THE DEDUP-COUNT GATE HAD NO DEDUP EXEMPTION. `published_tick_count_
  identical` (the equality of `n_published_ticks` between golden and
  candidate) was ANDed into `verdict` unconditionally -- `--dedup-ticks`
  relaxed the numeric allowance (line ~1078, 0.65 deg on declared ticks)
  and the publish-mask tick-exactness check
  (check_controller_topics_runtime's `publish_mask_tick_exact`), but not
  this one, so a spec-conforming T2 dedup build that correctly declared
  every tick it held still hard-failed on this sole remaining term -- a
  fail-for-the-wrong-reason on exactly the optimization DO-PHASE A1 exists
  to permit. Fixed by counting published ticks only over ticks NOT in
  `dedup_ticks`, on both sides, from the per-tick `pub_mask` streams (see
  `compare()`'s own inline comment) -- a declared dedup tick's publish
  status is excluded from this count entirely, since it is already graded
  by the two dedup-aware gates above it. The whole-tick-dedup case (every
  one of the six groups holding on the same declared tick) is therefore
  passable: with every tick declared, both counted totals collapse to
  0 == 0.
"""

import argparse
import ast
import filecmp
import glob
import hashlib
import json
import math
import os
import sys
import time as _time_module
import types

_ROOT = __file__.rsplit("/tools/", 1)[0]
sys.path.insert(0, _ROOT + "/src/talos_mirror")

from talos_mirror.body_track import (  # noqa: E402
    LIMB_GATE_LANDMARKS,
    BodyObservation,
    observation_dict,
    torso_frame,
)
from talos_mirror.pose_source import body_points  # noqa: E402
from talos_mirror.talos_config import (  # noqa: E402
    ALL_JOINTS,
    CONTROLLER_TOPICS,
    GROUP_JOINTS,
    GROUPS,
    home_joint_state,
)

HOME = home_joint_state()
RAD2DEG = 180.0 / math.pi

# Numeric-fidelity floor for ticks NOT declared as a dedup hold (see module
# docstring's NUMERIC FIDELITY SPEC). Was a hard 0.0 deg; widened to admit
# the accepted T1-arm bisection change's measured 8.0214e-4 deg floor
# (arm_l/arm_r only, ticks 96/398 -- see retarget.py's _solve4 comment).
# Three orders of magnitude below physical significance -- not a general
# tolerance, and anything at 0.01-deg scale still fails this gate.
BASE_ALLOWED_DEG = 0.001

# REPAIR ROUND 2, defect (b): six HARDCODED topic name strings, deliberately
# NOT derived from talos_config.CONTROLLER_TOPICS -- check_controller_topics_
# runtime() below asserts the topic names the six wrapped publishers
# actually report at runtime against THESE literals, so a mutation to the
# CONTROLLER_TOPICS dict comprehension cannot launder itself through the
# very check meant to catch it. Must stay in lockstep with talos_config.py's
# `CONTROLLER_NAMES = {g: f"{g}_controller" for g in GROUPS}` /
# `CONTROLLER_TOPICS = {g: f"/{name}/joint_trajectory" ...}` by hand -- that
# IS the point: a change to either side without the other is exactly what
# this is meant to surface, not paper over.
LITERAL_CONTROLLER_TOPICS = {
    "torso": "/torso_controller/joint_trajectory",
    "arm_l": "/arm_l_controller/joint_trajectory",
    "arm_r": "/arm_r_controller/joint_trajectory",
    "head": "/head_controller/joint_trajectory",
    "leg_l": "/leg_l_controller/joint_trajectory",
    "leg_r": "/leg_r_controller/joint_trajectory",
}

# ------------------------------------------------------------ THE FIXTURE
# Deterministic per-limb hide windows, in SESSION TIME (seconds, same t axis
# body_points(t) uses). Each limb gets exactly one gate-OUT (visible ->
# hidden) and one gate-IN (hidden -> visible) transition, at a time no other
# limb's window touches, so a comparator can attribute a mismatched
# transition to a SPECIFIC limb unambiguously. Torso is never hidden here --
# it has no independent gate (see retarget.py's module docstring: torso is
# driven whenever ANYTHING is tracked), and hiding both shoulders would drop
# tracked=True entirely, which is the (deliberately out-of-scope, see module
# docstring) whole-body-loss branch, not per-limb gating.
GATE_SCHEDULE = {
    "arm_l": [(1.0, 2.0)],
    "arm_r": [(2.5, 3.5)],
    "leg_l": [(4.0, 5.0)],
    "leg_r": [(5.5, 6.5)],
    "head": [(7.0, 8.0)],
}

# What to actually zero visibility on for each window above -- deliberately
# NOT the same as body_track.LIMB_GATE_LANDMARKS[limb] for arm_l/arm_r:
# that tuple includes the SHOULDER (LEFT_SHOULDER/RIGHT_SHOULDER), which is
# ALSO body_track.CORE_LANDMARKS -- the torso frame's own prerequisite
# (torso_frame() returns None, i.e. THE WHOLE BODY untracked, the instant
# either shoulder drops below min_visibility; see that function and its
# module's "design change" note on CORE_LANDMARKS being shoulders-only).
# Hiding an arm's shoulder therefore does not test "that arm's gate", it
# tests "the whole body was lost" -- exactly the branch this fixture's own
# module docstring documents as OUT OF SCOPE for A1 -- and a real per-limb
# test needs to fail limb_gate() without touching torso_frame()'s
# prerequisite. Hiding the elbow+wrist alone is sufficient: limb_gate()
# requires EVERY one of LIMB_GATE_LANDMARKS[limb] to clear min_visibility,
# so 2 of 3 failing still gates the limb off, while the shoulder stays
# visible and the torso frame (and every OTHER limb) keeps tracking. legs
# and head need no such carve-out: none of their gate landmarks are shared
# with CORE_LANDMARKS.
GATE_HIDE_LANDMARKS = {
    # LIMB_GATE_LANDMARKS[...] is (SHOULDER, ELBOW, WRIST) for arms -- [1:]
    # drops the shoulder, keeping elbow+wrist as the two landmarks hidden.
    "arm_l": LIMB_GATE_LANDMARKS["arm_l"][1:],
    "arm_r": LIMB_GATE_LANDMARKS["arm_r"][1:],
    "leg_l": LIMB_GATE_LANDMARKS["leg_l"],
    "leg_r": LIMB_GATE_LANDMARKS["leg_r"],
    "head": LIMB_GATE_LANDMARKS["head"],
}

# /mirror_enable disable/enable cycle, same time axis, applied OUT OF BAND
# (this is a live service call in the real system, never carried by
# /body/observation) during tick simulation, not session recording -- now
# delivered through the real node's own `_on_enable()` handler, see
# RealNodeTicker.tick below.
ENABLE_SCHEDULE = [(0.0, True), (10.0, False), (11.0, True)]

DURATION_S = 12.0
RATE_HZ = 30.0
CONTROL_HZ = 50.0
PERIOD_S = 16.0  # body_points' own motion period -- unrelated to the above

# Bumped from A1-v1: the repair round changed HOW the commanded-joint
# stream is produced (real MirrorNode instead of a hand copy of its state
# machine) even though the FIXTURE recipe below is unchanged, so an old
# A1-v1 golden should not silently compare as "the same recipe" against
# this version's mechanism.
RECIPE_VERSION = "A1-v3"


def _recipe_params(mode):
    return {
        "recipe_version": RECIPE_VERSION,
        "mode": mode,
        "gate_schedule": GATE_SCHEDULE,
        # Landmark indices actually zeroed per window -- NOT just
        # GATE_SCHEDULE's time windows -- must be fingerprinted too: a
        # change here (e.g. re-including the shoulder for an arm) silently
        # changes what the fixture exercises (per-limb gate-out vs
        # whole-body loss, see GATE_HIDE_LANDMARKS's own comment) without
        # changing GATE_SCHEDULE at all.
        "gate_hide_landmarks": {k: list(v) for k, v in GATE_HIDE_LANDMARKS.items()},
        "enable_schedule": ENABLE_SCHEDULE,
        "duration_s": DURATION_S,
        "rate_hz": RATE_HZ,
        "control_hz": CONTROL_HZ,
        "period_s": PERIOD_S,
    }


def _fingerprint(mode):
    blob = json.dumps(_recipe_params(mode), sort_keys=True).encode()
    return hashlib.sha256(blob).hexdigest()[:16]


# --------------------------------------------------------------- recording
def _observation_at(t):
    """One BodyObservation at session time `t`: body_points(t) (the SAME
    pure motion generator SyntheticBodyPoseSource.read() drives) with
    GATE_SCHEDULE's per-limb visibility windows applied on top. All-visible
    outside every window -- reproduces the always-tracked default exactly
    like SyntheticBodyPoseSource when no window is active."""
    points = body_points(t, PERIOD_S)
    vis = {i: 1.0 for i in points}
    for limb, windows in GATE_SCHEDULE.items():
        if any(lo <= t < hi for lo, hi in windows):
            for idx in GATE_HIDE_LANDMARKS[limb]:
                vis[idx] = 0.0
    basis, source = torso_frame(points, vis)
    tracked = basis is not None
    return BodyObservation(
        t, points, vis, basis, source if tracked else "", tracked, "" if tracked else source
    )


def record_session():
    """[observation_dict(...), ...] at RATE_HZ over [0, DURATION_S] -- the
    exact /body/observation wire-format dicts, matching qp_ab_harness.
    record's own output shape. Pure function of the module constants above,
    so this is never written to disk (see module docstring: the golden
    fixture banks the COMMANDED-JOINT output of replaying this, not this
    session itself, which any checkout can regenerate byte-identically)."""
    n = int(round(DURATION_S * RATE_HZ)) + 1
    return [observation_dict(_observation_at(i / RATE_HZ)) for i in range(n)]


def _enabled_at(t):
    state = True
    for at_t, val in ENABLE_SCHEDULE:
        if t >= at_t:
            state = val
    return state


# --------------------------------------------------------- real-node driver
class _FakeClock:
    """Mutable stand-in for time.monotonic()'s return value. mirror_node.py
    reads `t = time.monotonic() - self._t0` inside `_tick()` -- patching the
    GLOBAL `time.monotonic` (Python modules are singletons in sys.modules,
    so `mirror_node.time is time_module` here) lets tick index k drive the
    clock exactly (t == k / CONTROL_HZ), never real wall time, per house
    law. Scoped to one RealNodeTicker's lifetime via patch()/unpatch()."""

    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value


_fake_clock = _FakeClock()
_real_monotonic = _time_module.monotonic
_RCLPY_MODE = None  # the mode baked into the one process-global rclpy.init() call


def _patch_time():
    _time_module.monotonic = _fake_clock.monotonic


def _unpatch_time():
    _time_module.monotonic = _real_monotonic


def _ensure_rclpy_init(mode):
    """rclpy.init() may only run once per process without an intervening
    shutdown; `mode` (mirror_node's `mode` parameter -- "direct" or "qp")
    can only be set through a process-global `-p mode:=...` override,
    exactly like mirror.launch.py's own ros_arguments forward it, since
    MirrorNode's constructor takes no per-instance overrides of its own. A
    single CLI invocation only ever needs one mode, so this is baked in at
    first use and asserted consistent on any later use in the same process
    (self-test's two internal simulate() calls always share args.mode).

    mode=="direct" passes NO override at all, rather than forcing
    `-p mode:=direct`. This matters: house law LOCKS mirror_node.py's own
    `mode` default at "direct" ("direct mode default (QP experimental)").
    A blanket override would make a mutation to that default (`p("mode",
    "direct")` -> `p("mode", "qp")` in mirror_node.py) invisible to every
    `--mode direct` run -- which is every subcommand's own default --
    because the override would silently paper over the mutated default
    every single time. Leaving "direct" unforced means a default `--mode
    direct` compare genuinely depends on mirror_node.py's own default, the
    same way a bare `ros2 launch talos_mirror mirror.launch.py` (no
    `mode:=` argument) does. Only a deliberately non-default `--mode qp`
    still needs (and gets) an explicit override."""
    global _RCLPY_MODE
    import rclpy

    if _RCLPY_MODE is None:
        init_args = ["--ros-args", "-p", f"mode:={mode}"] if mode != "direct" else None
        rclpy.init(args=init_args)
        _RCLPY_MODE = mode
    elif _RCLPY_MODE != mode:
        raise RuntimeError(
            f"diff_replay.py bakes `mode` into a process-global rclpy "
            f"parameter override at first use ({_RCLPY_MODE!r}); a second, "
            f"different mode ({mode!r}) was requested in the same process "
            f"-- run these as separate invocations."
        )
    return rclpy


class RealNodeTicker:
    """Drives the ACTUAL shipped talos_mirror.mirror_node.MirrorNode --
    not a re-implementation of its state machine -- through `_tick()`,
    `_on_enable()`, `_on_observation()`, `_on_joint_state()`, the exact
    entry points a real ROS timer/service/subscription would call, invoked
    directly and synchronously so tick index (never wall-clock arrival)
    drives everything. See module docstring's REPAIR ROUND section for why.

    Needs rclpy -- run under /opt/mpvenv/bin/python inside ros2-talos:jazzy
    (docker/ros2-arm diff-bench already does this).

    Every node parameter except `mode` is left at MirrorNode's OWN
    declare_parameter default -- read LIVE off the constructed node, never
    copied into this file -- so a change to any default in mirror_node.py's
    __init__ changes the replayed stream for real.
    """

    def __init__(self, mode="direct"):
        rclpy = _ensure_rclpy_init(mode)  # noqa: F841 -- import + init side effect
        from talos_mirror.mirror_node import MirrorNode

        _fake_clock.value = 0.0
        _patch_time()
        try:
            self.node = MirrorNode()
        except Exception:
            _unpatch_time()
            raise

        # REPAIR ROUND 2 (c): exempt exactly the one parameter THIS run's
        # own _ensure_rclpy_init() overrode from the launch-default
        # cross-check -- see check_launch_defaults()'s own docstring.
        override_param = "mode" if mode != "direct" else None
        self.launch_defaults = check_launch_defaults(self.node, override_param=override_param)

        self._captured_targets = None
        orig_read = self.node.source.read

        def _spy_read(t):
            targets = orig_read(t)
            self._captured_targets = targets
            return targets

        self.node.source.read = _spy_read

        # REPAIR ROUND 2 (a): publish-layer capture -- see module docstring.
        self.topic_names = {}
        self.partition_errors = []
        self._tick_pub_hits = {}
        self._wire = dict(HOME)
        self._wrap_publishers()

        # First-tick seed == mock hardware's boot pose == HOME_POSE (see
        # talos_config.HOME_POSE's own docstring) -- equivalent to
        # MirrorNode._seed() reading /joint_states on its very first tick.
        # node.q_cmd is None until _tick()'s own _seed() call consumes this.
        self._echo_joint_state(dict(HOME))

    def _wrap_publishers(self):
        """Wraps each of the six REAL rclpy Publisher.publish bound methods
        (self.node.pubs[group], created by MirrorNode.__init__'s own
        `self.create_publisher(...)` loop over CONTROLLER_TOPICS) so every
        call is captured -- topic name, joint_names, positions -- before
        being forwarded unchanged to the original publish(). This is the
        ONLY way `_publish()`'s own behavior (as opposed to a description of
        it) becomes visible to this tool -- see module docstring REPAIR
        ROUND 2 (a)."""
        for group, pub in self.node.pubs.items():
            self.topic_names[group] = pub.topic_name
            expected_joints = list(GROUP_JOINTS.get(group, []))
            orig_publish = pub.publish

            def _make_wrapper(g, expected, orig):
                def _wrapper(msg):
                    names = list(msg.joint_names)
                    positions = [float(x) for x in msg.points[0].positions] if msg.points else []
                    if names != expected:
                        # A published joint_names ordering/partition that
                        # doesn't match the group it was published on --
                        # unconditional failure, folded into topics_runtime.
                        self.partition_errors.append({
                            "group": g,
                            "expected_joint_names": expected,
                            "actual_joint_names": names,
                        })
                    self._tick_pub_hits[g] = {"joint_names": names, "positions": positions}
                    return orig(msg)
                return _wrapper

            pub.publish = _make_wrapper(group, expected_joints, orig_publish)

    def _echo_joint_state(self, q):
        names = list(q.keys())
        msg = types.SimpleNamespace(name=names, position=[q[n] for n in names])
        self.node._on_joint_state(msg)

    def tick(self, t, obs, enabled):
        """One control tick. Returns (q_cmd_copy, driven_joint_names_set,
        pub_mask, wire_row):
          - q_cmd_copy: node.q_cmd right after this tick (internal state).
          - driven: joints retarget() targeted this tick (solver INTENT).
          - pub_mask: bitmask over GROUPS -- bit set iff that group's real
            publisher actually fired (publish() was CALLED) this tick.
          - wire_row: the CARRY-FORWARD published-position vector (ALL_
            JOINTS order) after folding in whatever the six wrapped
            publishers emitted this tick, if anything -- unchanged for any
            joint whose group did not publish this tick, exactly like a
            real JointTrajectory controller still chasing its last point.
        """
        was_enabled = self.node.enabled

        if enabled and not was_enabled:
            # Re-enable: mock hardware only ever echoes what was last
            # actually PUBLISHED, and nothing publishes while disabled, so
            # the echoed state equals q_cmd's own pre-freeze value -- feed
            # that BEFORE the real _on_enable() nulls q_cmd, so the real
            # _seed() inside _tick() reseeds from it exactly like a live
            # /joint_states echo would (matches "resuming can never jump").
            frozen = dict(self.node.q_cmd) if self.node.q_cmd is not None else dict(HOME)
            self._echo_joint_state(frozen)

        if enabled != was_enabled:
            req = types.SimpleNamespace(data=bool(enabled))
            self.node._on_enable(req, types.SimpleNamespace())

        self.node._on_observation(types.SimpleNamespace(data=json.dumps(obs)))

        self._captured_targets = None
        self._tick_pub_hits = {}
        _fake_clock.value = t
        self.node._tick()

        driven = set(self._captured_targets or {}) if enabled else set()

        pub_mask = 0
        for i, g in enumerate(GROUPS):
            hit = self._tick_pub_hits.get(g)
            if hit is not None:
                pub_mask |= (1 << i)
                for name, pos in zip(hit["joint_names"], hit["positions"]):
                    self._wire[name] = pos

        wire_row = [self._wire[j] for j in ALL_JOINTS]
        return dict(self.node.q_cmd), driven, pub_mask, wire_row

    def close(self):
        self.node.destroy_node()
        _unpatch_time()


def _driven_mask(driven_joints):
    """Per-tick bitmask over GROUPS (torso=bit0 .. leg_r=bit5): bit set iff
    ANY joint of that group was driven (present in retarget()'s targets)
    this tick. Per-limb gating is all-or-nothing per the retarget() contract
    (a limb's joints are never partially present), so "any" and "all" agree
    here -- "any" is used because it stays correct even if that contract
    were ever loosened, without this comparator silently going blind."""
    mask = 0
    for i, g in enumerate(GROUPS):
        if any(j in driven_joints for j in GROUP_JOINTS[g]):
            mask |= (1 << i)
    return mask


def simulate(session, mode="direct"):
    """Replay `session` (a list of observation_dict()s at RATE_HZ) through
    a REAL MirrorNode with ENABLE_SCHEDULE applied via its own
    `_on_enable()`, hold-latest exactly like qp_ab_harness.replay. Control
    rate/tick-count come from the LIVE node's own `rate_hz` parameter, read
    AFTER construction -- NOT from the CONTROL_HZ module constant (REPAIR
    ROUND 2 (d): a build whose rate_hz default drifted must produce a
    genuinely different tick count, not a tautological one both sides share
    by construction). Returns {"q": [[30 floats]*n_ticks],
    "wire": [[30 floats]*n_ticks], "pub_mask": [int]*n_ticks,
    "driven_mask": [int]*n_ticks, "enabled": [bool]*n_ticks,
    "n_published_ticks": int, "node_control_rate_hz": float,
    "topic_names": {...}, "partition_errors": [...],
    "launch_defaults": {...}}. NOTE: "node_control_rate_hz" (the LIVE
    node's own rate_hz parameter) is a DELIBERATELY different key from
    _recipe_params()'s "rate_hz" (the fixture's fixed 30 Hz OBSERVATION
    recording rate, part of the recipe fingerprint) -- they are different
    quantities and a prior version of this file collided the two under one
    "rate_hz" key, silently overwriting the live control-rate reading with
    the constant recipe value via dict-spread key order."""
    driver = RealNodeTicker(mode=mode)
    try:
        rate_hz = float(driver.node.rate_hz)
        dt = 1.0 / rate_hz
        n_ticks = int(round(DURATION_S * rate_hz)) + 1

        q_stream, mask_stream, enabled_stream = [], [], []
        pub_mask_stream, wire_stream = [], []
        j = 0
        for k in range(n_ticks):
            t = k * dt
            while j + 1 < len(session) and session[j + 1]["t"] <= t:
                j += 1
            obs = session[j]
            enabled = _enabled_at(t)
            q_cmd, driven, pub_mask, wire_row = driver.tick(t, obs, enabled)
            q_stream.append([q_cmd[name] for name in ALL_JOINTS])
            mask_stream.append(_driven_mask(driven))
            enabled_stream.append(enabled)
            pub_mask_stream.append(pub_mask)
            wire_stream.append(wire_row)
        n_published_ticks = sum(1 for m in pub_mask_stream if m != 0)
        return {
            "q": q_stream, "wire": wire_stream, "pub_mask": pub_mask_stream,
            "driven_mask": mask_stream, "enabled": enabled_stream,
            "n_published_ticks": n_published_ticks, "node_control_rate_hz": rate_hz,
            "topic_names": driver.topic_names, "partition_errors": driver.partition_errors,
            "launch_defaults": driver.launch_defaults,
        }
    finally:
        driver.close()


# ---------------------------------------------------------- launch/topics
_LAUNCH_ARG_TO_PARAM = {"mirror_rate_hz": "rate_hz"}  # every other name matches 1:1


def _parse_launch_args(list_name):
    """AST-literal-eval mirror.launch.py's MIRROR_ARGS/TRACK_ARGS list --
    pure text parsing, no ROS/launch import needed, so this half of the
    check works even on bare host python."""
    path = _ROOT + "/src/talos_mirror/launch/mirror.launch.py"
    with open(path) as f:
        source = f.read()
    tree = ast.parse(source, filename=path)
    for node in tree.body:
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == list_name
        ):
            return ast.literal_eval(node.value)
    raise RuntimeError(f"{list_name} not found in {path}")


def _coerce_launch_value(s):
    """Mirrors ROS launch's own YAML-ish type inference for a ros_arguments
    `-p name:=value` string, per mirror.launch.py's own module docstring
    ("ROS's YAML type inference turns '50.0' into a double and 'true' into
    a bool")."""
    low = s.lower()
    if low == "true":
        return True
    if low == "false":
        return False
    try:
        return float(s)
    except ValueError:
        return s


def check_launch_defaults(node, override_param=None):
    """Cross-checks mirror.launch.py's MIRROR_ARGS default VALUES against
    the ACTUAL declared parameter defaults on `node` (a live MirrorNode) --
    catches the launch file and the node source drifting out of the 1:1
    correspondence the launch file's own module docstring promises ("every
    track_node/mirror_node parameter is also a launch argument"), which
    driving the node directly cannot see on its own (direct construction
    never goes through mirror.launch.py's ros_arguments at all).

    `override_param` (REPAIR ROUND 2 (c)): the ONE parameter name (e.g.
    "mode") that THIS run's own `_ensure_rclpy_init()` forced away from its
    declared default via a process-global `-p name:=value` override -- that
    parameter's comparison is skipped, because comparing the launch file's
    default against a value this tool itself just injected is a fail-for-
    the-wrong-reason (a real drift in that parameter's OWN default is still
    caught by every run that does NOT override it, which is the default
    for every subcommand). Every other parameter is still fully checked."""
    mismatches = []
    for name, default_str, _desc in _parse_launch_args("MIRROR_ARGS"):
        param_name = _LAUNCH_ARG_TO_PARAM.get(name, name)
        if override_param is not None and param_name == override_param:
            continue
        expected = _coerce_launch_value(default_str)
        try:
            actual = node.get_parameter(param_name).value
        except Exception as exc:  # noqa: BLE001
            mismatches.append({"launch_arg": name, "param": param_name, "error": str(exc)})
            continue
        # isinstance-gate first: 1.0 == True in python, which would let a
        # bool<->number drift silently pass a bare `==`.
        if isinstance(expected, bool) != isinstance(actual, bool) or expected != actual:
            mismatches.append({
                "launch_arg": name, "param": param_name,
                "launch_default": expected, "node_default": actual,
            })
    return {"ok": not mismatches, "mismatches": mismatches, "skipped_param": override_param}


def check_controller_topics_static():
    """House law 'six controller topics stay six', STATIC half: the
    CONTROLLER_TOPICS/GROUP_JOINTS dict literals partition ALL_JOINTS into
    exactly six groups. Independent of any replay -- always computed fresh
    in `compare`. REPAIR ROUND 2 (b): this half ALONE was refuted as
    circular -- it reads only the config it is meant to guard, so a
    `_publish()` edit that silently stops emitting to one of these six
    topics is invisible to it. See check_controller_topics_runtime for the
    half that watches what actually got published."""
    joints_union = set()
    for j in GROUP_JOINTS.values():
        joints_union |= set(j)
    ok = (
        len(CONTROLLER_TOPICS) == 6
        and set(CONTROLLER_TOPICS) == set(GROUPS)
        and joints_union == set(ALL_JOINTS)
        and sum(len(v) for v in GROUP_JOINTS.values()) == len(ALL_JOINTS)
    )
    return {"ok": ok, "n_topics": len(CONTROLLER_TOPICS), "topic_groups": list(CONTROLLER_TOPICS)}


def _check_topic_names_and_partition(topic_names, partition_errors, golden_topic_names=None):
    """The structural halves of the runtime six-topics guard, plus the
    golden-mapping cross-check -- used both by `build_golden`'s own
    recording-time sanity gate (golden_topic_names=None there -- there is
    no prior golden to cross-check against while RECORDING one) and folded
    into `check_controller_topics_runtime` at compare time (golden_topic_
    names=golden["meta"]["topic_names"] there):

      1. LITERAL MAPPING (not sets): the six {group: topic_name} pairs the
         wrapped publishers actually report (`rclpy.Publisher.topic_name`)
         equal LITERAL_CONTROLLER_TOPICS as a DICT -- hardcoded, NOT
         derived from CONTROLLER_TOPICS, so a mutation to that dict cannot
         launder itself through here. REPAIR ROUND 3 (D1): round 2 compared
         `set(topic_names.values())` against `set(LITERAL_CONTROLLER_
         TOPICS.values())`, i.e. the two VALUE SETS -- so any PERMUTATION
         of the six destinations across groups (e.g. arm_l and arm_r
         controller names swapped) passed, because the same six strings
         were still present, just attached to the wrong groups. Comparing
         the dicts directly makes the per-group assignment part of the
         equality, not just the multiset of destinations.
      2. GOLDEN MAPPING: the same per-group dict equals the GOLDEN bank's
         own recorded `meta.topic_names` (banked at record-golden time).
         REPAIR ROUND 3 (D1), gap (b): round 2's `compare()` never read
         golden's own topic_names at all -- only the hardcoded literal was
         checked, so a golden recorded against one mapping and a candidate
         silently drifted to another (with LITERAL_CONTROLLER_TOPICS ALSO
         edited to match, which the literal check alone cannot catch)
         would still pass. This is a SEPARATE sub-condition from (1): a
         candidate can fail either one independently, and both must hold.
      3. no captured publish ever had a joint_names list that didn't match
         its group's expected partition (`partition_errors`, populated live
         by RealNodeTicker's publish wrapper)."""
    candidate_mapping = dict(topic_names)
    literal_mapping_ok = len(candidate_mapping) == 6 and candidate_mapping == LITERAL_CONTROLLER_TOPICS
    if golden_topic_names is not None:
        golden_mapping_ok = candidate_mapping == dict(golden_topic_names)
    else:
        # build_golden's own recording-time sanity gate has no prior golden
        # to compare against -- vacuously true, NOT a pass on a bad mapping
        # (literal_mapping_ok still gates that case).
        golden_mapping_ok = True
    names_ok = literal_mapping_ok and golden_mapping_ok
    return {
        "ok": names_ok and not partition_errors,
        "topic_names_ok": names_ok,
        # REPAIR ROUND 3 (D1): mapping-specific sub-conditions, reported
        # separately so a mapping mismatch is distinguishable from a
        # partition mismatch, and a literal-dict mismatch is distinguishable
        # from a golden-mapping mismatch.
        "topic_names_literal_mapping_ok": literal_mapping_ok,
        "topic_names_golden_mapping_ok": golden_mapping_ok,
        "topic_names": candidate_mapping,
        "expected_topic_names": dict(LITERAL_CONTROLLER_TOPICS),
        "golden_topic_names": dict(golden_topic_names) if golden_topic_names is not None else None,
        "partition_errors": partition_errors[:20],
        "n_partition_errors": len(partition_errors),
    }


def check_controller_topics_runtime(
    topic_names, golden_pub_mask, cand_pub_mask, dedup_ticks, partition_errors, golden_topic_names=None,
):
    """House law 'six controller topics stay six', RUNTIME half (REPAIR
    ROUND 2 (b)): reads the RECORDED published stream, not the config dict
    the static half reads.

    `publish_mask_tick_exact`: the candidate's `pub_mask` (which of the six
    wrapped publishers actually fired, per tick) must equal the GOLDEN's own
    recorded `pub_mask`, tick for tick, outside `--dedup-ticks` -- exactly
    the same "diff the recorded ground truth by tick index" pattern as
    `gate_transitions_tick_exact`/`deadman_ticks_tick_exact` above, NOT a
    theoretical "must always be all six while enabled" assumption: that
    theoretical version is what this check used BEFORE this fix, and it
    false-failed `--mode qp` record-golden on a clean tree, because qp
    mode's own documented crash-triggered degrade-to-hold (see module
    docstring's "Side effect of driving the real node in --mode qp") is a
    SECOND, legitimate, non-deadman reason `_tick()` can skip `_publish()`
    entirely -- direct mode's fixture never exercises it (torso is always
    driven whenever anything is tracked, so `targets` is never empty here),
    but qp mode's per-tick pink solve can raise. Comparing against the
    GOLDEN's own recorded mask (whatever mode produced it) is correct in
    both cases and, unlike the theoretical version, still catches a
    candidate that stops matching -- including the dedup-holds case, where
    a T2 candidate is EXPECTED to publish less than the (pre-T2) golden did,
    which is exactly what `--dedup-ticks` exists to declare.

    `golden_topic_names` (REPAIR ROUND 3, D1): the GOLDEN bank's own
    recorded `meta.topic_names` mapping, threaded through to
    `_check_topic_names_and_partition` so the candidate's per-group mapping
    is cross-checked against what was actually banked, not only against the
    hardcoded LITERAL_CONTROLLER_TOPICS -- see that function's own
    docstring for why both checks are independently necessary."""
    structural = _check_topic_names_and_partition(
        topic_names, partition_errors, golden_topic_names=golden_topic_names,
    )

    n_common = min(len(golden_pub_mask), len(cand_pub_mask))
    bad_ticks = [
        k for k in range(n_common)
        if k not in dedup_ticks and golden_pub_mask[k] != cand_pub_mask[k]
    ]
    mask_ok = not bad_ticks and len(golden_pub_mask) == len(cand_pub_mask)

    ok = structural["ok"] and mask_ok
    return {
        "ok": ok,
        "topic_names_ok": structural["topic_names_ok"],
        "topic_names_literal_mapping_ok": structural["topic_names_literal_mapping_ok"],
        "topic_names_golden_mapping_ok": structural["topic_names_golden_mapping_ok"],
        "topic_names": structural["topic_names"],
        "expected_topic_names": structural["expected_topic_names"],
        "golden_topic_names": structural["golden_topic_names"],
        "publish_mask_tick_exact": mask_ok,
        "n_bad_publish_mask_ticks": len(bad_ticks),
        "bad_publish_mask_ticks_sample": bad_ticks[:20],
        "partition_errors": structural["partition_errors"],
        "n_partition_errors": structural["n_partition_errors"],
    }


# --------------------------------------------------------------- golden IO
def _sparse_wire_overrides(q_rounded, wire_rounded):
    """Compact storage for the golden's WIRE stream (REPAIR ROUND 2 (a)):
    today (no T2 dedup implemented anywhere yet -- see module docstring), a
    correctly-behaving `_publish()` echoes q_cmd verbatim into the wire
    every tick it fires, and every tick it does NOT fire is a deadman-
    freeze tick where q_cmd is ALSO frozen -- so a golden recorded against
    correct code has wire byte-identical to q at every single tick, and
    storing a full second 601x30-float array alongside "q" would double the
    committed file for zero information (blowing the ~200 KB budget). This
    stores only the ticks where the REAL captured wire (from RealNodeTicker.
    _wrap_publishers, not a re-derivation) actually differs from q -- empty
    on every golden this file can currently produce, but NOT an assumption:
    if a future golden is ever recorded once T2 dedup exists, or against a
    build whose _publish() already corrupts values, this would carry real
    entries and _reconstruct_wire (below) replays them faithfully."""
    overrides = {}
    for k in range(len(q_rounded)):
        if wire_rounded[k] != q_rounded[k]:
            overrides[str(k)] = wire_rounded[k]
    return overrides


def _reconstruct_wire(q_rows, wire_overrides):
    """Inverse of _sparse_wire_overrides -- rebuilds the golden's true
    recorded wire stream from "q" + the sparse override map."""
    out = []
    for k, row in enumerate(q_rows):
        ov = wire_overrides.get(str(k))
        out.append(list(ov) if ov is not None else list(row))
    return out


def build_golden(mode="direct"):
    session = record_session()
    result = simulate(session, mode=mode)
    q_rounded = [[round(v, 6) for v in row] for row in result["q"]]
    wire_rounded = [[round(v, 6) for v in row] for row in result["wire"]]
    wire_overrides = _sparse_wire_overrides(q_rounded, wire_rounded)

    # Structural sanity gate on the session just recorded (REPAIR ROUND 2
    # (b)): the six wrapped publishers reported the six LITERAL topic
    # names and no captured publish had a malformed joint_names partition.
    # This does NOT assert a full six-of-six mask on every tick -- that
    # would be the theoretical assumption `check_controller_topics_runtime`
    # deliberately no longer makes (see its own docstring: qp mode's
    # documented crash-triggered hold is a legitimate reason to skip a
    # publish that has nothing to do with a bad recording). At LEAST one
    # tick must have actually published something, or this "golden" would
    # be an all-frozen no-op baseline.
    structural = _check_topic_names_and_partition(result["topic_names"], result["partition_errors"])
    if not structural["ok"] or result["n_published_ticks"] == 0:
        raise RuntimeError(
            "record-golden: the session just recorded does not satisfy the "
            "structural six-controller-topics invariant (or never published "
            "anything at all) -- refusing to bank a bad baseline. "
            f"structural={json.dumps(structural)} "
            f"n_published_ticks={result['n_published_ticks']}"
        )

    return {
        "meta": {
            "joint_order": list(ALL_JOINTS),
            "group_order": list(GROUPS),
            "recipe_fingerprint": _fingerprint(mode),
            "node_control_rate_hz": result["node_control_rate_hz"],
            "topic_names": result["topic_names"],
            **_recipe_params(mode),
        },
        "n_ticks": len(result["q"]),
        "n_published_ticks": result["n_published_ticks"],
        "q": q_rounded,
        "wire_overrides": wire_overrides,
        "driven_mask": result["driven_mask"],
        "pub_mask": result["pub_mask"],
        "enabled": result["enabled"],
    }


def _transition_ticks(mask_stream):
    """Tick indices where driven_mask differs from the PREVIOUS tick --
    i.e. every gate-out or gate-in event, by construction (deadman freeze
    ticks carry mask 0, which is itself a transition into/out of "nothing
    driven" and is correctly caught here too)."""
    out = []
    prev = None
    for k, m in enumerate(mask_stream):
        if prev is not None and m != prev:
            out.append(k)
        prev = m
    return out


def _freeze_ok(q_stream, enabled_stream):
    """Sanity invariant, independent of any golden comparison: while
    enabled==False, q_cmd must be byte-identical tick to tick (no drift
    while frozen). Returns (ok, first_bad_tick_or_None)."""
    for k in range(1, len(q_stream)):
        if not enabled_stream[k]:
            if q_stream[k] != q_stream[k - 1]:
                return False, k
    return True, None


# ----------------------------------------------------------------- compare
def compare(golden, candidate_mode="direct", dedup_ticks=None):
    """Diff `golden` (a build_golden()-shaped dict, freshly loaded or
    in-memory) against a fresh replay of the CURRENTLY CHECKED OUT
    talos_mirror build's REAL MirrorNode. Returns a report dict;
    report["pass"] is the single PASS/FAIL verdict this tool exists to
    produce."""
    dedup_ticks = dedup_ticks or set()

    expect_fp = _fingerprint(candidate_mode)
    fp_ok = golden["meta"]["recipe_fingerprint"] == expect_fp

    session = record_session()
    result = simulate(session, mode=candidate_mode)
    cand_q, cand_mask, cand_enabled = result["q"], result["driven_mask"], result["enabled"]
    cand_wire, cand_pub_mask = result["wire"], result["pub_mask"]

    n_golden, n_cand = golden["n_ticks"], len(cand_q)
    tick_count_identical = n_golden == n_cand

    # REPAIR ROUND 2 (d): a SECOND, independent tick-count signal -- how
    # many ticks actually published something -- not just how many loop
    # iterations this replay ran. Raw totals kept for reporting only.
    n_pub_golden = golden.get("n_published_ticks")
    n_pub_cand = result["n_published_ticks"]

    # REPAIR ROUND 3 (D2): the RAW totals above are NOT what gates the
    # verdict any more. Round 2's `published_tick_count_identical` ANDed
    # `n_pub_golden == n_pub_cand` unconditionally into `verdict`, with no
    # `--dedup-ticks` exemption -- the module docstring (~lines 145-148) and
    # `check_controller_topics_runtime`'s own docstring both document that a
    # T2 candidate is EXPECTED to publish less than the pre-T2 golden did on
    # its declared dedup-hold ticks, but nothing actually honored that here,
    # so a spec-conforming T2 build (every transient inside the 0.65 deg
    # allowance, every hold correctly declared) failed this ONE remaining
    # unconditional term no matter what it declared.
    #
    # Fix: count "did this tick publish anything" only over ticks NOT
    # declared as a dedup hold, on BOTH sides, from the per-tick `pub_mask`
    # streams (not the aggregate `n_published_ticks` field, which carries no
    # per-tick information to exclude by). A declared dedup tick's publish/
    # no-publish status is excluded from this count entirely -- it is
    # already graded by two OTHER dedup-aware gates: the 0.65 deg numeric
    # allowance just above (`allowed = 0.65 if k in dedup_ticks else 0.0`)
    # and `publish_mask_tick_exact` in check_controller_topics_runtime
    # (`k not in dedup_ticks and golden_pub_mask[k] != cand_pub_mask[k]`) --
    # this gate would otherwise be the ONE place still double-penalizing the
    # exact behavior those two already excuse.
    #
    # Whole-tick-dedup case (charter: "the whole-tick-dedup case ... must be
    # passable"): when every tick is declared, both counted totals collapse
    # to 0 == 0 and this gate cannot veto -- correct, because at that point
    # EVERY tick's publish status is already covered by the two dedup-aware
    # gates above; there is nothing left for a non-dedup-aware count to add.
    golden_pub_mask_raw = golden.get("pub_mask", [])
    n_pub_golden_counted = sum(
        1 for k in range(len(golden_pub_mask_raw)) if k not in dedup_ticks and golden_pub_mask_raw[k] != 0
    )
    n_pub_cand_counted = sum(
        1 for k in range(len(cand_pub_mask)) if k not in dedup_ticks and cand_pub_mask[k] != 0
    )
    published_tick_count_identical = n_pub_golden_counted == n_pub_cand_counted

    golden_wire = _reconstruct_wire(golden["q"], golden.get("wire_overrides", {}))

    joint_order = golden["meta"]["joint_order"]
    group_order = golden["meta"]["group_order"]
    joint_group = {j: g for g in group_order for j in GROUP_JOINTS[g]}

    # REPAIR ROUND 2 (a): the numeric-fidelity gate diffs the WIRE stream
    # (what the six real publishers actually put on the topics, carried
    # forward tick to tick), NOT q_cmd -- see module docstring.
    per_group = {g: {"max_deg": 0.0, "mean_deg_sum": 0.0, "n": 0, "max_tick": None} for g in group_order}
    max_ok = True
    n_common = min(n_golden, n_cand)
    for k in range(n_common):
        gold_row, cand_row = golden_wire[k], cand_wire[k]
        allowed = 0.65 if k in dedup_ticks else BASE_ALLOWED_DEG
        for name, gv, cv in zip(joint_order, gold_row, cand_row):
            # golden_wire is reconstructed from data rounded to 1e-6 rad
            # (see build_golden -- keeps the committed fixture under the
            # size budget); round the freshly-simulated candidate to the
            # SAME precision before diffing, or a bit-exact rerun would
            # show a fake ~1e-6 rad (~6e-5 deg) "deviation" that is a
            # STORAGE artifact, not a behavior difference -- exactly the
            # kind of self-inflicted false positive this instrument must
            # not produce.
            dev_deg = abs(gv - round(cv, 6)) * RAD2DEG
            g = joint_group[name]
            stat = per_group[g]
            stat["mean_deg_sum"] += dev_deg
            stat["n"] += 1
            if dev_deg > stat["max_deg"]:
                stat["max_deg"] = dev_deg
                stat["max_tick"] = k
            if dev_deg > allowed:
                max_ok = False

    for g, stat in per_group.items():
        stat["mean_deg"] = stat["mean_deg_sum"] / stat["n"] if stat["n"] else 0.0
        del stat["mean_deg_sum"], stat["n"]

    gold_transitions = _transition_ticks(golden["driven_mask"])
    cand_transitions = _transition_ticks(cand_mask)
    transitions_match = gold_transitions == cand_transitions

    gold_deadman_ticks = [k for k, e in enumerate(golden["enabled"]) if not e]
    cand_deadman_ticks = [k for k, e in enumerate(cand_enabled) if not e]
    deadman_ticks_match = gold_deadman_ticks == cand_deadman_ticks
    freeze_ok, freeze_bad_tick = _freeze_ok(cand_q, cand_enabled)

    launch_defaults = result["launch_defaults"]
    topics_static = check_controller_topics_static()
    topics_runtime = check_controller_topics_runtime(
        result["topic_names"], golden_pub_mask_raw, cand_pub_mask, dedup_ticks, result["partition_errors"],
        golden_topic_names=golden["meta"].get("topic_names"),
    )
    topics_ok = topics_static["ok"] and topics_runtime["ok"]

    verdict = (
        fp_ok and tick_count_identical and published_tick_count_identical and max_ok
        and transitions_match and deadman_ticks_match and freeze_ok
        and launch_defaults["ok"] and topics_ok
    )

    return {
        "pass": verdict,
        "recipe_fingerprint_ok": fp_ok,
        "tick_count_identical": tick_count_identical,
        "n_ticks_golden": n_golden,
        "n_ticks_candidate": n_cand,
        "published_tick_count_identical": published_tick_count_identical,
        "n_published_ticks_golden": n_pub_golden,
        "n_published_ticks_candidate": n_pub_cand,
        "n_published_ticks_golden_nondedup": n_pub_golden_counted,
        "n_published_ticks_candidate_nondedup": n_pub_cand_counted,
        "deviation_within_spec": max_ok,
        "per_group_deg": per_group,
        "gate_transitions_golden": gold_transitions,
        "gate_transitions_candidate": cand_transitions,
        "gate_transitions_tick_exact": transitions_match,
        "deadman_ticks_golden": gold_deadman_ticks,
        "deadman_ticks_candidate": cand_deadman_ticks,
        "deadman_ticks_tick_exact": deadman_ticks_match,
        "deadman_freeze_ok": freeze_ok,
        "deadman_freeze_first_bad_tick": freeze_bad_tick,
        "dedup_ticks_applied": sorted(dedup_ticks),
        "launch_defaults_ok": launch_defaults["ok"],
        "launch_defaults_mismatches": launch_defaults["mismatches"],
        "launch_defaults_skipped_param": launch_defaults.get("skipped_param"),
        "topics_ok": topics_ok,
        "topics_static": topics_static,
        "topics_runtime": topics_runtime,
        "node_control_rate_hz_golden": golden["meta"].get("node_control_rate_hz"),
        "node_control_rate_hz_candidate": result["node_control_rate_hz"],
    }


# ---------------------------------------------------------- src/install
_PARITY_FILES = ["mirror_node.py", "retarget.py", "qp_retarget.py", "joint_limits.py", "body_track.py"]


def check_src_install_parity():
    """REPAIR ROUND 2, defect (SRC-VS-INSTALL GAP): this whole file always
    measures src/ (see the `sys.path.insert` above -- src wins over
    whatever install/setup.bash already put on sys.path), but a LIVE stack
    (`ros2-arm mirror`, `ros2-arm sweep`, ...) always runs install/.
    docker/ros2-arm's own diff-bench block rebuilds talos_mirror
    UNCONDITIONALLY right before invoking this file, so under normal
    launcher use the two copies are already back in sync by the time this
    runs -- this loud pre-flight exists for the OTHER paths: this file
    invoked directly inside an already-running container shell (bypassing
    that rebuild step), or a colcon build that silently failed to overwrite
    a root-owned stale install/ (see docker/ros2-arm's own comment on that
    failure mode, next to its talos_mirror build guard).

    Returns ok=True, checked=False when no install/ build is found at all
    (a scratch copy with no colcon build yet has no "live stack" to
    diverge against, so there is nothing to catch -- this is NOT the same
    as "stale", which requires an install/ that EXISTS and disagrees)."""
    src_dir = os.path.join(_ROOT, "src", "talos_mirror", "talos_mirror")
    install_candidates = sorted(glob.glob(
        os.path.join(_ROOT, "install", "talos_mirror", "lib", "python3.*", "site-packages", "talos_mirror")
    ))
    if not install_candidates:
        return {
            "ok": True,
            "checked": False,
            "note": (
                "no install/talos_mirror/lib/python3.*/site-packages/talos_mirror "
                "found -- skipping src/install parity (fine for a scratch copy "
                "with no colcon build yet; a LIVE STACK needs install/ built and "
                "in sync with src/)."
            ),
        }
    install_dir = install_candidates[-1]
    stale = []
    for fname in _PARITY_FILES:
        src_path = os.path.join(src_dir, fname)
        if not os.path.exists(src_path):
            continue  # not every file in _PARITY_FILES exists in every checkout
        install_path = os.path.join(install_dir, fname)
        if not os.path.exists(install_path):
            stale.append({"file": fname, "reason": "missing from install/"})
        elif not filecmp.cmp(src_path, install_path, shallow=False):
            stale.append({"file": fname, "reason": "byte-differs from src/"})
    return {"ok": not stale, "checked": True, "install_dir": install_dir, "stale_files": stale}


# ------------------------------------------------------------------- CLI
def _load_golden(path):
    with open(path) as f:
        return json.load(f)


def _save_golden(path, golden):
    with open(path, "w") as f:
        json.dump(golden, f, separators=(",", ":"))


def _load_dedup_ticks(path):
    if not path:
        return set()
    with open(path) as f:
        return {int(line.strip()) for line in f if line.strip()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_rec = sub.add_parser("record-golden")
    p_rec.add_argument("--out", default=_ROOT + "/tools/golden_commanded.json")
    p_rec.add_argument("--mode", choices=("direct", "qp"), default="direct")

    p_cmp = sub.add_parser("compare")
    p_cmp.add_argument("--golden", default=_ROOT + "/tools/golden_commanded.json")
    p_cmp.add_argument("--mode", choices=("direct", "qp"), default="direct")
    p_cmp.add_argument("--dedup-ticks", default=None, help="file, one tick index per line")

    p_self = sub.add_parser("self-test")
    p_self.add_argument("--golden", default=_ROOT + "/tools/golden_commanded.json")
    p_self.add_argument("--mode", choices=("direct", "qp"), default="direct")
    p_self.add_argument("--perturb-joint", default="arm_left_2_joint")
    p_self.add_argument("--perturb-deg", type=float, default=2.0)
    p_self.add_argument("--perturb-tick-start", type=int, default=200)
    p_self.add_argument("--perturb-tick-len", type=int, default=20)

    args = ap.parse_args(argv)

    # REPAIR ROUND 2, SRC-VS-INSTALL GAP: loud pre-flight before every
    # subcommand -- see check_src_install_parity()'s own docstring.
    parity = check_src_install_parity()
    if not parity["ok"]:
        print(json.dumps({
            "error": "src/install parity check FAILED -- install/ is STALE",
            "stale_files": parity["stale_files"],
            "install_dir": parity.get("install_dir"),
            "rebuild_hint": (
                "colcon build --packages-select talos_mirror   (then re-source "
                "install/setup.bash -- or just re-run via "
                "./docker/ros2-arm diff-bench <cmd>, which rebuilds unconditionally)"
            ),
        }, indent=2), file=sys.stderr)
        return 2
    if not parity.get("checked"):
        print(f"# diff_replay.py: {parity['note']}", file=sys.stderr)

    if args.cmd == "record-golden":
        golden = build_golden(mode=args.mode)
        _save_golden(args.out, golden)
        print(json.dumps({
            "wrote": args.out, "n_ticks": golden["n_ticks"],
            "recipe_fingerprint": golden["meta"]["recipe_fingerprint"],
        }))
        return 0

    if args.cmd == "compare":
        golden = _load_golden(args.golden)
        dedup_ticks = _load_dedup_ticks(args.dedup_ticks)
        report = compare(golden, candidate_mode=args.mode, dedup_ticks=dedup_ticks)
        print(json.dumps(report, indent=2))
        return 0 if report["pass"] else 1

    if args.cmd == "self-test":
        golden = _load_golden(args.golden)

        clean = compare(golden, candidate_mode=args.mode)

        # Re-simulate to get a real candidate stream, then perturb a COPY of
        # its WIRE (REPAIR ROUND 2 (a): the gating channel is now the
        # published stream, not q_cmd) -- proves the DIFF LOGIC (compare()'s
        # core, reused verbatim by a real `compare` run) catches a genuine
        # deviation, not a re-derivation of the same code path in isolation.
        session = record_session()
        result = simulate(session, mode=args.mode)
        # Round to the SAME precision build_golden stores (see compare()'s
        # comment on this) -- otherwise every non-perturbed joint shows a
        # spurious ~1e-6 rad "deviation" purely from comparing a rounded
        # stored golden against this unrounded in-memory stream, which would
        # mask whether the instrument's signal is really localised to the
        # one joint/tick-range this self-test perturbs.
        perturbed_q = [[round(v, 6) for v in row] for row in result["q"]]
        perturbed_wire = [[round(v, 6) for v in row] for row in result["wire"]]
        joint_idx = ALL_JOINTS.index(args.perturb_joint)
        lo = args.perturb_tick_start
        hi = min(lo + args.perturb_tick_len, len(perturbed_wire))
        offset_rad = math.radians(args.perturb_deg)
        for k in range(lo, hi):
            perturbed_wire[k][joint_idx] += offset_rad

        fake_golden = dict(golden)
        fake_golden["q"] = perturbed_q
        fake_golden["wire_overrides"] = _sparse_wire_overrides(perturbed_q, perturbed_wire)
        fake_golden["n_ticks"] = len(perturbed_q)
        fake_golden["n_published_ticks"] = result["n_published_ticks"]
        fake_golden["driven_mask"] = result["driven_mask"]
        fake_golden["pub_mask"] = result["pub_mask"]
        fake_golden["enabled"] = result["enabled"]
        fake_golden["meta"] = dict(golden["meta"])

        dirty = compare(fake_golden, candidate_mode=args.mode)
        joint_group = None
        for g in golden["meta"]["group_order"]:
            if args.perturb_joint in GROUP_JOINTS[g]:
                joint_group = g
                break
        caught = (
            not dirty["pass"]
            and not dirty["deviation_within_spec"]
            and dirty["per_group_deg"][joint_group]["max_deg"] >= args.perturb_deg - 1e-6
        )

        report = {
            "clean_compare_pass": clean["pass"],
            "clean_compare_report": clean,
            "perturbed_group": joint_group,
            "perturbed_joint": args.perturb_joint,
            "perturbed_deg": args.perturb_deg,
            "perturbed_tick_range": [lo, hi],
            "dirty_compare_pass": dirty["pass"],
            "dirty_compare_report": dirty,
            "instrument_ok": bool(clean["pass"] and caught),
        }
        print(json.dumps(report, indent=2))
        return 0 if report["instrument_ok"] else 1

    return 1


if __name__ == "__main__":
    raise SystemExit(main())

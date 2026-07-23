"""Gesture state machine for Gen3-lite hand-guided pick-and-place.

Pure Python, ROS-free, unit-testable. One instance per tracked hand.

API contract (binding, per the R3 gap constraints):

* The caller MUST feed ``update()`` the gesture label/score AND the landmark
  openness metric taken at the SAME MediaPipe result index ``i`` as the
  selected LEFT hand (parallel-arrays invariant: ``gestures[i]``,
  ``hand_world_landmarks[i]``).  The API forces this by taking
  ``(label, score, extension_ratios)`` together in a single call — there is
  no way to feed a gesture without also (not) feeding the matching openness
  evidence for that frame.
* ``extension_ratios`` is the 4-tuple of wrist-anchored 3-D extension ratios
  for (index, middle, ring, pinky), computed on the METRIC world landmarks of
  the same hand index — use :func:`extension_ratios_from_world_landmarks`.
  Pass ``None`` when the hand is not tracked this frame.

States: ``RELEASED`` (gripper open / idle) and ``GRIPPING`` (fist committed).
Events: ``"grip"`` / ``"release"`` emitted exactly once on each commit.

RELEASE FALLBACK (binding spec): all release paths require POSITIVE evidence
of an open hand; loss of tracking NEVER releases (loss during carry = hold).
  PRIMARY   : top label Open_Palm at the tracked index, score >= 0.60,
              4 consecutive frames.
  SECONDARY : landmark openness on metric world landmarks. Per non-thumb
              finger f, e_f = |P_TIP - P0| / |P_MCP - P0| (wrist-anchored 3-D
              distances; viewpoint-robust). Per-finger hysteresis: extended
              at e_f >= 1.55, curled at e_f <= 1.25, in between keep previous
              state.  Openness O = count of extended fingers (0-4).
              Secondary release fires when state == GRIPPING, O >= 3 for
              N_secondary consecutive frames, AND the classifier has not
              reported a confident Closed_Fist for >= none_dwell_s
              (None-score dwell — the tilted-palm signature).

GESTURE-TRANSITION LATCH (binding spec): curling into a fist perturbs the
wrist anchor and the depth proxy BEFORE the classifier flips.  A ring buffer
of (t, position_target) covering 0.5 s is kept; at candidate onset t_c (first
raw frame implying a meaningful transition) the target is FROZEN to the ring
buffer entry nearest (t_c - 150 ms), back-dating the latch behind the physical
perturbation.  ``position_hold`` stays True through the transition and a
post-commit settle window; a candidate that never commits is aborted after
N_abort consecutive evidence-free frames or a hard timeout, unfreezing the
target.  The 20 Hz command loop must use ``frozen_target`` whenever
``position_hold`` is True.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Constants

STATE_RELEASED = "RELEASED"
STATE_GRIPPING = "GRIPPING"

EVENT_GRIP = "grip"
EVENT_RELEASE = "release"

LABEL_FIST = "Closed_Fist"
LABEL_PALM = "Open_Palm"

# (MCP, TIP) landmark indices for the four non-thumb fingers, in order
# (index, middle, ring, pinky) — MediaPipe 21-landmark hand topology.
FINGER_MCP_TIP: Tuple[Tuple[int, int], ...] = ((5, 8), (9, 12), (13, 16), (17, 20))
WRIST_IDX = 0


# ---------------------------------------------------------------------------
# Config

@dataclass(frozen=True)
class GestureSMConfig:
    # -- commit hysteresis (asymmetric: release is stickier than grip)
    score_commit: float = 0.60      # min classifier score for a committing frame
    n_grip: int = 3                 # consecutive Closed_Fist frames to commit grip
    n_release: int = 4              # consecutive Open_Palm frames to commit release
    # -- secondary (landmark-openness) release fallback
    e_extended: float = 1.55        # finger extended when e_f >= this
    e_curled: float = 1.25          # finger curled when e_f <= this
    openness_release: int = 3       # fingers extended needed for secondary evidence
    n_secondary: int = 6            # consecutive openness frames to commit release
    none_dwell_s: float = 0.25      # classifier must be fist-silent this long first
    # -- transition latch
    raw_onset_score: float = 0.30   # raw top-label score that counts as onset evidence
    latch_backdate_s: float = 0.150 # freeze to the target this far BEFORE onset
    ring_horizon_s: float = 0.5     # ring buffer span (~15 entries at 30 fps)
    t_settle_s: float = 0.30        # position_hold persists this long after commit
    n_abort: int = 3                # consecutive evidence-free frames to abort candidate
    t_freeze_max_s: float = 1.0     # hard cap on a candidate that never commits


# ---------------------------------------------------------------------------
# Per-frame output

@dataclass
class GestureSMOutput:
    state: str                      # STATE_RELEASED | STATE_GRIPPING
    event: Optional[str]            # EVENT_GRIP | EVENT_RELEASE | None
    position_hold: bool             # True: command loop must use frozen_target
    frozen_target: Any              # latched target while position_hold (else None)
    openness: Optional[int]         # 0-4 this frame, None if no landmarks fed
    raw_label: Optional[str]        # normalized label as seen this frame


# ---------------------------------------------------------------------------
# Helpers

def extension_ratios_from_world_landmarks(
    world_landmarks: Sequence[Any],
) -> Tuple[float, float, float, float]:
    """Wrist-anchored 3-D extension ratios e_f for (index, middle, ring, pinky).

    ``world_landmarks`` is the 21-entry metric hand_world_landmarks list for
    the SAME result index as the selected hand.  Entries may be (x, y, z)
    triples or objects with .x/.y/.z.  Ratios of metric distances in the
    hand-centered frame do not degrade with palm tilt.
    """
    def _xyz(p: Any) -> Tuple[float, float, float]:
        if hasattr(p, "x"):
            return (float(p.x), float(p.y), float(p.z))
        return (float(p[0]), float(p[1]), float(p[2]))

    def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.sqrt(
            (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
        )

    if len(world_landmarks) < 21:
        raise ValueError("expected 21 world landmarks, got %d" % len(world_landmarks))
    p0 = _xyz(world_landmarks[WRIST_IDX])
    out = []
    for mcp_i, tip_i in FINGER_MCP_TIP:
        d_mcp = _dist(_xyz(world_landmarks[mcp_i]), p0)
        d_tip = _dist(_xyz(world_landmarks[tip_i]), p0)
        out.append(d_tip / d_mcp if d_mcp > 1e-9 else 0.0)
    return tuple(out)  # type: ignore[return-value]


def _norm_label(label: Optional[str]) -> Optional[str]:
    """Normalize classifier output: None / '' / 'None' -> None."""
    if label is None or label == "" or label == "None":
        return None
    return label


# ---------------------------------------------------------------------------
# State machine

class GestureSM:
    """Fist=grip / palm=release state machine with release fallback and
    transition latch.  Call :meth:`update` once per vision frame."""

    def __init__(self, config: Optional[GestureSMConfig] = None) -> None:
        self.cfg = config or GestureSMConfig()
        self.state: str = STATE_RELEASED

        # commit counters
        self._grip_count = 0
        self._release_primary_count = 0
        self._release_secondary_count = 0

        # per-finger hysteresis state (index, middle, ring, pinky); start curled
        self._finger_extended = [False, False, False, False]
        self._last_openness: Optional[int] = None

        # None-score dwell: last time a confident Closed_Fist was seen
        self._last_confident_fist_t = -math.inf

        # transition latch
        self._ring: Deque[Tuple[float, Any]] = deque()
        self._phase: str = "idle"          # idle | candidate | settle
        self._candidate_kind: Optional[str] = None  # 'grip' | 'release'
        self._t_onset: float = 0.0
        self._no_evidence_count = 0
        self._settle_until: float = -math.inf
        self.frozen_target: Any = None

    # -- ring buffer -------------------------------------------------------
    def _ring_append(self, t: float, target: Any) -> None:
        if target is not None:
            self._ring.append((t, target))
        while self._ring and (t - self._ring[0][0]) > self.cfg.ring_horizon_s:
            self._ring.popleft()

    def _ring_nearest(self, t_query: float) -> Any:
        best = None
        best_d = math.inf
        for (t, target) in self._ring:  # oldest-first; ties resolve to older
            d = abs(t - t_query)
            if d < best_d:
                best_d = d
                best = target
        return best

    # -- openness ----------------------------------------------------------
    def _update_openness(
        self, ratios: Optional[Sequence[float]]
    ) -> Optional[int]:
        if ratios is None:
            self._last_openness = None
            return None
        if len(ratios) != 4:
            raise ValueError("extension_ratios must have 4 entries (got %d)" % len(ratios))
        for i, e in enumerate(ratios):
            if e >= self.cfg.e_extended:
                self._finger_extended[i] = True
            elif e <= self.cfg.e_curled:
                self._finger_extended[i] = False
            # else: dead band — keep previous per-finger state
        o = sum(1 for x in self._finger_extended if x)
        self._last_openness = o
        return o

    # -- latch helpers -----------------------------------------------------
    def _start_candidate(self, kind: str, t: float) -> None:
        self._phase = "candidate"
        self._candidate_kind = kind
        self._t_onset = t
        self._no_evidence_count = 0
        latched = self._ring_nearest(t - self.cfg.latch_backdate_s)
        if latched is not None:
            self.frozen_target = latched

    def _abort_candidate(self) -> None:
        self._phase = "idle"
        self._candidate_kind = None
        self._no_evidence_count = 0
        self.frozen_target = None

    def _commit(self, new_state: str, t: float) -> None:
        self.state = new_state
        self._grip_count = 0
        self._release_primary_count = 0
        self._release_secondary_count = 0
        if self._phase != "candidate":
            # commit without a prior onset (shouldn't happen in practice, but
            # keep the freeze guarantee): latch back-dated target now.
            latched = self._ring_nearest(t - self.cfg.latch_backdate_s)
            if latched is not None:
                self.frozen_target = latched
        self._phase = "settle"
        self._candidate_kind = None
        self._settle_until = t + self.cfg.t_settle_s

    # -- main entry point --------------------------------------------------
    def update(
        self,
        t: float,
        label: Optional[str],
        score: float,
        extension_ratios: Optional[Sequence[float]] = None,
        position_target: Any = None,
    ) -> GestureSMOutput:
        """Process one vision frame.

        t                : monotonic timestamp (seconds) of the frame
        label, score     : top gesture category at the SELECTED hand's result
                           index i (parallel-arrays invariant — never index 0
                           blindly)
        extension_ratios : 4-tuple e_f from hand_world_landmarks[i] via
                           extension_ratios_from_world_landmarks(), or None
                           when the hand is untracked this frame
        position_target  : the filtered position(+orientation) target computed
                           this frame; appended to the latch ring buffer
        """
        cfg = self.cfg
        label = _norm_label(label)
        self._ring_append(t, position_target)
        openness = self._update_openness(extension_ratios)
        tracked = (label is not None) or (extension_ratios is not None)

        confident_fist = label == LABEL_FIST and score >= cfg.score_commit
        confident_palm = label == LABEL_PALM and score >= cfg.score_commit
        raw_fist = label == LABEL_FIST and score >= cfg.raw_onset_score
        raw_palm = label == LABEL_PALM and score >= cfg.raw_onset_score
        open_evidence = openness is not None and openness >= cfg.openness_release
        if confident_fist:
            self._last_confident_fist_t = t

        event: Optional[str] = None

        # ---- settle phase expiry ----------------------------------------
        if self._phase == "settle" and t >= self._settle_until:
            self._phase = "idle"
            self.frozen_target = None

        # ---- candidate onset (idle only) --------------------------------
        if self._phase == "idle":
            if self.state != STATE_GRIPPING and raw_fist:
                self._start_candidate("grip", t)
            elif self.state == STATE_GRIPPING and (raw_palm or open_evidence):
                self._start_candidate("release", t)

        # ---- commit counters --------------------------------------------
        if self.state == STATE_RELEASED:
            self._grip_count = self._grip_count + 1 if confident_fist else 0
            if self._grip_count >= cfg.n_grip:
                self._commit(STATE_GRIPPING, t)
                event = EVENT_GRIP
        else:  # GRIPPING — release needs POSITIVE evidence; loss never releases
            self._release_primary_count = (
                self._release_primary_count + 1 if confident_palm else 0
            )
            dwell_ok = (t - self._last_confident_fist_t) >= cfg.none_dwell_s
            secondary_frame = open_evidence and dwell_ok and not confident_fist
            self._release_secondary_count = (
                self._release_secondary_count + 1 if secondary_frame else 0
            )
            if (
                self._release_primary_count >= cfg.n_release
                or self._release_secondary_count >= cfg.n_secondary
            ):
                self._commit(STATE_RELEASED, t)
                event = EVENT_RELEASE

        # ---- candidate maintenance / abort ------------------------------
        if self._phase == "candidate":
            if self._candidate_kind == "grip":
                evidence = raw_fist
            else:
                evidence = raw_palm or open_evidence
            if evidence:
                self._no_evidence_count = 0
            elif tracked:
                # tracking-loss frames are neutral: they neither support nor
                # abort a candidate (loss during carry = hold)
                self._no_evidence_count += 1
            if (
                self._no_evidence_count >= cfg.n_abort
                or (t - self._t_onset) >= cfg.t_freeze_max_s
            ):
                self._abort_candidate()

        position_hold = self._phase != "idle"
        return GestureSMOutput(
            state=self.state,
            event=event,
            position_hold=position_hold,
            frozen_target=self.frozen_target if position_hold else None,
            openness=openness,
            raw_label=label,
        )

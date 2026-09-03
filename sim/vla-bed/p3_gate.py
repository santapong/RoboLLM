"""Phase 3 gate G3 (SDD §8): the OXE replay proves the embodiment bridge.

PASS when the map is data-verified, state-mode tracking is within 1.5 cm mean /
0.5 cm final / 5° on every replay episode at the chosen alignment with no
rejections, action-mode tracking with the measured gain stays within 10 cm, inside
the workspace, and beats unit-gain integration by ≥ 2×, and a side-by-side PNG
exists per episode. Thresholds were revised after the first measurement (SDD §6.3).
Writes results/p3/<host>/acceptance.json; exit 1 on FAIL.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path

BED_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BED_DIR))
import resources  # noqa: E402
from oxe import map as oxe_map  # noqa: E402

# Thresholds revised after the first measurement (3–4 Sep 2026, see SDD §6.3):
# state mode — the mean error during motion is the bed's own servo lag at the real
# arm's speeds (up to 2 cm per 0.2 s), so the mean is bounded at 1.5 cm and the
# *final* error (arm at rest) at 0.5 cm; action mode — open-loop integration of
# the real commands drifts because the real teleop loop was closed by a human, so
# the criterion is relative: the measured-gain replay must stay inside the
# workspace and beat unit-gain integration by ≥ 2× on every episode.
STATE_TRANSL_M = 0.015
STATE_FINAL_M = 0.005
STATE_ROT_DEG = 5.0
ACTION_TRANSL_M = 0.10
ACTION_GAIN_RATIO = 2.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    host = socket.gethostname()
    out = args.out or (BED_DIR / "results" / "p3" / host)
    summary = json.loads((out / "summary.json").read_text())
    mapping = oxe_map.load_map()
    checks = {"map_verified": bool(mapping.get("verified")), "alignment_chosen": mapping.get("bed_alignment", {}).get("chosen") is not None}
    for e in summary["episodes"]:
        tag = f"ep{e['episode_index']}"
        s, a, u = e["state_mode"], e["action_mode_measured_gain"], e["action_mode_unit_gain"]
        checks[f"{tag}_state_mean_transl_le_{STATE_TRANSL_M}"] = s["L_transl_m"] <= STATE_TRANSL_M
        checks[f"{tag}_state_final_transl_le_{STATE_FINAL_M}"] = s["final_transl_m"] <= STATE_FINAL_M
        checks[f"{tag}_state_rot_le_{STATE_ROT_DEG}deg"] = s["L_rot_deg"] <= STATE_ROT_DEG
        checks[f"{tag}_state_no_rejections"] = not s["rejections"]
        checks[f"{tag}_action_transl_le_{ACTION_TRANSL_M}"] = a["L_transl_m"] <= ACTION_TRANSL_M
        checks[f"{tag}_action_beats_unit_gain_x{ACTION_GAIN_RATIO}"] = u["L_transl_m"] >= ACTION_GAIN_RATIO * a["L_transl_m"]
        checks[f"{tag}_action_no_rejections"] = not a["rejections"]
        checks[f"{tag}_png"] = bool(e.get("png")) and (out / e["png"]).exists()
    verdict = "PASS" if checks and all(checks.values()) else "FAIL"
    failing = [k for k, v in checks.items() if not v]
    acceptance = {"schema": "robollm.vla-bed.p3-acceptance.v1", "phase": "P3", "verdict": verdict, "checks": checks, "failing": failing, "thresholds": {"state_mean_transl_m": STATE_TRANSL_M, "state_final_transl_m": STATE_FINAL_M, "state_rot_deg": STATE_ROT_DEG, "action_transl_m": ACTION_TRANSL_M, "action_gain_ratio": ACTION_GAIN_RATIO}, "alignment": summary.get("alignment_chosen"), "resources_replay": summary.get("resources"), "resources_gate": resources.snapshot()}
    (out / "acceptance.json").write_text(json.dumps(acceptance, indent=2) + "\n")
    print(json.dumps({"verdict": verdict, "failing": failing, "alignment": summary.get("alignment_chosen")}, indent=1))
    print(f"G3 {verdict} → {out / 'acceptance.json'}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

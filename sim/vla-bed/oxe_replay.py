"""Replay real Open X-Embodiment UR5 episodes on the simulated UR5e (SDD §6.3, §8/P3).

Two modes through the bed's own controller and action path (`BedEnv.step`):

- **state**: the recorded end-effector pose sequence (5 Hz → 20 Hz by linear
  position interpolation and geodesic rotation interpolation) is commanded as
  per-step deltas of the commanded pose; the *actual* sim EE is scored against
  the real pose at every real sample. Isolates reachability and the frame
  alignment between the real base frame and the bed.
- **action**: the recorded actions, expressed in the state frame through the
  measured permutation P (configs/oxe_ur5_map.yaml) and the measured tracking
  gain, are integrated from the recorded initial pose. Isolates the action
  convention co-training relies on.

Scores are SimplerEnv's (2405.05941 eqs. 3–4): L_transl = mean ‖x − x'‖,
L_rot = mean arcsin(‖R − R'‖_F / 2√2). Alignment (rotation about z by k·90°,
z offset so the lowest real pose sits 0.10 m above the floor) is chosen by the
lowest state-mode loss and written back into the map file.

    .venv-lerobot/bin/python sim/vla-bed/oxe_replay.py            # both modes, all alignments, PNGs, summary
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mink  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

BED_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BED_DIR))
import families  # noqa: E402
import resources  # noqa: E402
from env import BedEnv  # noqa: E402
from expert import IK_DT, STEP_LIMITS, clip_action  # noqa: E402
from oxe import map as oxe_map  # noqa: E402
from oxe.geom import mat_to_quat_wxyz, matrix_to_rotvec, quat_xyzw_to_matrix, rot_z, rotation_error_rad, substep_targets  # noqa: E402
from safety import WORKSPACE_HIGH, WORKSPACE_LOW  # noqa: E402
from scene import build_scene  # noqa: E402

EPISODES_FILE = BED_DIR / "configs" / "oxe_replay_episodes.json"
ALIGN_ANGLES_DEG = (0, 90, 180, 270)
FLOOR_CLEARANCE_M = 0.10
SUBSTEPS = 4  # 5 Hz real → 20 Hz bed


def load_episode(df, S, A, E, ep_index: int) -> dict:
    idx = np.where(E == ep_index)[0]
    s, a = S[idx], A[idx]
    return {
        "episode_index": int(ep_index),
        "pos": s[:, :3].copy(),
        "rot": np.stack([quat_xyzw_to_matrix(q) for q in s[:, 3:7]]),
        "gripper_closed": s[:, 7].copy(),
        "actions": a.copy(),
        "frames": int(len(idx)),
    }


def align(ep: dict, theta_deg: float, z_offset: float) -> dict:
    R = rot_z(np.deg2rad(theta_deg))
    t = np.array([0.0, 0.0, z_offset])
    return {**ep, "pos": (ep["pos"] @ R.T) + t, "rot": np.einsum("ij,njk->nik", R, ep["rot"]), "R_align": R, "t_align": t}


def teleport(env: BedEnv, pos: np.ndarray, rot: np.ndarray, iterations: int = 300) -> float:
    """Put the arm at a pose: solve IK from home, write joints, settle physics. Returns residual (m)."""
    c = env.controller
    q = c.configuration.q.copy()
    q[c.arm_qpos] = build_scene.HOME_QPOS
    c.configuration.update(q)
    c.posture_task.set_target(c.configuration.q)
    c.ee_task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rot), pos))
    for _ in range(iterations):
        vel = mink.solve_ik(c.configuration, [c.ee_task, c.posture_task], IK_DT, c.solver, limits=c.limits)
        vel[c.gripper_dof] = 0.0
        c.configuration.integrate_inplace(vel, IK_DT)
    q_arm = c.configuration.q[c.arm_qpos].copy()
    env.data.qpos[env.arm_qpos] = q_arm
    env.data.ctrl[env.arm_act] = q_arm
    env.data.qvel[:] = 0.0
    mujoco.mj_forward(env.model, env.data)
    for _ in range(env.substeps * 4):
        env.data.qfrc_applied[env.arm_dof] = env.data.qfrc_bias[env.arm_dof]
        mujoco.mj_step(env.model, env.data)
    c.sync(env.data)
    return float(np.linalg.norm(env.end_effector - pos))


def in_box(p: np.ndarray) -> bool:
    return bool(np.all(p >= WORKSPACE_LOW) and np.all(p <= WORKSPACE_HIGH))


def step_toward(env: BedEnv, target_pos: np.ndarray, target_rot: np.ndarray, gripper_cmd: float) -> dict:
    cmd_pos, cmd_rot = env.commanded_ee
    action = np.zeros(7)
    action[:3] = target_pos - cmd_pos
    action[3:6] = matrix_to_rotvec(target_rot @ cmd_rot.T)
    action[6] = gripper_cmd
    result = env.step(clip_action(action).astype(np.float32))
    return {"code": result.decision.code, "ok": result.decision.ok}


def score(env: BedEnv, real_pos: np.ndarray, real_rot: np.ndarray) -> tuple[float, float]:
    return float(np.linalg.norm(env.end_effector - real_pos)), rotation_error_rad(env.ee_rot, real_rot)


def run_state_mode(env: BedEnv, ep: dict) -> dict:
    spec = families.EpisodeSpec(seed=ep["episode_index"], split="evaluation", family="oxe", cell=0, target=tuple(ep["pos"][-1]), initial_q=tuple(build_scene.HOME_QPOS))
    env.reset(spec)
    teleport_res = teleport(env, ep["pos"][0], ep["rot"][0])
    transl, rots, codes, outside = [], [], {}, 0
    sim_path = [env.end_effector.copy()]
    for k in range(1, ep["frames"]):
        grip = 1.0 if ep["gripper_closed"][k] > 0.5 else -1.0
        for tp, tr in substep_targets(ep["pos"][k - 1], ep["rot"][k - 1], ep["pos"][k], ep["rot"][k], SUBSTEPS):
            if not in_box(tp):
                outside += 1
            r = step_toward(env, tp, tr, grip)
            if not r["ok"]:
                codes[r["code"]] = codes.get(r["code"], 0) + 1
        e_t, e_r = score(env, ep["pos"][k], ep["rot"][k])
        transl.append(e_t)
        rots.append(e_r)
        sim_path.append(env.end_effector.copy())
    return _pack(transl, rots, codes, outside, teleport_res, ep, sim_path, env)


def run_action_mode(env: BedEnv, ep: dict, P: np.ndarray, Pr: np.ndarray, gain_t: float, gain_r: float) -> dict:
    spec = families.EpisodeSpec(seed=ep["episode_index"], split="evaluation", family="oxe", cell=0, target=tuple(ep["pos"][-1]), initial_q=tuple(build_scene.HOME_QPOS))
    env.reset(spec)
    teleport_res = teleport(env, ep["pos"][0], ep["rot"][0])
    R = ep["R_align"]
    transl, rots, codes, outside = [], [], {}, 0
    sim_path = [env.end_effector.copy()]
    for k in range(ep["frames"] - 1):
        a = ep["actions"][k]
        d_state = gain_t * (P @ a[:3])  # real action → state-frame displacement
        w_state = gain_r * (Pr @ a[3:6])  # real rpy delta → state-frame rotation vector
        d_sim, w_sim = R @ d_state, R @ w_state
        grip = -1.0 if a[6] > 0.5 else 1.0  # dataset: 1 = open; bed: −1 open, +1 close
        for _ in range(SUBSTEPS):
            action = np.concatenate([d_sim / SUBSTEPS, w_sim / SUBSTEPS, [grip]])
            cmd_pos, _ = env.commanded_ee
            if not in_box(cmd_pos + action[:3]):
                outside += 1
            result = env.step(clip_action(action).astype(np.float32))
            if not result.decision.ok:
                codes[result.decision.code] = codes.get(result.decision.code, 0) + 1
        e_t, e_r = score(env, ep["pos"][k + 1], ep["rot"][k + 1])
        transl.append(e_t)
        rots.append(e_r)
        sim_path.append(env.end_effector.copy())
    return _pack(transl, rots, codes, outside, teleport_res, ep, sim_path, env)


def _pack(transl, rots, codes, outside, teleport_res, ep, sim_path, env) -> dict:
    return {
        "L_transl_m": round(float(np.mean(transl)), 4),
        "L_rot_rad": round(float(np.mean(rots)), 4),
        "L_rot_deg": round(float(np.degrees(np.mean(rots))), 2),
        "max_transl_m": round(float(np.max(transl)), 4),
        "final_transl_m": round(float(transl[-1]), 4),
        "teleport_residual_m": round(teleport_res, 4),
        "steps_outside_box": int(outside),
        "steps_total": int((ep["frames"] - 1) * SUBSTEPS),
        "rejections": codes,
        "safe": env.safety.safe,
        "sim_path": np.asarray(sim_path),
    }


def side_by_side(env: BedEnv, ep: dict, res_state: dict, res_action: dict | None, out: Path, title: str) -> None:
    frame = Image.fromarray(env.observation()["observation.images.front"])
    W, H = 224, 224
    canvas = Image.new("RGB", (W * 3, H + 26), (248, 250, 252))
    canvas.paste(frame, (0, 26))
    d = ImageDraw.Draw(canvas)
    d.text((4, 6), title[:90], fill=(15, 23, 42))
    paths = [("real", ep["pos"], (15, 108, 189)), ("sim state", res_state["sim_path"], (217, 119, 6))]
    if res_action is not None:
        paths.append(("sim action", res_action["sim_path"], (22, 163, 74)))
    allp = np.vstack([p for _, p, _ in paths])
    for panel, (ax, ay, label) in enumerate([((0, 1), (0, 1), "top view x–y"), ((0, 2), (0, 2), "side view x–z")]):
        ox = W * (panel + 1)
        lo, hi = allp.min(0), allp.max(0)
        span = max((hi - lo).max(), 1e-3)
        def px(p):
            return (ox + 12 + (p[ax[0]] - lo[ax[0]]) / span * (W - 24), 26 + H - 12 - (p[ay[1]] - lo[ay[1]]) / span * (H - 24))
        d.rectangle([ox, 26, ox + W - 1, 26 + H - 1], outline=(148, 163, 184))
        d.text((ox + 4, 30), label, fill=(71, 85, 105))
        for i, (name, p, col) in enumerate(paths):
            pts = [px(q) for q in p]
            d.line(pts, fill=col, width=2)
            d.text((ox + 4, 26 + H - 14 * (len(paths) - i)), name, fill=col)
    canvas.save(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--angles", type=str, default=",".join(str(a) for a in ALIGN_ANGLES_DEG))
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    host = socket.gethostname()
    out = args.out or (BED_DIR / "results" / "p3" / host)
    out.mkdir(parents=True, exist_ok=True)

    mapping = oxe_map.load_map()
    episodes_cfg = json.loads(EPISODES_FILE.read_text())
    df, S, A, E = oxe_map.load_arrays()
    P = np.asarray(mapping["action_frame_vs_state_frame"]["translation"]["P_action_to_state"], dtype=float)
    Pr = np.asarray(mapping["action_frame_vs_state_frame"]["rotation"]["P_action_to_state"], dtype=float)
    gain_t = float(mapping["action_frame_vs_state_frame"]["translation"]["gain"])
    gain_r = float(mapping["action_frame_vs_state_frame"]["rotation"]["gain"])
    episodes = [load_episode(df, S, A, E, e["episode_index"]) for e in episodes_cfg["episodes"]]
    z_min = min(float(ep["pos"][:, 2].min()) for ep in episodes)
    z_offset = FLOOR_CLEARANCE_M - z_min
    angles = [float(a) for a in args.angles.split(",")]

    t0 = time.perf_counter()
    env = BedEnv(render=not args.no_render)
    summary: dict = {"schema": "robollm.vla-bed.oxe-replay.v1", "phase": "P3", "host": host, "repo_id": episodes_cfg["repo_id"], "revision": episodes_cfg["revision"], "z_offset_m": round(z_offset, 4), "floor_clearance_m": FLOOR_CLEARANCE_M, "substeps": SUBSTEPS, "map_gain": {"translation": gain_t, "rotation": gain_r}, "alignment_search": {}, "episodes": []}
    try:
        # 1. alignment by state-mode loss
        for theta in angles:
            losses = []
            for ep in episodes:
                res = run_state_mode(env, align(ep, theta, z_offset))
                losses.append({"episode_index": ep["episode_index"], **{k: v for k, v in res.items() if k != "sim_path"}})
            summary["alignment_search"][str(int(theta))] = {"mean_L_transl_m": round(float(np.mean([l["L_transl_m"] for l in losses])), 4), "mean_L_rot_deg": round(float(np.mean([l["L_rot_deg"] for l in losses])), 2), "steps_outside_box": int(sum(l["steps_outside_box"] for l in losses)), "rejections": int(sum(sum(l["rejections"].values()) for l in losses)), "per_episode": losses}
        best = min(summary["alignment_search"].items(), key=lambda kv: kv[1]["mean_L_transl_m"])
        theta_best = float(best[0])
        summary["alignment_chosen"] = {"rotation_about_z_deg": theta_best, "z_offset_m": round(z_offset, 4), "mean_L_transl_m": best[1]["mean_L_transl_m"], "mean_L_rot_deg": best[1]["mean_L_rot_deg"]}
        # 2. action mode at the chosen alignment, measured gain and unit gain
        for cfg in episodes_cfg["episodes"]:
            ep = align(next(e for e in episodes if e["episode_index"] == cfg["episode_index"]), theta_best, z_offset)
            res_state = run_state_mode(env, ep)
            res_action = run_action_mode(env, ep, P, Pr, gain_t, gain_r)
            res_action_unit = run_action_mode(env, ep, P, Pr, 1.0, 1.0)
            png = out / f"episode_{cfg['episode_index']:04d}_{cfg['task_index']}.png"
            if not args.no_render:
                run_action_mode(env, ep, P, Pr, gain_t, gain_r)  # leave the env at the action-replay end state for the frame
                side_by_side(env, ep, res_state, res_action, png, f"ep {cfg['episode_index']} · {cfg['task']}")
            summary["episodes"].append({
                "episode_index": cfg["episode_index"], "task": cfg["task"], "frames": ep["frames"],
                "state_mode": {k: v for k, v in res_state.items() if k != "sim_path"},
                "action_mode_measured_gain": {k: v for k, v in res_action.items() if k != "sim_path"},
                "action_mode_unit_gain": {k: v for k, v in res_action_unit.items() if k != "sim_path"},
                "png": png.name if not args.no_render else None,
            })
    finally:
        env.close()
    summary["wall_s"] = round(time.perf_counter() - t0, 1)
    summary["resources"] = resources.snapshot()
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    # write the chosen alignment back into the map file
    mapping["bed_alignment"]["chosen"] = summary["alignment_chosen"]
    oxe_map.write_yaml(mapping)
    brief = {"alignment_search": {k: {kk: vv for kk, vv in v.items() if kk != "per_episode"} for k, v in summary["alignment_search"].items()}, "alignment_chosen": summary["alignment_chosen"],
             "episodes": [{k: (e[k] if k in ("episode_index", "frames") else {kk: e[k][kk] for kk in ("L_transl_m", "L_rot_deg", "max_transl_m", "steps_outside_box", "rejections", "teleport_residual_m")}) for k in ("episode_index", "frames", "state_mode", "action_mode_measured_gain", "action_mode_unit_gain")} for e in summary["episodes"]],
             "wall_s": summary["wall_s"], "resources": summary["resources"]}
    print(json.dumps(brief, indent=1))
    print(f"→ {out / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

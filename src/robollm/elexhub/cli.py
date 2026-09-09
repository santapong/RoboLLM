"""Reproducible local entry points. No command imports or controls Kaggle."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import subprocess
import time
import urllib.request
import zipfile
from pathlib import Path

import yaml
from robo_arm_driver.config import load_arm_config
from robo_arm_driver.elexhub_profile import hardware_gaps

ROOT = Path(__file__).resolve().parents[3]
PHYSICAL = ROOT / "ros2/robo_arm_driver/config/elexhub.yaml"
SIMULATION = PHYSICAL.with_name("elexhub.sim.yaml")


def simulation_profile(source, destination):
    raw = yaml.safe_load(Path(source).read_text())
    if raw.get("runtime") != "hardware":
        raise ValueError("derive simulation from the physical source profile")
    result = copy.deepcopy(raw)
    result.update(runtime="simulation", calibrated=True, commissioning_allowed=False)
    Path(destination).write_text(
        "# GENERATED from elexhub.yaml; explicit provisional simulation, never USB.\n"
        + yaml.safe_dump(result, sort_keys=False)
    )
    load_arm_config(destination)
    return destination


def download_models(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    urls = {
        "hand_landmarker.task": "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task",
        "vosk-model-small-en-us-0.15.zip": "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip",
    }
    records = []
    for name, url in urls.items():
        target = directory / name
        if not target.exists():
            temporary = target.with_suffix(target.suffix + ".part")
            urllib.request.urlretrieve(url, temporary)
            temporary.replace(target)
        records.append(
            {
                "file": name,
                "source": url,
                "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
                "bytes": target.stat().st_size,
            }
        )
        if (
            name.endswith(".zip")
            and not (directory / name.removesuffix(".zip")).exists()
        ):
            with zipfile.ZipFile(target) as archive:
                for member in archive.infolist():
                    candidate = (directory / member.filename).resolve()
                    if not candidate.is_relative_to(directory.resolve()):
                        raise ValueError("unsafe model archive member")
                    if (member.external_attr >> 16) & 0o170000 == 0o120000:
                        raise ValueError("model archive contains a symlink")
                archive.extractall(directory)
    (directory / "MODEL_PROVENANCE.json").write_text(
        json.dumps(records, indent=2) + "\n"
    )
    return records


def register(calibration, measurements, output):
    import numpy as np

    from .vision import TableCamera, rigid, save_calibration

    camera = TableCamera(calibration)
    raw = json.loads(Path(measurements).read_text())

    def pairs(key):
        value = raw.get(key, [])
        if len(value) < 6:
            raise ValueError(f"{key} requires at least six measured point pairs")
        a = np.asarray([p["mat_m"] for p in value], float)
        b = np.asarray([p["base_m"] for p in value], float)
        if (
            a.shape != (len(value), 3)
            or b.shape != a.shape
            or not np.isfinite(a).all()
            or not np.isfinite(b).all()
        ):
            raise ValueError("point pairs must be finite 3D coordinates in meters")
        if np.linalg.matrix_rank(a - a.mean(0), tol=1e-5) < 2:
            raise ValueError("registration points must span the tabletop")
        return a, b

    a, b = pairs("fit_pairs")
    ha, hb = pairs("held_out_pairs")
    if any(np.linalg.norm(p - a, axis=1).min() < 0.001 for p in ha):
        raise ValueError("held-out points overlap the fitted points")
    u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
    R = vt.T @ u.T
    if np.linalg.det(R) < 0:
        vt[-1] *= -1
        R = vt.T @ u.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = b.mean(0) - R @ a.mean(0)
    if T[2, 2] < 0.9:
        raise ValueError("tabletop +Z must agree with robot base +Z")
    error = (R @ ha.T).T + T[:3, 3] - hb
    rmse = float(np.sqrt(np.mean(np.sum(error**2, axis=1))))
    if rmse > 0.005:
        raise ValueError(
            f"held-out registration error {rmse * 1000:.2f} mm exceeds 5 mm"
        )
    offset = rigid(raw["T_toolmarker_tcp"])
    evidence = raw.get("measurement_record")
    if not isinstance(evidence, str) or not evidence.strip():
        raise ValueError("measurement_record is required")
    return save_calibration(
        {
            **camera.calibration,
            "T_base_mat": T.tolist(),
            "T_toolmarker_tcp": offset.tolist(),
            "held_out_rmse_m": rmse,
            "base_registration_evidence": evidence,
            "registration_measurements_sha256": hashlib.sha256(
                Path(measurements).read_bytes()
            ).hexdigest(),
            "registration_fit_count": len(a),
            "registration_held_out_count": len(ha),
        },
        output,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=None)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("doctor")
    sub.add_parser("sync-sim-profile")
    p = sub.add_parser("build")
    p.add_argument("--output", type=Path, default=ROOT / "assets/elexhub/provisional")
    p = sub.add_parser("cad")
    p.add_argument("--freecad", required=True)
    p.add_argument(
        "--gui-colors",
        action="store_true",
        help="use a FreeCAD GUI executable offscreen to save CAD materials",
    )
    p.add_argument("--output", type=Path, default=ROOT / "assets/elexhub/provisional")
    p = sub.add_parser("printables")
    p.add_argument("--output", type=Path, default=ROOT / "output/pdf")
    p = sub.add_parser("prepare-perception")
    p.add_argument("--output", type=Path, default=ROOT / ".cache/perception")
    p = sub.add_parser("calibrate")
    p.add_argument("images", type=Path, nargs="+")
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("register")
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--measurements", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("capture-calibration")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--count", type=int, default=20)
    p.add_argument("--output", type=Path, default=ROOT / "captures/calibration")
    p = sub.add_parser("evaluate")
    p.add_argument("--count", type=int, default=100)
    p.add_argument("--seed", type=int, default=20260906)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("physical-templates")
    p.add_argument(
        "--output", type=Path, default=ROOT / "docs/physical-arm/elexhub/trials"
    )
    p = sub.add_parser("demo")
    p.add_argument("--task", choices=["reach", "pick_place"], default="reach")
    p.add_argument("--condition", choices=list("ABC"), default="C")
    p.add_argument("--destination", type=float, nargs=2, default=[0.10, 0.08])
    p = sub.add_parser("pca-sim")
    p = sub.add_parser("serve")
    p.add_argument("--mode", choices=["sim", "shadow", "hardware"], default="sim")
    p.add_argument("--port", type=int, default=8767)
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--calibration", type=Path)
    p.add_argument(
        "--hand-model",
        type=Path,
        default=ROOT / ".cache/perception/hand_landmarker.task",
    )
    p.add_argument(
        "--speech-model",
        type=Path,
        default=ROOT / ".cache/perception/vosk-model-small-en-us-0.15",
    )
    args = parser.parse_args(argv)
    chosen = args.profile or (
        PHYSICAL
        if args.command == "doctor" or getattr(args, "mode", None) == "hardware"
        else SIMULATION
    )
    config = load_arm_config(chosen)
    try:
        if args.command == "doctor":
            import importlib.metadata

            value = {
                "profile": str(config.source),
                "profile_hash": config.profile_hash,
                "joint_count": config.njoints,
                "actuator_count": config.njoints + 1,
                "calibrated": config.calibrated,
                "runtime": "simulation" if config.simulation else "hardware",
                "geometry": config.profile_data["geometry"]["status"],
                "hardware_ready": not hardware_gaps(config.profile_data)
                and not config.simulation,
                "missing_evidence": hardware_gaps(config.profile_data),
                "Kaggle_access": "none",
                "versions": {
                    p: importlib.metadata.version(p)
                    for p in ["numpy", "mujoco", "mink", "opencv-contrib-python"]
                },
            }
        elif args.command == "sync-sim-profile":
            value = str(simulation_profile(args.profile or PHYSICAL, SIMULATION))
        elif args.command in {"build", "cad"}:
            from .model import build_assets

            value = build_assets(config, args.output)
            if args.command == "cad":
                build_id = str(time.time_ns())
                env = {
                    **os.environ,
                    "ELEXHUB_CAD_DESCRIPTOR": str(
                        (args.output / "cad-description.json").resolve()
                    ),
                }
                env["ELEXHUB_CAD_BUILD_ID"] = build_id
                command = [args.freecad]
                if args.gui_colors:
                    env.update(QT_QPA_PLATFORM="offscreen", ELEXHUB_CAD_QUIT="1")
                elif Path(args.freecad).name == "AppRun":
                    command += ["freecadcmd"]
                subprocess.run(
                    command + [str(ROOT / "cad/build_elexhub_freecad.py")],
                    env=env,
                    check=True,
                )
                value = json.loads((args.output / "CAD_VALIDATION.json").read_text())
                if value.get("build_id") != build_id:
                    raise RuntimeError(
                        "FreeCAD did not produce fresh validation for this build"
                    )
        elif args.command == "printables":
            from .printables import generate

            value = generate(args.output)
        elif args.command == "prepare-perception":
            value = download_models(args.output)
        elif args.command == "calibrate":
            from .vision import calibrate_images

            value = calibrate_images(args.images, args.output)
        elif args.command == "register":
            value = register(args.calibration, args.measurements, args.output)
        elif args.command == "capture-calibration":
            import cv2

            args.output.mkdir(parents=True, exist_ok=True)
            capture = cv2.VideoCapture(args.camera)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            try:
                if not capture.isOpened():
                    raise ValueError("webcam unavailable")
                print(
                    "Move and tilt the printed calibration board; one image is saved every 2 seconds.",
                    flush=True,
                )
                for index in range(args.count):
                    end = time.monotonic() + 2
                    while time.monotonic() < end:
                        ok, frame = capture.read()
                    if not ok:
                        raise ValueError("camera read failed")
                    cv2.imwrite(
                        str(args.output / f"view-{time.time_ns()}-{index:02d}.png"),
                        frame,
                    )
                    print(f"Saved view {index + 1}/{args.count}", flush=True)
            finally:
                capture.release()
            value = {"captured": args.count, "path": str(args.output)}
        elif args.command == "physical-templates":
            from .evaluation import physical_templates

            value = str(physical_templates(args.output))
        elif args.command == "evaluate":
            from .evaluation import evaluate, frozen_suite

            if not config.simulation:
                raise ValueError("this evaluator is simulation-only")
            value = evaluate(config, args.output, frozen_suite(args.count, args.seed))
        elif args.command == "pca-sim":
            from robo_arm_driver.pca_sim import serve

            if not config.simulation:
                raise ValueError("PTY simulator needs the explicit simulation profile")
            return serve(config)
        elif args.command in {"serve", "demo"}:
            os.environ.setdefault("MUJOCO_GL", "egl")
            from .simulation import SimArm

            if args.command == "demo":
                from .interaction import TaskExecutor, parse_instruction

                if not config.simulation:
                    raise ValueError("demo is simulation-only")
                arm = SimArm(config, ROOT / "assets/elexhub/provisional", rebuild=False)
                try:
                    task = TaskExecutor(arm)
                    intent = parse_instruction(
                        "reach here"
                        if args.task == "reach"
                        else "move this red box here",
                        args.destination,
                    )
                    preview = task.preview(intent)
                    value = task.execute(
                        preview["preview_id"],
                        workspace_clear=True,
                        condition=args.condition,
                    )
                finally:
                    arm.close()
            else:
                import uvicorn

                from .web import create_app

                camera = None
                if args.mode != "sim":
                    if args.calibration is None:
                        raise ValueError("webcam modes require --calibration")
                    if args.mode == "hardware" and (
                        config.simulation or not config.calibrated
                    ):
                        raise ValueError(
                            "hardware mode is blocked until the measured physical profile passes its gates"
                        )
                    from .capture import Webcam

                    camera = Webcam(args.camera, args.calibration, args.hand_model)
                if args.mode == "hardware":
                    from .hardware import HardwareArm

                    arm = HardwareArm(config, ROOT / ".cache/measured-model", camera)
                else:
                    if not config.simulation:
                        raise ValueError(
                            "simulation/shadow mode requires a simulation profile"
                        )
                    arm = SimArm(
                        config, ROOT / "assets/elexhub/provisional", rebuild=False
                    )
                speech = None
                if args.speech_model.is_dir():
                    from .speech import Speech

                    speech = Speech(args.speech_model)
                app = create_app(
                    arm,
                    ROOT,
                    mode=args.mode,
                    webcam=camera,
                    speech=speech,
                    hand_model=args.hand_model,
                )
                print(
                    f"Local {args.mode} console: http://127.0.0.1:{args.port} (hardware starts disabled)",
                    flush=True,
                )
                uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")
                return 0
        print(json.dumps(value, indent=2, default=str, allow_nan=False))
        return 0
    except (ValueError, RuntimeError) as exc:
        parser.exit(2, f"Refused: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())

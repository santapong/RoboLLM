"""Package only the Elexhub robot model and verified review evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODEL_FILES = (
    "model.html",
    "elexhub.FCStd",
    "elexhub.step",
    "elexhub.glb",
    "elexhub.urdf",
    "elexhub-robot.xml",
    "cad-description.json",
    "CAD_VALIDATION.json",
    "MODEL_STATUS.json",
    "APPEARANCE_PROVENANCE.json",
)
REVIEW_VIEWS = {"angle", "front", "side", "top", "back", "claw"}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def bundle(model_dir, review_dir, measurement_note, output):
    descriptor = json.loads((model_dir / "cad-description.json").read_text())
    cad = json.loads((model_dir / "CAD_VALIDATION.json").read_text())
    if cad["descriptor_sha256"] != digest(
        (model_dir / "cad-description.json").read_bytes()
    ):
        raise ValueError("CAD validation does not match the current model descriptor")
    files = {name: model_dir / name for name in MODEL_FILES}
    # Ship the watertight material meshes actually used by URDF/MuJoCo. The
    # legacy combined-body OBJ compatibility files weld touching materials in
    # some importers and are not needed to load any model in this package.
    for records in descriptor["visual_meshes"].values():
        for record in records:
            name = f"meshes/{record['name']}.obj"
            files[name] = model_dir / name
    files["docs/MEASUREMENTS-PHOTO-MODEL.md"] = measurement_note
    render_manifests = sorted(review_dir.rglob("RENDER_MANIFEST.json"))
    if not render_manifests:
        raise ValueError("no robot-only render manifest found in the review directory")
    for path in render_manifests:
        manifest = json.loads(path.read_text())
        if (
            not manifest.get("robot_only")
            or manifest.get("model_file") != "elexhub-robot.xml"
        ):
            raise ValueError("the model bundle accepts only robot-only review renders")
        for name, expected in manifest["artifact_sha256"].items():
            if name not in files or digest((model_dir / name).read_bytes()) != expected:
                raise ValueError(f"review artifact is stale or unsupported: {name}")
        relative = path.relative_to(review_dir)
        files[str(Path("review") / relative)] = path
        for view in manifest["rendered_views"]:
            if view not in REVIEW_VIEWS:
                raise ValueError(f"unsupported robot review view: {view}")
            name = view + ".png"
            image = path.with_name(name)
            if digest(image.read_bytes()) != manifest["render_sha256"][name]:
                raise ValueError(f"review image is stale: {image.name}")
            files[str(Path("review") / relative.parent / name)] = image
    for name in [
        "VALIDATION.json",
        "unit-tests.txt",
        "lint.txt",
        "REVIEW.md",
        "README.md",
    ]:
        path = review_dir / name
        if path.is_file():
            files["review/" + name] = path
    payloads = {name: path.read_bytes() for name, path in sorted(files.items())}
    payloads["START_HERE.md"] = (
        b"# Elexhub robot model\n\n"
        b"Unzip this folder and open model.html to rotate and zoom the robot. "
        b"The FreeCAD, STEP and GLB links work beside the viewer.\n\n"
        b"elexhub.FCStd is editable CAD; elexhub.step is an exchange model; "
        b"elexhub.glb is a static Y-up model. CAD, URDF and MuJoCo use Z-up. "
        b"elexhub-robot.xml contains only the fixed-base robot and its existing "
        b"five arm joints plus one coupled gripper drive. Keep meshes/ beside "
        b"the MuJoCo and URDF files.\n\n"
        b"Dimensions remain provisional. See docs/MEASUREMENTS-PHOTO-MODEL.md "
        b"before supplying measured lengths. Review-only poses are recorded "
        b"in review/; the CAD and browser exports retain the profile home pose. "
        b"This package contains no room photographs or physical calibration evidence.\n"
    )
    manifest = {
        "schema_version": 1,
        "content": "robot-only model",
        "geometry_status": descriptor["status"],
        "profile_hash": descriptor["profile_hash"],
        "appearance_revision": descriptor["appearance"]["revision"],
        "hardware_ready": False,
        "physical_evidence": False,
        "files": {
            name: {"sha256": digest(data), "bytes": len(data)}
            for name, data in sorted(payloads.items())
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    try:
        with zipfile.ZipFile(
            temporary, "w", compression=zipfile.ZIP_DEFLATED
        ) as archive:
            for name, data in sorted(payloads.items()):
                archive.writestr(name, data)
            archive.writestr(
                "BUNDLE_MANIFEST.json", json.dumps(manifest, indent=2) + "\n"
            )
        with zipfile.ZipFile(temporary) as archive:
            if archive.testzip() is not None:
                raise ValueError("model bundle failed ZIP integrity validation")
            for name, expected in manifest["files"].items():
                if digest(archive.read(name)) != expected["sha256"]:
                    raise ValueError(f"model bundle hash mismatch: {name}")
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    sha = digest(output.read_bytes())
    output.with_suffix(".sha256").write_text(f"{sha}  {output.name}\n")
    return {
        "path": str(output),
        "files": len(payloads) + 1,
        "bytes": output.stat().st_size,
        "sha256": sha,
        "verified": True,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model-dir", type=Path, default=ROOT / "assets/elexhub/provisional"
    )
    parser.add_argument(
        "--review-dir",
        type=Path,
        default=ROOT / "output/elexhub-photo-model-review-2026-09-08",
    )
    parser.add_argument(
        "--measurement-note",
        type=Path,
        default=ROOT / "docs/physical-arm/elexhub/MEASUREMENTS-PHOTO-MODEL.md",
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/elexhub-robot-model.zip"
    )
    args = parser.parse_args(argv)
    print(
        json.dumps(
            bundle(args.model_dir, args.review_dir, args.measurement_note, args.output),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()

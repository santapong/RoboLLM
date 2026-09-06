"""Package the local simulator, CAD and reference files; exclude personal data."""

from __future__ import annotations

import hashlib
import json
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    directories = [
        "assets/elexhub/provisional",
        "docs/physical-arm/elexhub",
        "output/pdf",
        "output/elexhub-evaluation-2026-09-06",
        "src/robollm/elexhub",
        "ros2/robo_arm_driver",
        "hardware/firmware/elexhub_pca9685",
        "examples/elexhub",
    ]
    individual = [
        "src/robollm/__init__.py",
        "src/robollm/bridge.py",
        "hardware/arm_serial.py",
        "hardware/generate_firmware_config.py",
        "cad/build_elexhub_freecad.py",
        "requirements/elexhub.txt",
        "requirements/elexhub-test.txt",
        "requirements/ros-constraints.txt",
        "tests/conftest.py",
        "tests/unit/test_elexhub.py",
        "output/final-unit-tests.txt",
        "output/final-lint.txt",
    ]
    files = {ROOT / p for p in individual}
    for directory in directories:
        files.update(p for p in (ROOT / directory).rglob("*") if p.is_file())
    files = sorted(
        p
        for p in files
        if p.exists()
        and "__pycache__" not in p.parts
        and p.suffix not in {".pyc", ".FCBak"}
    )
    output = ROOT / "output/elexhub-bundle.zip"
    manifest = {
        "schema_version": 1,
        "geometry": "provisional",
        "hardware_ready": False,
        "physical_evidence": False,
        "files": {},
    }
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            relative = str(path.relative_to(ROOT))
            data = path.read_bytes()
            manifest["files"][relative] = {
                "sha256": hashlib.sha256(data).hexdigest(),
                "bytes": len(data),
            }
            archive.writestr(relative, data)
        archive.writestr("BUNDLE_MANIFEST.json", json.dumps(manifest, indent=2) + "\n")
        archive.writestr(
            "START_HERE.md",
            "# Elexhub experiment pack\n\nStart with docs/physical-arm/elexhub/README.md.\n\nThe CAD and physics are provisional, not measurements of your arm. Hardware remains disabled. This pack contains no training notebooks, model checkpoints, camera recordings or credentials. Install the listed dependencies in a separate Python 3.12 environment to run examples/elexhub/run.py. FreeCAD and perception-model runtimes are downloaded separately; generated CAD and reference sheets are already included.\n",
        )
    sha = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(".sha256").write_text(f"{sha}  {output.name}\n")
    print(
        json.dumps(
            {
                "path": str(output),
                "files": len(files) + 2,
                "bytes": output.stat().st_size,
                "sha256": sha,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()

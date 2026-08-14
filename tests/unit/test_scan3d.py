"""Native regression coverage for the CPU scan-to-print tail."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
trimesh = pytest.importorskip("trimesh")

ROOT = Path(__file__).resolve().parents[2]


def _load(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


scale_mat = _load("scan3d_scale_mat", "scan3d/scale_mat.py")
mesh_to_print = _load("scan3d_mesh_to_print", "scan3d/mesh_to_print.py")


def test_charuco_dependency_and_board_geometry_are_available():
    assert hasattr(cv2, "aruco")
    board = scale_mat.get_board(scale_mat.SQUARE_MM)
    assert tuple(board.getChessboardSize()) == (
        scale_mat.SQUARES_X,
        scale_mat.SQUARES_Y,
    )
    assert board.getSquareLength() == pytest.approx(scale_mat.SQUARE_MM)


def test_metric_scale_self_test_recovers_known_units():
    assert scale_mat.self_test() == 0


def test_largest_component_drops_disconnected_debris():
    body = trimesh.creation.box(extents=(1.0, 2.0, 3.0))
    debris = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
    debris.apply_translation((5.0, 0.0, 0.0))
    combined = trimesh.util.concatenate((body, debris))

    cleaned = mesh_to_print.largest_component(combined)

    assert len(cleaned.faces) == len(body.faces)
    assert cleaned.extents == pytest.approx(body.extents)


@pytest.mark.parametrize("scale_source", ["height", "manifest"])
def test_mesh_cli_exports_watertight_true_scale_stl(tmp_path, scale_source):
    source = tmp_path / "object.ply"
    output = tmp_path / "object_print.stl"
    mesh = trimesh.creation.box(extents=(1.0, 2.0, 3.0))
    mesh.export(source)

    command = [
        sys.executable,
        str(ROOT / "scan3d/mesh_to_print.py"),
        str(source),
        "-o",
        str(output),
    ]
    if scale_source == "height":
        command.extend(["--height-mm", "90"])
    else:
        (tmp_path / "scale.json").write_text(
            json.dumps({"schema": "scan3d.scale.v1", "mm_per_unit": 30.0})
        )

    completed = subprocess.run(command, capture_output=True, text=True, check=False)

    assert completed.returncode == 0, completed.stderr
    exported = trimesh.load(output, force="mesh")
    assert exported.is_watertight
    assert np.min(exported.bounds[:, 2]) == pytest.approx(0.0, abs=1e-6)
    assert exported.extents[2] == pytest.approx(90.0, abs=0.1)

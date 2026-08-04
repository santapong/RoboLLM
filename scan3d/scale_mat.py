#!/usr/bin/env python3
"""scale_mat.py — true metric scale for photogrammetry via a printed ChArUco mat.

Photogrammetry (Route C) is scale-free: the reconstruction is correct up to an
unknown global scale. Put a printed ChArUco mat under/next to the object while
shooting and this tool recovers the real millimetres automatically — no more
calliper + --height-mm guessing.

Modes:
  make   — generate the mat image to print at EXACT size (A4, 300 DPI)
             python3 scale_mat.py make -o ../assets/scale_mat.png
  solve  — after COLMAP sparse SfM: detect mat corners in the photos,
           triangulate them through the recovered poses, compare with the
           board's known geometry -> writes <session>/scale.json
             python3 scale_mat.py solve --session ../assets/scan/mug
  --self-test — verify the triangulation/scale math on synthetic cameras

Board: 7x5 squares, DICT_5X5_100, 30 mm squares / 22 mm markers (fits A4 with
margins). PRINT AT 100% / "actual size", then MEASURE one square with a ruler —
if it isn't 30.0 mm pass the real value via --square-mm to solve.

Deps: numpy, opencv-contrib-python (<5 in the repo venv: -c constraints.txt).
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

SQUARES_X, SQUARES_Y = 7, 5
SQUARE_MM = 30.0
MARKER_MM = 22.0
DICT_NAME = "DICT_5X5_100"


def get_board(square_mm: float):
    import cv2
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), square_mm, MARKER_MM * square_mm / SQUARE_MM, dic)
    return board


def cmd_make(args) -> int:
    import cv2
    dpi = 300
    mm2px = dpi / 25.4
    w = int(SQUARES_X * SQUARE_MM * mm2px)
    h = int(SQUARES_Y * SQUARE_MM * mm2px)
    img = get_board(SQUARE_MM).generateImage((w, h), marginSize=0)
    # centre on A4 (210 x 297 mm) landscape so 100%-scale printing is unambiguous
    page = np.full((int(210 * mm2px), int(297 * mm2px)), 255, np.uint8)
    y0 = (page.shape[0] - h) // 2
    x0 = (page.shape[1] - w) // 2
    page[y0:y0 + h, x0:x0 + w] = img
    cv2.putText(page, f"scan3d scale mat  {SQUARES_X}x{SQUARES_Y} {DICT_NAME}"
                f"  square={SQUARE_MM:.0f}mm  PRINT AT 100% AND MEASURE A SQUARE",
                (x0, y0 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
    out = Path(args.out)
    cv2.imwrite(str(out), page)
    print(f"wrote {out} ({page.shape[1]}x{page.shape[0]} px @ {dpi} DPI, A4 landscape)")
    print("Print at 100% scale, measure one square, keep the mat FLAT and in-frame.")
    return 0


# --------------------------------------------------------------------------
# solve: COLMAP text model + photos -> metric scale
# --------------------------------------------------------------------------

def read_colmap_text(sparse_dir: Path):
    """Minimal cameras.txt + images.txt parser -> {name: (K, dist, R, t)}."""
    cams = {}
    for line in (sparse_dir / "cameras.txt").read_text().splitlines():
        if line.startswith("#") or not line.strip():
            continue
        f = line.split()
        cam_id, model = int(f[0]), f[1]
        p = list(map(float, f[4:]))
        if model == "OPENCV":
            K = np.array([[p[0], 0, p[2]], [0, p[1], p[3]], [0, 0, 1]])
            dist = np.array(p[4:8])
        elif model == "PINHOLE":
            K = np.array([[p[0], 0, p[2]], [0, p[1], p[3]], [0, 0, 1]])
            dist = np.zeros(4)
        elif model in ("SIMPLE_PINHOLE", "SIMPLE_RADIAL"):
            K = np.array([[p[0], 0, p[1]], [0, p[0], p[2]], [0, 0, 1]])
            dist = np.array([p[3], 0, 0, 0]) if model == "SIMPLE_RADIAL" else np.zeros(4)
        else:
            raise SystemExit(f"unsupported camera model {model}")
        cams[cam_id] = (K, dist)

    views = {}
    lines = [l for l in (sparse_dir / "images.txt").read_text().splitlines()
             if not l.startswith("#") and l.strip()]
    for meta in lines[0::2]:  # every image has a metadata line + a points2D line
        f = meta.split()
        qw, qx, qy, qz = map(float, f[1:5])
        t = np.array(list(map(float, f[5:8])))
        cam_id, name = int(f[8]), f[9]
        # COLMAP: x_cam = R x_world + t, R from Hamilton quaternion
        R = quat_to_R(qw, qx, qy, qz)
        K, dist = cams[cam_id]
        views[name] = (K, dist, R, t)
    return views


def quat_to_R(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def triangulate_dlt(obs):
    """Multi-view DLT: obs = [(P 3x4, (u,v)), ...] -> 3D point in model units."""
    A = []
    for P, (u, v) in obs:
        A.append(u * P[2] - P[0])
        A.append(v * P[2] - P[1])
    _, _, Vt = np.linalg.svd(np.asarray(A))
    X = Vt[-1]
    return X[:3] / X[3]


def solve_scale(detections, views, square_mm):
    """detections: {image_name: {corner_id: (u,v)}} -> (mm_per_unit, stats).

    Triangulates every ChArUco corner seen in >=2 registered views, then takes
    the median over corner pairs of known_mm / model_distance.
    """
    import itertools
    # gather observations per corner id, with undistorted-normalized reprojection
    per_corner = {}
    for name, corners in detections.items():
        if name not in views:
            continue
        K, dist, R, t = views[name]
        P = K @ np.hstack([R, t.reshape(3, 1)])
        for cid, uv in corners.items():
            per_corner.setdefault(cid, []).append((P, uv, K, dist))

    import cv2
    pts3d = {}
    for cid, obs in per_corner.items():
        if len(obs) < 2:
            continue
        und = []
        for P, uv, K, dist in obs:
            u = cv2.undistortPoints(np.array([[uv]], np.float64), K, dist, P=K)[0, 0]
            und.append((P, (u[0], u[1])))
        pts3d[cid] = triangulate_dlt(und)

    if len(pts3d) < 2:
        return None, {"corners_3d": len(pts3d)}

    # board-truth corner grid: interior corners of the squares, spacing = square_mm
    def truth(cid):
        return np.array([(cid % (SQUARES_X - 1)) * square_mm,
                         (cid // (SQUARES_X - 1)) * square_mm, 0.0])

    ratios = []
    for a, b in itertools.combinations(sorted(pts3d), 2):
        d_model = np.linalg.norm(pts3d[a] - pts3d[b])
        d_true = np.linalg.norm(truth(a) - truth(b))
        if d_model > 1e-9:
            ratios.append(d_true / d_model)
    ratios = np.array(ratios)
    mm_per_unit = float(np.median(ratios))
    spread = float(np.percentile(np.abs(ratios / mm_per_unit - 1), 90) * 100)
    return mm_per_unit, {"corners_3d": len(pts3d), "pairs": len(ratios),
                         "p90_spread_pct": round(spread, 2)}


def cmd_solve(args) -> int:
    import cv2
    sess = Path(args.session)
    images_dir = sess / "images"
    sparse = sess / "sparse_txt"
    if not sparse.exists():
        print(f"error: {sparse} missing — reconstruct_cpu.sh exports it "
              f"(colmap model_converter --output_type TXT)", file=sys.stderr)
        return 1

    views = read_colmap_text(sparse)
    detector = cv2.aruco.CharucoDetector(get_board(args.square_mm))
    detections = {}
    for img_path in sorted(images_dir.iterdir()):
        img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        corners, ids, _, _ = detector.detectBoard(img)
        if ids is not None and len(ids) >= 4:
            detections[img_path.name] = {
                int(i): tuple(c) for i, c in zip(ids.ravel(), corners.reshape(-1, 2))}
    print(f"mat detected in {len(detections)}/{len(views)} registered views")
    if len(detections) < 2:
        print("error: need the mat visible in >=2 registered photos", file=sys.stderr)
        return 1

    mm_per_unit, stats = solve_scale(detections, views, args.square_mm)
    if mm_per_unit is None:
        print(f"error: too few triangulated corners ({stats})", file=sys.stderr)
        return 1

    out = sess / "scale.json"
    out.write_text(json.dumps({
        "schema": "scan3d.scale.v1", "mm_per_unit": mm_per_unit,
        "square_mm": args.square_mm, "board": f"{SQUARES_X}x{SQUARES_Y} {DICT_NAME}",
        "views_with_mat": len(detections), **stats}, indent=2) + "\n")
    print(f"mm_per_unit = {mm_per_unit:.4f}  (p90 spread {stats['p90_spread_pct']}% "
          f"over {stats['pairs']} corner pairs)")
    print(f"wrote {out}")
    if stats["p90_spread_pct"] > 3:
        print("WARNING: spread >3% — mat not flat, mis-scaled print, or bad poses",
              file=sys.stderr)
    return 0


# --------------------------------------------------------------------------

def self_test() -> int:
    """Synthetic cameras look at the board; solve must recover a known scale."""
    import cv2
    rng = np.random.default_rng(7)
    SCALE = 0.013  # model units per mm: reconstruction is 1/76.9 of real size
    n_corners = (SQUARES_X - 1) * (SQUARES_Y - 1)
    truth_mm = np.array([[(i % (SQUARES_X - 1)) * SQUARE_MM,
                          (i // (SQUARES_X - 1)) * SQUARE_MM, 0.0]
                         for i in range(n_corners)])
    world = truth_mm * SCALE  # the "reconstruction" lives in scaled units

    K = np.array([[1000.0, 0, 640], [0, 1000.0, 480], [0, 0, 1]])
    dist = np.zeros(4)
    views, detections = {}, {}
    center = world.mean(axis=0)
    for k in range(6):
        ang = k * np.pi / 5
        cam_pos = center + np.array([4 * np.cos(ang), 4 * np.sin(ang), 5.0])
        z = center - cam_pos
        z /= np.linalg.norm(z)
        x = np.cross(z, [0, 0, 1.0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        R = np.stack([x, y, z])           # world->cam rotation
        t = -R @ cam_pos
        views[f"v{k}"] = (K, dist, R, t)
        P = K @ np.hstack([R, t.reshape(3, 1)])
        det = {}
        for cid, X in enumerate(world):
            uv = P @ np.append(X, 1.0)
            uv = uv[:2] / uv[2] + rng.normal(0, 0.3, 2)  # 0.3 px detection noise
            det[cid] = (uv[0], uv[1])
        detections[f"v{k}"] = det

    mm_per_unit, stats = solve_scale(detections, views, SQUARE_MM)
    expect = 1.0 / SCALE
    err = abs(mm_per_unit - expect) / expect * 100
    print(f"self-test: expected {expect:.3f} mm/unit, got {mm_per_unit:.3f} "
          f"({err:.3f}% error, {stats})")
    assert err < 0.5, "scale recovery error above 0.5%"
    print("self-test PASSED")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true")
    sub = ap.add_subparsers(dest="cmd")
    mk = sub.add_parser("make", help="generate printable mat image")
    mk.add_argument("-o", "--out", default="scale_mat.png")
    sv = sub.add_parser("solve", help="recover metric scale from a session")
    sv.add_argument("--session", required=True, help="assets/scan/<name> dir")
    sv.add_argument("--square-mm", type=float, default=SQUARE_MM,
                    help="MEASURED square size on your printout (default 30.0)")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.cmd == "make":
        return cmd_make(args)
    if args.cmd == "solve":
        return cmd_solve(args)
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())

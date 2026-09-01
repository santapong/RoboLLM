#!/usr/bin/env python3
"""visual_hull.py — turn turntable webcam frames into a 3D mesh, CPU-only.

No GPU, no camera calibration. It works by "shape-from-silhouette": each frame
is a shadow of the object; the 3D shape is the intersection of all those shadows
(the visual hull). Great for a bad webcam because it only needs the OUTLINE of the
object, not sharp detail.

Assumptions (match them with capture.py --turntable):
  - the object rotated a full, even 360° across the frames
  - the camera stayed put, looking roughly horizontally at the object
  - you can separate object from background — best: capture a background.jpg first

Limitations: a visual hull cannot see concavities (a cup's inside looks filled).
For higher fidelity use the photogrammetry path (reconstruct.sh) on a GPU.

    python3 visual_hull.py --session mug --height-mm 95
Output: ../assets/scan/<session>/<session>_hull.obj  (+ .ply)
"""

import argparse
import glob
import os

import cv2
import numpy as np
import trimesh
from segmentation import silhouette
from skimage import measure

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", default="object")
    ap.add_argument("--res", type=int, default=128, help="voxel grid resolution")
    ap.add_argument(
        "--height-mm", type=float, default=100.0, help="real object height for scaling"
    )
    args = ap.parse_args()

    sess = os.path.join(HERE, "..", "assets", "scan", args.session)
    files = sorted(glob.glob(os.path.join(sess, "images", "*.jpg")))
    if len(files) < 8:
        raise SystemExit(
            f"need >=8 frames, found {len(files)} in {sess}/images — "
            f"run capture.py --turntable 36 --session {args.session}"
        )
    bg_path = os.path.join(sess, "background.jpg")
    bg = cv2.imread(bg_path) if os.path.exists(bg_path) else None
    print(f"{len(files)} frames | background: {'yes' if bg is not None else 'no'}")

    # 1) silhouettes + geometry estimate (rotation-axis column, base row, extents)
    masks, centroids, bottoms, tops = [], [], [], []
    for f in files:
        img = cv2.imread(f)
        m = silhouette(img, bg)
        ys, xs = np.nonzero(m)
        if xs.size == 0:
            continue
        masks.append(m)
        centroids.append(xs.mean())
        bottoms.append(ys.max())
        tops.append(ys.min())
    if len(masks) < 8:
        raise SystemExit("too few usable silhouettes — improve lighting/background")

    cx = float(np.median(centroids))
    base_v = float(np.median(bottoms))
    half_w = float(
        np.percentile(
            [
                max(abs(np.nonzero(m)[1].max() - cx), abs(cx - np.nonzero(m)[1].min()))
                for m in masks
            ],
            95,
        )
    )
    height_px = float(np.percentile([base_v - t for t in tops], 95))
    z_max = height_px / half_w
    print(
        f"axis col={cx:.0f}  base row={base_v:.0f}  half-width={half_w:.0f}px  z_max={z_max:.2f}"
    )

    # 2) build a voxel grid and carve with every silhouette (orthographic hull)
    R = args.res
    xs = np.linspace(-1, 1, R)
    zs = np.linspace(0, z_max, max(4, int(R * z_max / 2)))
    X, Y, Z = np.meshgrid(xs, xs, zs, indexing="ij")  # x, y(depth), z(height)
    occ = np.ones(X.shape, dtype=bool)
    N = len(masks)
    H, W = masks[0].shape
    for i, m in enumerate(masks):
        th = 2 * np.pi * i / N
        xr = X * np.cos(th) - Y * np.sin(th)  # rotate about vertical axis
        u = np.round(cx + xr * half_w).astype(np.int64)
        v = np.round(base_v - Z * half_w).astype(np.int64)
        inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        hit = np.zeros(X.shape, dtype=bool)
        uu, vv = np.clip(u, 0, W - 1), np.clip(v, 0, H - 1)
        hit[inside] = m[vv[inside], uu[inside]]
        occ &= hit
        if not occ.any():
            raise SystemExit(
                "carved to nothing — silhouettes disagree; check background/lighting"
            )
    print(f"occupied voxels: {int(occ.sum())}")

    # 3) surface mesh via marching cubes, scaled to real millimetres, then metres
    verts, faces, _, _ = measure.marching_cubes(occ.astype(np.float32), level=0.5)
    # marching cubes returns index coords; map back to world units
    verts[:, 0] = np.interp(verts[:, 0], [0, R - 1], [-1, 1])
    verts[:, 1] = np.interp(verts[:, 1], [0, R - 1], [-1, 1])
    verts[:, 2] = np.interp(verts[:, 2], [0, len(zs) - 1], [0, z_max])
    mm_per_world = args.height_mm / z_max
    verts *= mm_per_world / 1000.0  # -> metres (URDF units)

    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    mesh.remove_duplicate_faces() if hasattr(mesh, "remove_duplicate_faces") else None
    mesh.fill_holes()
    obj = os.path.join(sess, f"{args.session}_hull.obj")
    ply = os.path.join(sess, f"{args.session}_hull.ply")
    mesh.export(obj)
    mesh.export(ply)
    dims = mesh.extents * 1000.0
    print(
        f"\nmesh: {len(mesh.vertices)} verts, {len(mesh.faces)} faces, "
        f"~{dims[0]:.0f}×{dims[1]:.0f}×{dims[2]:.0f} mm"
    )
    print(f"saved → {obj}")
    print(f"Next: URDF link →  python3 mesh_to_urdf.py {obj} --name {args.session}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""masks.py — build COLMAP masks so a FIXED camera can reconstruct a turntable.

Route E. Structure-from-Motion constrains *relative* motion, so masking away the
static room leaves only the object — and a rotating object seen by a fixed camera
is geometrically the same problem as a fixed object seen by an orbiting camera.
Without the masks, COLMAP solves the room and throws the object away as a moving
outlier.

COLMAP's contract (see scan3d/SDD.md §4.2): one PNG per image, named after the
FULL image filename plus ".png" (frame_0007.jpg -> frame_0007.jpg.png), same
width and height, and pixels of zero intensity are ignored. So object -> 255,
background -> 0.

Segmentation is `segmentation.silhouette()`, the same one Route A carves with.

    python3 masks.py --session mug
    ./reconstruct_cpu.sh mug          # picks masks/ up automatically

Requires assets/scan/<session>/background.jpg — shoot it first with
`capture.py --background` on the empty scene.
"""

from __future__ import annotations

import argparse
import glob
import os

import cv2
import numpy as np
from segmentation import silhouette

HERE = os.path.dirname(os.path.abspath(__file__))
SCAN_ROOT = os.path.join(HERE, "..", "assets", "scan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="object")
    ap.add_argument(
        "--min-coverage",
        type=float,
        default=0.01,
        help="reject a mask covering less than this fraction of the frame",
    )
    ap.add_argument(
        "--max-coverage",
        type=float,
        default=0.90,
        help="reject a mask covering more than this fraction of the frame",
    )
    args = ap.parse_args()

    sess = os.path.join(SCAN_ROOT, args.session)
    images = os.path.join(sess, "images")
    out = os.path.join(sess, "masks")
    files = sorted(
        f
        for ext in ("jpg", "jpeg", "png")
        for f in glob.glob(os.path.join(images, f"*.{ext}"))
    )
    if not files:
        raise SystemExit(f"no images at {images} — run capture.py first")

    bg_path = os.path.join(sess, "background.jpg")
    bg = cv2.imread(bg_path) if os.path.exists(bg_path) else None
    if bg is None:
        # The HSV fallback exists, but on a turntable it is a coin flip: without
        # a background frame there is nothing to subtract, and a leaky mask
        # readmits the static room, which pins COLMAP to a zero-baseline
        # solution where nothing can be triangulated.
        print(
            f"WARNING: no {bg_path} — falling back to HSV saturation.\n"
            "         Shoot `capture.py --background` on the empty scene and "
            "rerun; Route E depends on a clean mask."
        )

    os.makedirs(out, exist_ok=True)
    written, suspect = 0, []
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        if bg is not None and bg.shape != img.shape:
            raise SystemExit(
                f"background.jpg is {bg.shape[1]}x{bg.shape[0]} but {os.path.basename(f)} "
                f"is {img.shape[1]}x{img.shape[0]} — recapture both with one camera setting"
            )
        m = silhouette(img, bg)
        coverage = float(m.mean())
        if not (args.min_coverage <= coverage <= args.max_coverage):
            suspect.append((os.path.basename(f), coverage))
        # COLMAP ignores zero-intensity pixels; keep the object at full white.
        cv2.imwrite(
            os.path.join(out, os.path.basename(f) + ".png"),
            (m.astype(np.uint8) * 255),
        )
        written += 1

    print(f"wrote {written} masks → {out}")
    if suspect:
        print(
            f"WARNING: {len(suspect)} mask(s) outside "
            f"[{args.min_coverage:.0%}, {args.max_coverage:.0%}] frame coverage — "
            "usually bad lighting, a moved camera, or the object leaving frame:"
        )
        for name, cov in suspect[:10]:
            print(f"  {name}: {cov:.1%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

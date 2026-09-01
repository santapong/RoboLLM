"""Object/background segmentation shared by the turntable routes.

Route A (`visual_hull.py`) carves silhouettes; Route E (`masks.py`) hands the
same masks to COLMAP so a fixed camera can reconstruct a rotating object. Both
need one segmentation, so it lives here rather than in either of them.

Deliberately depends on cv2 + numpy only — `visual_hull.py` additionally needs
trimesh and skimage, which are not installed everywhere the masks are.
"""

from __future__ import annotations

import cv2
import numpy as np


def silhouette(frame: np.ndarray, bg: np.ndarray | None) -> np.ndarray:
    """Return a boolean mask of the object (True = object).

    With a background frame this is a straight difference; without one it falls
    back to HSV saturation, which assumes a plain, contrasting backdrop.

    Only the largest blob survives, which drops speckle — and means anything
    touching the object is absorbed into it. That is why the turntable routes
    require the object to be released onto the turntable, never held.
    """
    if bg is not None:
        diff = cv2.absdiff(frame, bg)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, m = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        # no background reference: assume a plain, contrasting backdrop
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        _, m = cv2.threshold(hsv[:, :, 1], 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    # keep only the largest blob (drops speckle)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        big = max(cnts, key=cv2.contourArea)
        m = np.zeros_like(m)
        cv2.drawContours(m, [big], -1, 255, cv2.FILLED)
    return m > 0


__all__ = ["silhouette"]

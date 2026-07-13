#!/usr/bin/env python3
"""capture.py — grab images from your laptop webcam for 3D scanning.

Two modes:
  Snapshots (default): a preview window opens. Press SPACE to save a frame,
      Q or ESC to finish. Move around the object between snaps.
  Turntable (--turntable N): saves N frames automatically at a steady interval
      while YOU rotate the object on a plate/lazy-Susan a full 360°. Best for the
      CPU-only visual-hull reconstructor (visual_hull.py).

Also grab a clean background shot first with --background: put NOTHING in front of
the camera and it saves one frame — visual_hull.py uses it to cut out the object.

Output goes to ../assets/scan/<session>/images/  (+ background.jpg).

Bad-webcam tips (they matter more than resolution):
  - even, bright, diffuse light; avoid glare and hard shadows
  - a plain, contrasting background (a sheet of paper / cloth)
  - keep the object fully in frame the whole way around
  - matte, textured objects scan far better than shiny/plain ones

    python3 capture.py --background          # once, empty scene
    python3 capture.py --turntable 36 --session mug
"""
import argparse
import os
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
SCAN_ROOT = os.path.join(HERE, "..", "assets", "scan")


def open_cam(index: int):
    cam = cv2.VideoCapture(index)
    if not cam.isOpened():
        raise SystemExit(f"could not open /dev/video{index} — try --camera 1")
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cam


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0, help="/dev/videoN index")
    ap.add_argument("--session", default="object", help="name for this scan")
    ap.add_argument("--turntable", type=int, metavar="N",
                    help="auto-save N frames while you rotate the object 360°")
    ap.add_argument("--interval", type=float, default=0.7, help="seconds between turntable frames")
    ap.add_argument("--background", action="store_true",
                    help="save a single empty-scene background.jpg and exit")
    args = ap.parse_args()

    sess = os.path.join(SCAN_ROOT, args.session)
    imgs = os.path.join(sess, "images")
    os.makedirs(imgs, exist_ok=True)
    cam = open_cam(args.camera)

    if args.background:
        for _ in range(10):  # let exposure settle
            cam.read(); time.sleep(0.05)
        ok, frame = cam.read()
        if ok:
            path = os.path.join(sess, "background.jpg")
            cv2.imwrite(path, frame)
            print(f"saved background → {path}")
        cam.release()
        return

    n = 0
    if args.turntable:
        print(f"Turntable mode: rotate the object slowly & steadily 360°. "
              f"Capturing {args.turntable} frames…")
        for _ in range(10):
            cam.read(); time.sleep(0.05)
        while n < args.turntable:
            ok, frame = cam.read()
            if not ok:
                break
            path = os.path.join(imgs, f"frame_{n:03d}.jpg")
            cv2.imwrite(path, frame)
            n += 1
            print(f"  {n}/{args.turntable}")
            cv2.imshow("capturing (turntable)", frame)
            if cv2.waitKey(1) in (27, ord("q")):
                break
            time.sleep(args.interval)
    else:
        print("Snapshot mode: SPACE = save, Q/ESC = done")
        while True:
            ok, frame = cam.read()
            if not ok:
                break
            view = frame.copy()
            cv2.putText(view, f"saved: {n}   SPACE=snap  Q=quit", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("capture", view)
            k = cv2.waitKey(1) & 0xFF
            if k == 32:  # SPACE
                path = os.path.join(imgs, f"frame_{n:03d}.jpg")
                cv2.imwrite(path, frame)
                n += 1
                print(f"  saved {path}")
            elif k in (27, ord("q")):
                break

    cam.release()
    cv2.destroyAllWindows()
    print(f"\n{n} images in {imgs}")
    print("Next: CPU-only mesh →  python3 visual_hull.py --session " + args.session)
    print("      or photogrammetry →  ./reconstruct.sh " + args.session)


if __name__ == "__main__":
    main()

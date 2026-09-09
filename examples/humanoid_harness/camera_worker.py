"""Private local pipe bridge: isolate the existing MediaPipe environment.

Only stdout carries in-memory JSON/JPEG packets to our parent. No camera images
are saved. SIGTERM and stdin closure release the capture in the worker finally.
"""

import argparse
import base64
import json
import signal
import sys
import threading

from camera import CameraInput


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--model", default=None)
    args = parser.parse_args()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    def watch_parent():
        sys.stdin.buffer.read()
        stop.set()

    threading.Thread(target=watch_parent, daemon=True).start()
    camera = CameraInput(device=args.device, mode="auto", fps=10, model_path=args.model)
    try:
        try:
            camera.start()
        except (ImportError, OSError, RuntimeError, ValueError):
            packet = camera.snapshot()
            packet["message"] += ". If another app uses this camera, close its camera session and retry."
            print(json.dumps(packet, allow_nan=False), flush=True)
            return
        while not stop.wait(0.1):
            packet = camera.snapshot()
            jpeg = camera.jpeg()
            packet["jpeg_b64"] = base64.b64encode(jpeg).decode("ascii") if jpeg else None
            try:
                print(json.dumps(packet, allow_nan=False), flush=True)
            except BrokenPipeError:
                break
    finally:
        camera.stop()


if __name__ == "__main__":
    main()

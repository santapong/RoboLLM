"""Exclusive capture ownership shared by the local camera pages."""

from threading import RLock


class CameraOwnership:
    def __init__(self):
        self.lock = RLock()
        self.owner = None

    def claim(self, owner):
        with self.lock:
            if self.owner is not None and self.owner is not owner:
                raise ValueError(
                    "Stop the other camera page before starting this camera"
                )
            self.owner = owner

    def release(self, owner):
        with self.lock:
            if self.owner is owner:
                self.owner = None

"""Simulated overhead camera + REAL OpenCV detection pipeline.

This module answers: "how does a camera solve an object's position?"

THE PINHOLE CAMERA MODEL
------------------------
A camera maps a 3D point p_cam = (X, Y, Z) in its own frame to a pixel:

    u = fx * X/Z + cx          fx, fy : focal length in pixels
    v = fy * Y/Z + cy          cx, cy : optical center (image middle)

That 3x3 parameter set is the INTRINSIC matrix K (found once by checkerboard
calibration, e.g. cv2.calibrateCamera). Where the camera sits in the robot's
world is the EXTRINSIC transform (R, t) — found by hand-eye calibration.

SOLVING POSITION (back-projection)
----------------------------------
One pixel alone gives only a RAY, not a point — depth is lost in projection.
To get a 3D point you need one extra constraint. Here (like most bin-picking
setups) it's the table plane: we know the cube's top face lies at height
z = CUBE_SIZE. So:

    1. detect the cube's centroid pixel (u, v)          <- OpenCV, from color
    2. ray direction in camera frame:  d = [(u-cx)/fx, (v-cy)/fy, 1]
    3. rotate the ray into the robot frame: d_w = R @ d
    4. intersect with the plane z = z_top:
           s = (z_top - cam_z) / d_w.z
           p = cam_pos + s * d_w        <- the solved (x, y, z)!

(Alternatives when there is no known plane: stereo cameras, depth cameras
like RealSense, or a fiducial of known size — AprilTag — where scale gives
depth. The math downstream is identical.)

ORIENTATION
-----------
cv2.minAreaRect fits a rotated rectangle to the cube's contour; its angle
gives the cube's yaw. A square looks identical every 90°, so yaw is only
defined modulo 90° — which is fine, because a parallel gripper also grasps
a cube identically every 90°.

The rendering side of this module draws the scene from the camera's point of
view using the SAME projection math — so the detector is solving a genuinely
unknown problem from pixels, exactly as it would with a real camera.
"""
import math

import cv2
import numpy as np


class TableCamera:
    # --- intrinsics (what checkerboard calibration would give you) ---
    W, H = 640, 480
    FX = FY = 600.0
    CX, CY = 320.0, 240.0

    # --- extrinsics (what hand-eye calibration would give you) ---
    # Mounted 0.9 m above the pick zone, looking straight down.
    POS = np.array([0.46, 0.18, 0.90])
    # Camera axes in robot frame: x_cam = +x_base, y_cam = -y_base, z_cam = down
    R = np.array([[1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0],
                  [0.0, 0.0, -1.0]])

    # BGR draw colors per cube color (also used to build HSV detection ranges)
    DRAW_BGR = {'red': (30, 30, 220), 'green': (40, 190, 40), 'blue': (230, 120, 30)}

    def project(self, p_world):
        """3D robot-frame point -> pixel (the forward pinhole equation)."""
        pc = self.R.T @ (np.asarray(p_world, dtype=float) - self.POS)
        if pc[2] <= 1e-6:
            return None
        u = self.FX * pc[0] / pc[2] + self.CX
        v = self.FY * pc[1] / pc[2] + self.CY
        return (int(round(u)), int(round(v)))

    def pixel_to_plane(self, u, v, z_plane):
        """Pixel -> 3D point on a known horizontal plane (back-projection)."""
        d_cam = np.array([(u - self.CX) / self.FX, (v - self.CY) / self.FY, 1.0])
        d_world = self.R @ d_cam
        s = (z_plane - self.POS[2]) / d_world[2]
        return self.POS + s * d_world

    # ------------------------------------------------------------------
    def render(self, objects, cube_size, noise_sigma=6.0):
        """Draw the scene as the camera would see it (cubes' top faces)."""
        img = np.full((self.H, self.W, 3), 120, np.uint8)      # gray table
        for o in objects:
            if o['status'] != 'pending':
                continue
            x, y, yaw = o['true']
            half = cube_size / 2
            corners = []
            for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
                dx = sx * half * math.cos(yaw) - sy * half * math.sin(yaw)
                dy = sx * half * math.sin(yaw) + sy * half * math.cos(yaw)
                px = self.project([x + dx, y + dy, cube_size])  # TOP face
                if px is None:
                    break
                corners.append(px)
            if len(corners) == 4:
                cv2.fillPoly(img, [np.array(corners)], self.DRAW_BGR[o['color']])
        if noise_sigma > 0:  # sensor noise, like a real camera
            noise = np.random.normal(0, noise_sigma, img.shape)
            img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return img

    # ------------------------------------------------------------------
    def _hsv_ranges(self, color):
        """HSV threshold range(s) for one draw color (red wraps the hue axis)."""
        bgr = np.uint8([[self.DRAW_BGR[color]]])
        h = int(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0, 0, 0])
        lo, hi = h - 10, h + 10
        ranges = []
        if lo < 0:
            ranges.append(((180 + lo, 70, 60), (179, 255, 255)))
            lo = 0
        if hi > 179:
            ranges.append(((0, 70, 60), (hi - 180, 255, 255)))
            hi = 179
        ranges.append(((lo, 70, 60), (hi, 255, 255)))
        return ranges

    def detect(self, img, cube_size):
        """Full OpenCV pipeline: pixels -> {color: (x, y, yaw, (u, v), area)}.

        Steps per color: HSV threshold -> morphological cleanup -> contours
        -> minAreaRect -> centroid pixel -> back-project to the cube-top
        plane -> robot-frame position + yaw.
        """
        hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        found = {}
        for color in self.DRAW_BGR:
            mask = np.zeros(img.shape[:2], np.uint8)
            for lo, hi in self._hsv_ranges(color):
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < 150:            # too small to be a cube — noise
                continue
            (u, v), (w, h), angle = cv2.minAreaRect(c)

            # pixel -> 3D on the cube-top plane
            p = self.pixel_to_plane(u, v, cube_size)

            # image angle -> robot yaw: y is flipped between image and robot
            # frame, so yaw = -angle; then fold into [-45, 45) since a square
            # is 90-degree symmetric.
            yaw = math.radians(-angle)
            yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
            found[color] = {'pos': (float(p[0]), float(p[1])), 'yaw': yaw,
                            'pixel': (float(u), float(v)), 'area': float(area),
                            'contour': c}
        return found


if __name__ == '__main__':
    # Self-test: random cubes -> render -> detect -> compare with truth.
    import random
    import sys
    sys.path.insert(0, '.')
    from sort_demo import spawn_cubes, CUBE_SIZE

    random.seed()
    cam = TableCamera()
    objs = spawn_cubes()
    img = cam.render(objs, CUBE_SIZE)
    det = cam.detect(img, CUBE_SIZE)
    ok = True
    for o in objs:
        x, y, yaw = o['true']
        d = det.get(o['color'])
        if d is None:
            print(f'{o["color"]:6s} NOT DETECTED')
            ok = False
            continue
        ex = (d['pos'][0] - x) * 1000
        ey = (d['pos'][1] - y) * 1000
        # compare yaw modulo 90 deg
        dyaw = math.degrees((d['yaw'] - yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4)
        print(f'{o["color"]:6s} truth=({x:.3f},{y:.3f},{math.degrees(yaw):+6.1f}°)  '
              f'solved=({d["pos"][0]:.3f},{d["pos"][1]:.3f},{math.degrees(d["yaw"]):+6.1f}°)  '
              f'err: {ex:+5.1f} mm, {ey:+5.1f} mm, {dyaw:+5.2f}°')
        if abs(ex) > 5 or abs(ey) > 5 or abs(dyaw) > 4:
            ok = False
    print('SELF-TEST', 'PASSED' if ok else 'FAILED')
    sys.exit(0 if ok else 1)

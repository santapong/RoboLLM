"""Metric tabletop perception. Monocular depth is never treated as measured 3D."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .printables import DICT, MAT_MARKER_SIZE, MAT_MARKERS, board


class CalibrationError(ValueError):
    pass


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (
        t.shape != (4, 4)
        or not np.isfinite(t).all()
        or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
    ):
        raise CalibrationError("invalid rigid transform")
    if not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(
        np.linalg.det(t[:3, :3]), 1, atol=1e-5
    ):
        raise CalibrationError(
            "transform rotation must be orthonormal and right handed"
        )
    return t


def calibration_id(data):
    payload = {k: v for k, v in data.items() if k != "calibration_id"}
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()[:24]


def save_calibration(data, path):
    data = dict(data)
    data["calibration_id"] = calibration_id(data)
    Path(path).write_text(json.dumps(data, indent=2) + "\n")
    return data


def calibrate_images(paths, output):
    detector = cv2.aruco.CharucoDetector(board())
    corners, ids, accepted = [], [], []
    size = None
    for path in paths:
        frame = cv2.imread(str(path))
        if frame is None:
            raise CalibrationError(f"unreadable calibration image: {Path(path).name}")
        shape = (frame.shape[1], frame.shape[0])
        if size is not None and shape != size:
            raise CalibrationError("calibration images have different resolutions")
        size = shape
        found, labels, _, _ = detector.detectBoard(frame)
        if labels is not None and len(labels) >= 10:
            corners.append(found)
            ids.append(labels)
            accepted.append(Path(path).name)
    if len(corners) < 12:
        raise CalibrationError(
            "need at least 12 usable views, with varied board position and tilt"
        )
    rms, K, dist, rvecs, _tvecs = cv2.aruco.calibrateCameraCharuco(
        corners, ids, board(), size, None, None
    )
    normals = np.array([cv2.Rodrigues(r)[0][:, 2] for r in rvecs])
    if np.max(np.linalg.norm(normals - normals.mean(axis=0), axis=1)) < 0.15:
        raise CalibrationError(
            "views lack board tilt diversity; take new calibration views"
        )
    if not math.isfinite(rms) or rms > 1.0:
        raise CalibrationError(f"reprojection RMS {rms:.3f} px exceeds 1.0 px")
    return save_calibration(
        {
            "schema_version": 1,
            "width": size[0],
            "height": size[1],
            "K": K.tolist(),
            "distortion": dist.ravel().tolist(),
            "rms_px": rms,
            "views": accepted,
            "board": {
                "squares": [7, 5],
                "square_m": 0.03,
                "marker_m": 0.022,
                "dictionary": "DICT_5X5_100",
            },
            "T_base_mat": None,
            "T_toolmarker_tcp": None,
            "base_registration_evidence": None,
            "held_out_rmse_m": None,
        },
        output,
    )


@dataclass
class MatPose:
    T_camera_mat: np.ndarray
    reprojection_rms_px: float
    marker_ids: list[int]


class TableCamera:
    def __init__(self, calibration):
        data = (
            json.loads(Path(calibration).read_text())
            if isinstance(calibration, (str, Path))
            else calibration
        )
        self.calibration = dict(data)
        self.K = np.asarray(data["K"], float)
        self.dist = np.asarray(data["distortion"], float)
        if (
            self.K.shape != (3, 3)
            or not np.isfinite(self.K).all()
            or not np.isfinite(self.dist).all()
            or self.K[0, 0] <= 0
            or self.K[1, 1] <= 0
        ):
            raise CalibrationError("invalid intrinsics")
        self.size = (int(data["width"]), int(data["height"]))
        if min(self.size) <= 0:
            raise CalibrationError("invalid camera dimensions")
        self.id = calibration_id(data)
        if data.get("calibration_id", self.id) != self.id:
            raise CalibrationError(
                "calibration identity mismatch; regenerate after editing"
            )
        self.base = (
            rigid(data["T_base_mat"]) if data.get("T_base_mat") is not None else None
        )
        self.tool_offset = (
            rigid(data["T_toolmarker_tcp"])
            if data.get("T_toolmarker_tcp") is not None
            else None
        )
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(DICT), cv2.aruco.DetectorParameters()
        )

    def require_hardware(self):
        rmse = self.calibration.get("held_out_rmse_m")
        if (
            self.base is None
            or self.tool_offset is None
            or not self.calibration.get("base_registration_evidence")
        ):
            raise CalibrationError(
                "base registration and tool-marker calibration are required"
            )
        if (
            not isinstance(rmse, (float, int))
            or isinstance(rmse, bool)
            or not math.isfinite(rmse)
            or not 0 <= rmse <= 0.005
        ):
            raise CalibrationError(
                "held-out tabletop RMSE must be measured and <= 5 mm"
            )

    def markers(self, frame):
        if frame.shape[1::-1] != self.size:
            raise CalibrationError("camera resolution changed; recalibrate")
        corners, ids, _ = self.detector.detectMarkers(frame)
        return (
            {}
            if ids is None
            else {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}
        )

    def mat_pose(self, markers):
        obj, img, used = [], [], []
        for identifier, (x, y) in MAT_MARKERS.items():
            if identifier not in markers:
                continue
            s = MAT_MARKER_SIZE
            # Top-left, top-right, bottom-right, bottom-left, with mat +Y up.
            obj.extend([[x, y + s, 0], [x + s, y + s, 0], [x + s, y, 0], [x, y, 0]])
            img.extend(markers[identifier])
            used.append(identifier)
        if len(used) < 2:
            raise CalibrationError("at least two workspace markers must be visible")
        obj = np.asarray(obj, float)
        img = np.asarray(img, float)
        ok, r, t = cv2.solvePnP(
            obj, img, self.K, self.dist, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            raise CalibrationError("workspace pose solve failed")
        R = cv2.Rodrigues(r)[0]
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.ravel()
        camera_in_mat = -R.T @ t.ravel()
        if camera_in_mat[2] <= 0 or np.any((R @ obj.T + t).T[:, 2] <= 0):
            raise CalibrationError("ambiguous/reversed tabletop pose")
        predicted = cv2.projectPoints(obj, r, t, self.K, self.dist)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((predicted - img) ** 2, axis=1))))
        if rms > 1:
            raise CalibrationError(f"workspace reprojection error {rms:.2f} px")
        return MatPose(T, rms, used)

    def pixel_to_mat(self, pixel, pose, height_m=0):
        pixel = np.asarray(pixel, float)
        if (
            pixel.shape != (2,)
            or not np.isfinite(pixel).all()
            or not np.isfinite(height_m)
        ):
            raise CalibrationError("invalid pixel/plane")
        undist = cv2.undistortPoints(pixel.reshape(1, 1, 2), self.K, self.dist).ravel()
        inv = np.linalg.inv(pose.T_camera_mat)
        origin = inv[:3, 3]
        direction = inv[:3, :3] @ np.r_[undist, 1]
        if abs(direction[2]) < 1e-8:
            raise CalibrationError("ray parallel to tabletop")
        distance = (height_m - origin[2]) / direction[2]
        if distance <= 0:
            raise CalibrationError("intersection behind the camera")
        return origin + distance * direction

    def tag_pose(self, corners, size_m):
        h = size_m / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
        ok, r, t = cv2.solvePnP(
            obj,
            np.asarray(corners, float),
            self.K,
            self.dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok or t[2, 0] <= 0:
            raise CalibrationError("tag pose solve failed")
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(r)[0]
        T[:3, 3] = t.ravel()
        return T

    def red_objects(self, frame, pose, height_m=0.01):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 65), (10, 255, 255)) | cv2.inRange(
            hsv, (170, 100, 65), (179, 255, 255)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 60:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            pixel = [moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]]
            xyz = self.pixel_to_mat(pixel, pose, height_m)
            if not 0 <= xyz[0] <= 0.18 or not 0 <= xyz[1] <= 0.12:
                continue
            result.append(
                {
                    "label": "red box",
                    "position_mat_m": xyz.tolist(),
                    "pixel": pixel,
                    "area_px": area,
                    "pose_source": "rgb_known_height_approximation",
                }
            )
        result.sort(key=lambda o: (o["position_mat_m"][0], o["position_mat_m"][1]))
        for i, obj in enumerate(result):
            obj["id"] = f"red_box_{i}"
        return result

    def observe(self, frame, *, captured_at=None):
        captured_at = time.monotonic() if captured_at is None else captured_at
        markers = self.markers(frame)
        pose = self.mat_pose(markers)
        objects = self.red_objects(frame, pose)
        inv = np.linalg.inv(pose.T_camera_mat)
        if 70 in markers:
            tag = inv @ self.tag_pose(markers[70], 0.015)
            center = tag[:3, 3] - tag[:3, 2] * 0.010
            objects = [
                {
                    "id": "red_box_0",
                    "label": "red box",
                    "position_mat_m": center.tolist(),
                    "pose_source": "fiducial_known_box_height",
                }
            ]
        tcp = None
        if 60 in markers and self.tool_offset is not None:
            tcp = (inv @ self.tag_pose(markers[60], 0.020) @ self.tool_offset)[
                :3, 3
            ].tolist()
        return {
            "captured_at_monotonic": captured_at,
            "calibration_id": self.id,
            "mat_pose": pose,
            "objects": objects,
            "tcp_mat_m": tcp,
            "reference_visible": True,
        }


class HandTracker:
    def __init__(self, model_path):
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision

        self.mp = mp
        self.landmarker = vision.HandLandmarker.create_from_options(
            vision.HandLandmarkerOptions(
                base_options=python.BaseOptions(model_asset_path=str(model_path)),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=2,
                min_hand_detection_confidence=0.65,
                min_hand_presence_confidence=0.65,
            )
        )
        self.last_timestamp = -1

    def detect(self, frame, timestamp_ms):
        timestamp_ms = max(int(timestamp_ms), self.last_timestamp + 1)
        self.last_timestamp = timestamp_ms
        image = self.mp.Image(
            image_format=self.mp.ImageFormat.SRGB,
            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
        )
        result = self.landmarker.detect_for_video(image, timestamp_ms)
        h, w = frame.shape[:2]
        hands = []
        for index, hand in enumerate(result.hand_landmarks):
            categories = (
                result.handedness[index] if index < len(result.handedness) else []
            )
            category = categories[0] if categories else None
            hands.append(
                {
                    "tip_px": [hand[8].x * w, hand[8].y * h],
                    "landmarks_px": [[p.x * w, p.y * h] for p in hand],
                    "landmarks_norm": [[float(p.x), float(p.y)] for p in hand],
                    # This wrapper does not flip calibrated camera pixels. Callers
                    # that use handedness must supply the intended image convention.
                    "handedness": category.category_name if category else None,
                    "confidence": float(category.score) if category else None,
                }
            )
        return hands

    def close(self):
        self.landmarker.close()

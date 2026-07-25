"""Webcam body tracking: MediaPipe PoseLandmarker -> a torso-relative body frame.

The top half of this module is PURE MATH — no cv2, no mediapipe, no ROS, no
numpy — so `tools/body_accept.py` can unit-test the geometry with synthetic
landmarks and no camera. The camera classes at the bottom import cv2/mediapipe
LAZILY inside __init__, so importing this module never drags vision libraries
into a synthetic run.

=============================================================================
MEASURED FACTS (probed on this box against a live camera — not from docs)
=============================================================================

1. AXIS CONVENTION of pose_world_landmarks (metres, hip-midpoint origin).
   Measured with a person facing the camera:
       LEFT_SHOULDER - RIGHT_SHOULDER  ->  x +0.304   => +x is the subject's LEFT
       SHOULDERmid   - HIPmid          ->  y -0.485   => +y is DOWN
       NOSE          - EARmid          ->  z -0.112   => -z is FORWARD (toward camera)
   So the map into a REP-103 body frame (x forward, y left, z up) is:
       body_x = -world_z      body_y = +world_x      body_z = -world_y
   That is what to_body() below implements.

2. cv2.flip SWAPS MediaPipe POSE left/right labels. Measured directly:
       |flip.LEFT_WRIST.x - (1 - raw.RIGHT_WRIST.x)| = 0.0177   <- content-following
       |flip.LEFT_WRIST.x - (1 - raw.LEFT_WRIST.x)|  = 0.6697   <- side-following
   A 38x separation: the labels follow ANATOMY, inferred from body cues, so
   mirroring the image moves the LEFT label onto the other physical arm.
   => POSE RUNS ON THE RAW FRAME. Only the preview is flipped.
   This is the OPPOSITE of the hand API, where handedness explicitly assumes a
   mirrored image — which is why hand_follow's flip is correct and mandatory.
   Two models, two conventions, one frame. body_accept.py --live regression-
   tests this so a future refactor cannot quietly reintroduce it.

3. HIPS ARE USUALLY INVISIBLE. Seated at a desk, measured visibility:
       shoulders 1.00   nose 1.00   ears 1.00   elbows 1.00
       HIPS 0.01        wrists 0.26
   The design assumed a shoulder-midpoint-to-hip-midpoint torso "up" vector.
   In the actual use case that vector is unavailable, so the camera-up
   fallback is the PRIMARY path, not the edge case. torso_frame() reports
   which one it used; do not treat "camera_up" as a degraded warning state.

4. MediaPipe HALLUCINATES occluded limbs with plausible coordinates. The
   wrists above were reported at ordinary-looking positions with visibility
   0.01-0.26 while out of frame. Gate on visibility or the robot will chase
   a hand that isn't there.
"""

import math

# Landmark indices we use (mediapipe PoseLandmark enum values).
NOSE = 0
LEFT_EAR, RIGHT_EAR = 7, 8
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_ELBOW, RIGHT_ELBOW = 13, 14
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24

# Landmarks that must be visible for tracking to count as live at all.
# Wrists are deliberately NOT here: an arm can hang out of frame while the
# rest of the body tracks fine, and M4 gates each arm independently.
CORE_LANDMARKS = (LEFT_SHOULDER, RIGHT_SHOULDER, NOSE)

DEFAULT_MIN_VISIBILITY = 0.5


# --------------------------------------------------------------- pure vectors
def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return (0.0, 0.0, 0.0) if n < 1e-9 else (a[0] / n, a[1] / n, a[2] / n)


def mid(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def to_body(world_xyz):
    """MediaPipe world landmark -> REP-103 body axes (x forward, y left, z up).

    Derived from measurement, not documentation — see fact 1 in the module
    docstring.  body_x = -world_z,  body_y = +world_x,  body_z = -world_y.
    """
    wx, wy, wz = world_xyz
    return (-wz, wx, -wy)


def rot_transpose_apply(basis, v):
    """Express world-ish vector v in the frame whose COLUMNS are `basis`.

    basis is (f, l, u) as three column vectors, i.e. the rotation body<-frame.
    Applying its transpose maps a vector INTO the torso frame.
    """
    f, l, u = basis
    return (dot(f, v), dot(l, v), dot(u, v))


# ------------------------------------------------------------------ the frame
def torso_frame(points, visibility=None, min_visibility=DEFAULT_MIN_VISIBILITY):
    """Build an orthonormal torso frame from body-frame landmarks.

    `points` maps landmark index -> (x, y, z) already in BODY axes (run each
    world landmark through to_body first).

    Returns ((f, l, u), source) where source is "hips" or "camera_up", or
    (None, reason) if the shoulders are unusable.

    Two paths, and the fallback is the common one in practice (fact 3):
      * "hips"      — up = shoulder-mid to hip-mid. Captures torso lean.
                      Only available when both hips clear min_visibility.
      * "camera_up" — up = +z (camera vertical). Cannot see torso lean, so a
                      leaning user reads as an upright one. Acceptable: the
                      arm angles we derive are measured RELATIVE to this frame,
                      and a seated user's torso is near-vertical anyway.
    """
    vis = visibility or {}

    def seen(i):
        return vis.get(i, 1.0) >= min_visibility

    if not (seen(LEFT_SHOULDER) and seen(RIGHT_SHOULDER)):
        return None, "shoulders_not_visible"

    l_raw = sub(points[LEFT_SHOULDER], points[RIGHT_SHOULDER])
    if norm(l_raw) < 1e-6:
        return None, "shoulders_coincident"

    if (
        LEFT_HIP in points
        and RIGHT_HIP in points
        and seen(LEFT_HIP)
        and seen(RIGHT_HIP)
    ):
        u_raw = sub(
            mid(points[LEFT_SHOULDER], points[RIGHT_SHOULDER]),
            mid(points[LEFT_HIP], points[RIGHT_HIP]),
        )
        source = "hips"
        if norm(u_raw) < 1e-6:
            u_raw, source = (0.0, 0.0, 1.0), "camera_up"
    else:
        u_raw, source = (0.0, 0.0, 1.0), "camera_up"

    u = unit(u_raw)
    l = unit(sub(l_raw, scale(u, dot(l_raw, u))))  # re-orthogonalise
    if norm(l) < 1e-6:
        return None, "shoulder_line_parallel_to_up"
    # REP-103 right-handed: x_hat = y_hat cross z_hat, i.e. forward = left x up.
    f = unit(cross(l, u))
    return (f, l, u), source


def basis_to_quaternion(basis):
    """(f, l, u) column vectors -> (x, y, z, w) quaternion.

    Publishing the torso frame as a real TF frame is the single most useful
    debugging aid for retargeting: if the arm angles come out wrong in M4, you
    look at whether human/torso's axes point where the person's body does
    before you suspect the trigonometry.

    Shepperd's method — pick the branch off the largest diagonal term so the
    square root never operates on something near zero.
    """
    f, l, u = basis
    # Rotation matrix with f, l, u as COLUMNS: m[row][col].
    m = (
        (f[0], l[0], u[0]),
        (f[1], l[1], u[1]),
        (f[2], l[2], u[2]),
    )
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return (x / n, y / n, z / n, w / n)


class BodyObservation:
    """One frame's worth of tracked body state, in the torso frame."""

    __slots__ = ("t", "points_body", "visibility", "basis", "frame_source", "tracked", "reason")

    def __init__(self, t, points_body, visibility, basis, frame_source, tracked, reason=""):
        self.t = t
        self.points_body = points_body
        self.visibility = visibility
        self.basis = basis
        self.frame_source = frame_source
        self.tracked = tracked
        self.reason = reason

    def in_torso(self, index):
        """Landmark position expressed in the torso frame, shoulder-agnostic."""
        return rot_transpose_apply(self.basis, self.points_body[index])

    def limb_dirs(self, side):
        """(upper_arm, forearm) unit vectors in the torso frame for 'left'/'right'.

        Anchored at the SHOULDER, not the hip: that removes both the
        hip-midpoint origin and any torso translation in one step, which
        matters because the hips are usually the least reliable landmarks.
        """
        sh, el, wr = (
            (LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST)
            if side == "left"
            else (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST)
        )
        p = self.points_body
        a = unit(rot_transpose_apply(self.basis, sub(p[el], p[sh])))
        b = unit(rot_transpose_apply(self.basis, sub(p[wr], p[el])))
        return a, b

    def head_dir(self):
        """Head forward direction in the torso frame, from NOSE and both EARs."""
        p = self.points_body
        ear_mid = mid(p[LEFT_EAR], p[RIGHT_EAR])
        return unit(rot_transpose_apply(self.basis, sub(p[NOSE], ear_mid)))


# =========================================================================
# Camera side. cv2 / mediapipe are imported lazily so a synthetic run never
# touches them.
# =========================================================================
class ThreadedCapture:
    """Camera reader on its own thread (latest frame only) so the ~40 ms v4l2
    read overlaps MediaPipe inference. CAP_PROP_BUFFERSIZE=1 stops the driver
    queue serving stale frames. Same pattern as hand_follow's."""

    def __init__(self, device):
        import threading

        import cv2

        self._cv2 = cv2
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"camera {device} failed to open — is the container started by "
                f"the ros2-arm launcher (it passes --device /dev/video0)?"
            )
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = self.cap.read()
        if not ok:
            self.cap.release()
            raise RuntimeError(
                f"camera {device} opened but read() failed — device busy, or "
                f"wrong node (try /dev/video1)?"
            )
        self._frame = frame
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if ok:
                with self._lock:
                    self._frame = frame

    def read(self):
        with self._lock:
            return self._frame

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


class BodyTracker:
    """PoseLandmarker on the RAW frame -> BodyObservation.

    Never flips the frame for inference (fact 2). `annotate()` is the only
    place a flip happens, and it flips the drawn preview image only.
    """

    name = "camera"

    def __init__(
        self,
        device="/dev/video0",
        model_path="/opt/models/pose_landmarker_full.task",
        min_visibility=DEFAULT_MIN_VISIBILITY,
        preview=False,
    ):
        import cv2
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mpv

        self._cv2, self._mp, self._mpv = cv2, mp, mpv
        self.min_visibility = min_visibility
        self.preview = preview
        self._frame_index = 0

        self.capture = ThreadedCapture(device)
        self.detector = mpv.PoseLandmarker.create_from_options(
            mpv.PoseLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=model_path),
                running_mode=mpv.RunningMode.VIDEO,
                num_poses=1,
                min_pose_detection_confidence=0.5,
                min_pose_presence_confidence=0.5,
                min_tracking_confidence=0.5,
                output_segmentation_masks=False,
            )
        )
        self.last_frame = None
        self.last_result = None

    def start(self):
        return None

    def stop(self):
        # Close the detector explicitly. mediapipe 0.10.35's PoseLandmarker
        # __del__ raises "TypeError: 'NoneType' object is not callable" when it
        # runs during interpreter teardown; closing first avoids the noise.
        try:
            self.detector.close()
        except Exception:  # noqa: BLE001
            pass
        self.capture.stop()

    def read(self, t):
        """One inference pass. Returns a BodyObservation (tracked False when
        the body is not usable) or None if no frame is available yet."""
        frame = self.capture.read()
        if frame is None:
            return None
        self.last_frame = frame

        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        self._frame_index += 1
        result = self.detector.detect_for_video(image, int(t * 1000.0))
        self.last_result = result

        if not result.pose_world_landmarks:
            return BodyObservation(t, {}, {}, None, "", False, "no_pose")

        world = result.pose_world_landmarks[0]
        points = {i: to_body((lm.x, lm.y, lm.z)) for i, lm in enumerate(world)}
        vis = {i: lm.visibility for i, lm in enumerate(world)}

        missing = [i for i in CORE_LANDMARKS if vis.get(i, 0.0) < self.min_visibility]
        if missing:
            return BodyObservation(t, points, vis, None, "", False, f"low_vis:{missing}")

        basis, source = torso_frame(points, vis, self.min_visibility)
        if basis is None:
            return BodyObservation(t, points, vis, None, "", False, source)
        return BodyObservation(t, points, vis, basis, source, True)

    def annotate(self, observation):
        """Mirrored preview image for a human to look at. THE ONLY FLIP.

        Drawn from NORMALIZED landmarks (image space), then flipped as a whole
        picture — so the geometry stays consistent and no inference ever sees
        a mirrored frame.
        """
        cv2, mpv = self._cv2, self._mpv
        frame = self.last_frame
        if frame is None:
            return None
        canvas = frame.copy()
        res = self.last_result
        if res is not None and res.pose_landmarks:
            h, w = canvas.shape[:2]
            lms = res.pose_landmarks[0]
            for conn in mpv.PoseLandmarksConnections.POSE_LANDMARKS:
                a, b = lms[conn.start], lms[conn.end]
                if min(a.visibility, b.visibility) < self.min_visibility:
                    continue
                cv2.line(canvas, (int(a.x * w), int(a.y * h)),
                         (int(b.x * w), int(b.y * h)), (0, 220, 0), 2)
            for i, lm in enumerate(lms):
                ok = lm.visibility >= self.min_visibility
                cv2.circle(canvas, (int(lm.x * w), int(lm.y * h)), 3,
                           (0, 220, 0) if ok else (0, 160, 255), -1)
        tracked = observation is not None and observation.tracked
        label = "TRACKED" if tracked else "no body"
        if tracked:
            label += f"  frame={observation.frame_source}"
        elif observation is not None:
            label += f"  ({observation.reason})"
        canvas = cv2.flip(canvas, 1)  # <-- preview only, never inference
        cv2.putText(canvas, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 220, 0) if tracked else (0, 160, 255), 2)
        return canvas

"""track_node -- TALOS M3: webcam FULL-BODY tracking, robot PARKED.

Two modes, selected by the `synthetic` parameter:
  synthetic:=true (default)  SyntheticBodyPoseSource drives a deterministic
                              whole-body motion generator. No camera, no
                              mediapipe import anywhere in the process.
  synthetic:=false           BodyTracker runs MediaPipe PoseLandmarker
                              against a real camera. mediapipe/cv2 are
                              imported LAZILY, only on this path.

Neither mode drives a robot joint: this node's only job is publishing what it
sees. TF `human/*` frames, a JSON body-observation topic, and RViz markers --
same three outputs regardless of source, so a consumer never needs to know
which one is live.

RUNNING UNDER THE RIGHT PYTHON MATTERS: mediapipe lives in /opt/mpvenv, not
the system python ament's console_scripts are shebanged to. track.launch.py
sets prefix=/opt/mpvenv/bin/python (the venv is --system-site-packages, so
rclpy still imports fine and this interpreter is correct for BOTH modes).
Without it, camera mode dies with ModuleNotFoundError while synthetic mode
keeps working -- which reads as a camera fault, not an interpreter one;
_make_source() below catches that and says so explicitly.
"""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from talos_mirror.body_track import basis_to_quaternion, observation_dict

PREVIEW_WINDOW = "talos_mirror - body (mirrored preview)"

# TF frames published per tracked landmark: index -> child frame id. Covers
# every landmark this milestone tracks -- arms+head (upstream's set) AND the
# leg chain this task adds.
TF_LANDMARKS = {
    0: "human/head",
    7: "human/l_ear",
    8: "human/r_ear",
    11: "human/l_shoulder",
    13: "human/l_elbow",
    15: "human/l_wrist",
    12: "human/r_shoulder",
    14: "human/r_elbow",
    16: "human/r_wrist",
    23: "human/l_hip",
    24: "human/r_hip",
    25: "human/l_knee",
    26: "human/r_knee",
    27: "human/l_ankle",
    28: "human/r_ankle",
    29: "human/l_heel",
    30: "human/r_heel",
    31: "human/l_foot",
    32: "human/r_foot",
}

# Marker colour per limb -- purely cosmetic, but makes "one leg dropped"
# visually obvious in RViz, which is the whole point of per-limb gating.
LIMB_COLOR = {
    "arm_l": (0.2, 0.9, 0.3),
    "arm_r": (0.2, 0.6, 0.95),
    "leg_l": (0.95, 0.65, 0.15),
    "leg_r": (0.85, 0.25, 0.85),
    "head": (0.95, 0.9, 0.2),
}


class TrackNode(Node):
    def __init__(self):
        super().__init__("talos_track")

        p = self.declare_parameter
        p("synthetic", True)
        p("rate_hz", 30.0)
        p("sweep_period_s", 16.0)
        p("camera", "/dev/video0")
        p("pose_model_path", "/opt/models/pose_landmarker_full.task")
        p("min_visibility", 0.5)
        p("preview", False)
        p("body_frame_id", "camera_link")

        g = self.get_parameter
        self.synthetic = bool(g("synthetic").value)
        self.rate_hz = float(g("rate_hz").value)
        self.min_visibility = float(g("min_visibility").value)
        self.preview = bool(g("preview").value)
        self.body_frame_id = str(g("body_frame_id").value)

        self.marker_pub = self.create_publisher(MarkerArray, "/body/markers", 10)
        self.tracked_pub = self.create_publisher(Bool, "/body/tracked", 10)
        self.observation_pub = self.create_publisher(String, "/body/observation", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._lock = threading.Lock()
        self._preview_canvas = None
        self._preview_ready = False
        self._vision_thread = None
        self._vision_stop = threading.Event()
        self._t0 = time.monotonic()

        self.source = self._make_source()
        self.source.start()

        if self.synthetic:
            # Cheap pure-math generator: a plain timer is enough, no thread.
            self.create_timer(1.0 / self.rate_hz, self._tick_synthetic)
        else:
            # Camera inference (~30 ms) runs on its own thread so it is never
            # gated behind a fixed-rate timer -- same pattern humanoid_mirror's
            # mirror_node uses for its vision thread.
            self._vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
            self._vision_thread.start()

        if self.preview:
            # 20 Hz is plenty for a human to watch; HighGUI painting happens
            # on THIS timer, i.e. the main/executor thread -- see _draw_preview.
            self.create_timer(0.05, self._draw_preview)

        self.get_logger().info(
            f"talos_track up: source={self.source.name} rate={self.rate_hz:.0f} Hz "
            f"min_visibility={self.min_visibility:.2f} body_frame_id={self.body_frame_id}"
        )

    # ------------------------------------------------------------------ setup
    def _make_source(self):
        if self.synthetic:
            from talos_mirror.pose_source import SyntheticBodyPoseSource

            return SyntheticBodyPoseSource(
                period_s=float(self.get_parameter("sweep_period_s").value)
            )
        try:
            from talos_mirror.body_track import BodyTracker
        except ModuleNotFoundError as exc:
            import sys

            raise RuntimeError(
                f"{exc}. talos_track is running under {sys.executable}. In the "
                f"ros2-talos:jazzy image mediapipe lives in /opt/mpvenv -- launch "
                f"via track.launch.py (it sets prefix=/opt/mpvenv/bin/python), or "
                f"run: /opt/mpvenv/bin/python $(which track_node). Synthetic mode "
                f"needs none of this, which is why it still works."
            ) from exc

        g = self.get_parameter
        return BodyTracker(
            device=g("camera").value,
            model_path=g("pose_model_path").value,
            min_visibility=self.min_visibility,
            preview=self.preview,
        )

    # ---------------------------------------------------------------- ticking
    def _tick_synthetic(self):
        t = time.monotonic() - self._t0
        obs = self.source.read(t)
        self._publish(obs)

    def _vision_loop(self):
        while not self._vision_stop.is_set():
            t = time.monotonic() - self._t0
            try:
                obs = self.source.read(t)
            except Exception as exc:  # noqa: BLE001 -- vision must not kill the node
                self.get_logger().warn(f"tracker raised: {exc}", throttle_duration_sec=5.0)
                time.sleep(0.1)
                continue
            if obs is None:
                time.sleep(0.005)
                continue
            self._publish(obs)
            if self.preview:
                try:
                    canvas = self.source.annotate(obs)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"annotate failed: {exc}", throttle_duration_sec=10.0)
                    canvas = None
                with self._lock:
                    self._preview_canvas = canvas

    def _draw_preview(self):
        """MUST run on the main thread -- OpenCV HighGUI is not thread-safe;
        on the Qt backend imshow() from a worker thread creates the window
        but never paints into it (see body_track.py / ../../TECHNICAL.md for
        the full explanation). rclpy timer callbacks run on the executor
        thread, i.e. here, so this is safe."""
        with self._lock:
            canvas = self._preview_canvas
            self._preview_canvas = None
        if canvas is None:
            return
        try:
            import cv2

            if not self._preview_ready:
                h, w = canvas.shape[:2]
                cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(PREVIEW_WINDOW, w, h)
                self._preview_ready = True
            cv2.imshow(PREVIEW_WINDOW, canvas)
            cv2.waitKey(1)
        except Exception as exc:  # noqa: BLE001 -- no X11 must not kill tracking
            self.get_logger().warn(f"preview unavailable, disabling: {exc}",
                                   throttle_duration_sec=10.0)
            self.preview = False

    # --------------------------------------------------------------- publish
    def _publish(self, obs):
        stamp = self.get_clock().now().to_msg()
        self.tracked_pub.publish(Bool(data=bool(obs.tracked)))
        self.observation_pub.publish(String(data=json.dumps(self._observation_dict(obs))))

        if not obs.tracked or obs.basis is None:
            clear = Marker()
            clear.header.frame_id = self.body_frame_id
            clear.header.stamp = stamp
            clear.action = Marker.DELETEALL
            self.marker_pub.publish(MarkerArray(markers=[clear]))
            return

        observed = obs.observed_landmarks(self.min_visibility)
        transforms = []
        for index, child in TF_LANDMARKS.items():
            if index not in observed or index not in obs.points_body:
                continue  # limb's gate failed -- never publish TF for it
            x, y, z = obs.points_body[index]
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.body_frame_id
            tf.child_frame_id = child
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = float(z)
            tf.transform.rotation.w = 1.0
            transforms.append(tf)

        qx, qy, qz, qw = basis_to_quaternion(obs.basis)
        torso = TransformStamped()
        torso.header.stamp = stamp
        torso.header.frame_id = self.body_frame_id
        torso.child_frame_id = "human/torso"
        sx, sy, sz = obs.points_body[11]
        rx, ry, rz = obs.points_body[12]
        torso.transform.translation.x = float((sx + rx) * 0.5)
        torso.transform.translation.y = float((sy + ry) * 0.5)
        torso.transform.translation.z = float((sz + rz) * 0.5)
        torso.transform.rotation.x = float(qx)
        torso.transform.rotation.y = float(qy)
        torso.transform.rotation.z = float(qz)
        torso.transform.rotation.w = float(qw)
        transforms.append(torso)
        self.tf_broadcaster.sendTransform(transforms)

        self.marker_pub.publish(self._skeleton_markers(obs, observed, stamp))

    def _observation_dict(self, obs):
        """The /body/observation payload. Thin wrapper -- see
        body_track.observation_dict for the field-by-field description
        (M4 added "basis" for mirror_node's benefit; everything else is
        unchanged from M3)."""
        return observation_dict(obs, self.min_visibility)

    def _skeleton_markers(self, obs, observed, stamp):
        """Skeleton as one LINE_LIST per limb (so RViz shows a dropped limb
        as a colour vanishing, not just points thinning out) plus one
        SPHERE_LIST for all observed joints."""
        # Connections within each limb, in traversal order. Gating is
        # all-or-nothing per limb (see body_track.observed_landmarks), so
        # either every index below is in `observed` or none are.
        chains = {
            "arm_l": (11, 13, 15),
            "arm_r": (12, 14, 16),
            "leg_l": (23, 25, 27, 29, 31),
            "leg_r": (24, 26, 28, 30, 32),
            "head": (7, 0, 8),  # ear - nose - ear, just to draw something
        }

        markers = []
        marker_id = 0
        for limb, chain in chains.items():
            if not all(i in observed for i in chain):
                continue
            line = Marker()
            line.header.frame_id = self.body_frame_id
            line.header.stamp = stamp
            line.ns = f"body/{limb}"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.02
            r, g, b = LIMB_COLOR[limb]
            line.color.r, line.color.g, line.color.b, line.color.a = r, g, b, 1.0
            line.pose.orientation.w = 1.0
            for idx in chain:
                if idx not in obs.points_body:
                    continue
                x, y, z = obs.points_body[idx]
                line.points.append(Point(x=float(x), y=float(y), z=float(z)))
            if line.points:
                markers.append(line)

        dots = Marker()
        dots.header.frame_id = self.body_frame_id
        dots.header.stamp = stamp
        dots.ns = "body/joints"
        dots.id = 0
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.scale.x = dots.scale.y = dots.scale.z = 0.035
        dots.color.r = 1.0
        dots.color.g = 0.8
        dots.color.a = 1.0
        dots.pose.orientation.w = 1.0
        for idx in sorted(observed):
            if idx not in obs.points_body:
                continue
            x, y, z = obs.points_body[idx]
            dots.points.append(Point(x=float(x), y=float(y), z=float(z)))
        markers.append(dots)

        return MarkerArray(markers=markers)

    # -------------------------------------------------------------- shutdown
    def shutdown(self):
        self._vision_stop.set()
        if self._vision_thread is not None:
            self._vision_thread.join(timeout=2.0)
        try:
            self.source.stop()
            if self.preview:
                import cv2

                cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001 -- best effort during teardown
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

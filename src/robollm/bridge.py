#!/usr/bin/env python3
"""
robollm.bridge — one shared ROS 2 node used by BOTH the MCP server and the web UI.

Why a shared module? So there is a single, readable place that talks to the robot.
- apps/mcp/server.py imports it       -> an LLM can drive via MCP tools.
- apps/dashboard/server.py imports it -> you drive from a browser dashboard.

The node runs in a background thread so the host program (MCP stdio loop or the
web server's event loop) stays responsive.

Teleop note: the web UI needs *continuous* motion while a key is held. Publishing
a fixed-duration Twist doesn't fit that. So the bridge keeps a "target velocity"
and a 20 Hz timer republishes it, with a DEADMAN watchdog: if no fresh command
arrives within `deadman_s`, the robot is automatically stopped. That means a
dropped websocket or a released key can never leave the robot driving away.

Requires ROS 2 (Jazzy) sourced before import — see scripts/launch/.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener


class Bridge(Node):
    """One long-lived ROS 2 node. Publishers/subscribers + small helper methods."""

    def __init__(self, node_name: str = "claude_ros2_bridge", deadman_s: float = 0.6,
                 time_source: Callable[[], float] | None = None) -> None:
        super().__init__(node_name)
        # Injectable monotonic clock for the deadman/teleop watchdog. Defaults to
        # time.monotonic (unchanged behaviour); tests pass a fake clock so the
        # deadman timeout can be exercised without real sleeping.
        self._now: Callable[[], float] = time_source if time_source is not None else time.monotonic
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        # Nav2 messages are an optional dependency: navigate_to needs Nav2 running
        # anyway, and nav2_msgs is absent from some minimal images. Build the
        # action client lazily so the module still imports where it's missing.
        self.nav = self._make_nav_client()
        self.arm = ActionClient(self, MoveGroup, "move_action")
        self._last_odom: Odometry | None = None
        self._last_scan: LaserScan | None = None
        self._last_image: Image | None = None
        self._last_map: OccupancyGrid | None = None
        self._last_joints: JointState | None = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        # camera: waffle/waffle_pi publish here; burger has no camera (stays None)
        self.create_subscription(Image, "/camera/image_raw", self._on_image, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        # /map is "latched": publishers keep the last map for late joiners,
        # so we must subscribe with matching transient_local durability
        map_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        # TF2: the listener fills the buffer from /tf and /tf_static
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # continuous-teleop state, guarded by the deadman watchdog
        self._deadman_s = deadman_s
        self._target = Twist()
        self._target_stamp = 0.0
        self._teleop_active = False
        # obstacle-safe teleop: block forward motion when something's too close
        self.safe_mode = True
        self.min_obstacle_m = 0.35
        self.create_timer(1.0 / 20.0, self._teleop_tick)  # 20 Hz republish

    def _make_nav_client(self) -> ActionClient | None:
        """Create the NavigateToPose action client if nav2_msgs is installed."""
        try:
            from nav2_msgs.action import NavigateToPose
        except ImportError:
            return None
        return ActionClient(self, NavigateToPose, "navigate_to_pose")

    # ---- subscriptions ----------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _on_image(self, msg: Image) -> None:
        self._last_image = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last_map = msg

    def _on_joints(self, msg: JointState) -> None:
        self._last_joints = msg

    # ---- continuous teleop (used by the web UI) ---------------------------
    def set_velocity(self, linear: float, angular: float) -> None:
        """Set the target velocity. A 20 Hz timer keeps publishing it until a
        newer command arrives or the deadman timeout stops the robot."""
        self._target.linear.x = float(linear)
        self._target.angular.z = float(angular)
        self._target_stamp = self._now()
        self._teleop_active = True

    def _front_distance(self) -> float:
        """Nearest obstacle within a ±15° forward cone (meters); inf if unknown."""
        s = self._last_scan
        if s is None:
            return float("inf")
        best = float("inf")
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r <= s.range_min:
                continue
            ang = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
            if ang <= 15 or ang >= 345:
                best = min(best, r)
        return best

    def _teleop_tick(self) -> None:
        if not self._teleop_active:
            return
        if self._now() - self._target_stamp > self._deadman_s:
            self.cmd_vel.publish(Twist())  # deadman: no fresh command -> stop
            self._teleop_active = False
            return
        out = Twist()
        out.linear.x = self._target.linear.x
        out.angular.z = self._target.angular.z
        # safety: refuse to drive forward into a close obstacle (turning still allowed)
        if self.safe_mode and out.linear.x > 0 and self._front_distance() < self.min_obstacle_m:
            out.linear.x = 0.0
        self.cmd_vel.publish(out)

    # ---- timed drive (used by the MCP `drive` tool) -----------------------
    def drive_for(self, linear: float, angular: float, seconds: float) -> None:
        """Blocking: drive for `seconds`, then stop. Bypasses teleop state."""
        self._teleop_active = False
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        end = time.time() + max(0.0, seconds)
        while time.time() < end:
            self.cmd_vel.publish(t)
            time.sleep(0.1)
        self.cmd_vel.publish(Twist())

    def stop(self) -> None:
        self._teleop_active = False
        self.cmd_vel.publish(Twist())

    # ---- read helpers -----------------------------------------------------
    def pose(self) -> dict[str, Any]:
        if self._last_odom is None:
            return {"error": "no /odom yet — is the simulation running?"}
        p = self._last_odom.pose.pose.position
        q = self._last_odom.pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        v = self._last_odom.twist.twist
        return {"x": round(p.x, 3), "y": round(p.y, 3), "yaw_deg": round(yaw, 1),
                "vx": round(v.linear.x, 3), "wz": round(v.angular.z, 3)}

    def laser_summary(self) -> dict[str, Any]:
        s = self._last_scan
        if s is None:
            return {"error": "no /scan yet — is the sim running (with a lidar)?"}

        def nearest(center_deg: float, width_deg: float = 30.0) -> float | None:
            vals = []
            for i, r in enumerate(s.ranges):
                if not math.isfinite(r) or r <= s.range_min:
                    continue
                ang = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
                d = min((ang - center_deg) % 360.0, (center_deg - ang) % 360.0)
                if d <= width_deg / 2.0:
                    vals.append(r)
            return round(min(vals), 3) if vals else None

        return {"front": nearest(0), "left": nearest(90),
                "back": nearest(180), "right": nearest(270),
                "num_points": len(s.ranges)}

    def _image_bgr(self):
        """Convert the latest /camera/image_raw to a BGR numpy array, or None.
        cv2/numpy are imported lazily so this module still loads in minimal
        environments (e.g. the .mcpb bundle) that don't have them."""
        msg = self._last_image
        if msg is None:
            return None, "no camera image — use a waffle model (burger has no camera) and check the sim"
        try:
            import numpy as np
            import cv2
        except ImportError:
            return None, "opencv/numpy not available in this environment"
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        enc = msg.encoding
        try:
            if enc in ("rgb8", "bgr8"):
                img = arr.reshape(msg.height, msg.width, 3)
                if enc == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif enc == "mono8":
                img = arr.reshape(msg.height, msg.width)
            else:
                return None, f"unsupported image encoding '{enc}'"
        except ValueError:
            return None, "image buffer/size mismatch"
        return img, None

    def save_camera_jpeg(self, path: str) -> dict[str, Any]:
        img, err = self._image_bgr()
        if err:
            return {"error": err}
        import cv2
        cv2.imwrite(path, img)
        return {"path": path, "width": int(img.shape[1]), "height": int(img.shape[0])}

    def camera_jpeg_bytes(self):
        """Latest frame as JPEG bytes (for the dashboard), or None."""
        img, err = self._image_bgr()
        if err:
            return None
        import cv2
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else None

    def laser_radar(self, sectors: int = 36) -> list[float | None]:
        """Nearest distance per angular sector (for a mini radar plot in the UI)."""
        s = self._last_scan
        if s is None:
            return []
        buckets: list[list[float]] = [[] for _ in range(sectors)]
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r <= s.range_min:
                continue
            ang = (s.angle_min + i * s.angle_increment) % (2 * math.pi)
            buckets[int(ang / (2 * math.pi) * sectors) % sectors].append(r)
        return [round(min(b), 3) if b else None for b in buckets]

    def get_transform(self, target: str, source: str) -> dict[str, Any]:
        """Latest TF2 transform: where is `source` frame, expressed in `target`?"""
        from rclpy.time import Time
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except Exception as e:
            return {"error": f"{type(e).__name__}: {e}"}
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        return {"target": target, "source": source,
                "x": round(t.x, 4), "y": round(t.y, 4), "z": round(t.z, 4),
                "yaw_deg": round(yaw, 2),
                "qx": round(q.x, 4), "qy": round(q.y, 4),
                "qz": round(q.z, 4), "qw": round(q.w, 4)}

    def map_status(self) -> dict[str, Any]:
        """Summary of the current /map (from SLAM or the map server)."""
        m = self._last_map
        if m is None:
            return {"error": "no /map received — is SLAM or a map server running?"}
        cells = m.data
        n = len(cells)
        free = sum(1 for c in cells if c == 0)
        occupied = sum(1 for c in cells if c > 50)
        unknown = sum(1 for c in cells if c < 0)
        return {"width": m.info.width, "height": m.info.height,
                "resolution_m": round(m.info.resolution, 4),
                "size_m": [round(m.info.width * m.info.resolution, 2),
                           round(m.info.height * m.info.resolution, 2)],
                "origin": [round(m.info.origin.position.x, 2),
                           round(m.info.origin.position.y, 2)],
                "explored_pct": round(100.0 * (n - unknown) / n, 1) if n else 0.0,
                "free_cells": free, "occupied_cells": occupied, "unknown_cells": unknown}

    def joint_states(self) -> dict[str, Any]:
        """Current joint positions (arms, grippers, wheels) from /joint_states."""
        js = self._last_joints
        if js is None:
            return {"error": "no /joint_states — is a robot (sim or real) running?"}
        return {"joints": {n: round(p, 4) for n, p in zip(js.name, js.position)}}

    def move_arm_joints(self, joints: dict[str, float], group: str = "panda_arm",
                        timeout_s: float = 60.0) -> str:
        """Plan + execute a MoveIt joint-space goal (requires move_group running)."""
        if not self.arm.wait_for_server(timeout_sec=3.0):
            return "error: MoveIt 'move_action' server not available — is move_group running?"
        constraints = Constraints()
        for name, pos in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints.append(constraints)
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.planning_options.plan_only = False
        send = self.arm.send_goal_async(goal)
        deadline = time.time() + timeout_s
        while not send.done() and time.time() < deadline:
            time.sleep(0.1)
        handle = send.result()
        if handle is None or not handle.accepted:
            return "error: arm goal rejected"
        result_future = handle.get_result_async()
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.2)
        if not result_future.done():
            return "error: arm motion timed out"
        code = result_future.result().result.error_code.val
        return f"arm motion finished (MoveItErrorCode {code}; 1 = SUCCESS)"

    def navigate_to(self, x: float, y: float, yaw_deg: float = 0.0,
                    timeout_s: float = 120.0) -> str:
        if self.nav is None:
            return "error: nav2_msgs not available — install Nav2 to use navigate_to"
        if not self.nav.wait_for_server(timeout_sec=3.0):
            return "error: Nav2 action server not available — is Nav2 running?"
        from nav2_msgs.action import NavigateToPose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
        goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
        send = self.nav.send_goal_async(goal)
        deadline = time.time() + timeout_s
        while not send.done() and time.time() < deadline:
            time.sleep(0.1)
        handle = send.result()
        if handle is None or not handle.accepted:
            return "error: goal rejected"
        result_future = handle.get_result_async()
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.2)
        if not result_future.done():
            return "error: navigation timed out"
        return f"navigation finished (status={result_future.result().status})"


# --- module-level singleton: init ROS once, spin in a background thread -----
_node: Bridge | None = None
_lock = threading.Lock()


def get_bridge(node_name: str = "claude_ros2_bridge") -> Bridge:
    """Return the shared Bridge, starting rclpy + a spin thread on first call."""
    global _node
    with _lock:
        if _node is None:
            if not rclpy.ok():
                rclpy.init()
            _node = Bridge(node_name=node_name)
            executor = SingleThreadedExecutor()
            executor.add_node(_node)
            threading.Thread(target=executor.spin, daemon=True).start()
        return _node

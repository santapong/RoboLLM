"""Measured-arm adapter using the canonical shared ROS bridge and arm driver.

ROS is imported only when the user explicitly selects hardware mode. USB is
owned by robo_arm_driver; this module never creates a second serial connection.
"""

from __future__ import annotations

import json
import time

import mujoco
import numpy as np

from .interaction import require_fresh
from .simulation import MotionRejected, SimArm


class RosTransport:
    def __init__(self, config):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from std_msgs.msg import Empty, Float64, String
        from std_srvs.srv import SetBool

        from robollm.bridge import get_bridge

        self.config = config
        self.bridge = get_bridge()
        self.action_type = FollowJointTrajectory
        self.float_type = Float64
        self.enable_type = SetBool
        self.empty_type = Empty
        self.status = {}
        self.stamp = 0
        self.goal = None
        self.subscription = self.bridge.create_subscription(
            String, "/arm/status", self._status, 10
        )
        self.gripper_pub = self.bridge.create_publisher(
            Float64, "/arm/gripper_command", 10
        )
        self.heartbeat_pub = self.bridge.create_publisher(
            Empty, "/arm/operator_heartbeat", 10
        )
        self.enable_client = self.bridge.create_client(SetBool, "/arm/enable")
        self.action = ActionClient(
            self.bridge,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

    def _status(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.status = value
                self.stamp = time.monotonic()
        except (ValueError, TypeError):
            pass

    def checked(self, *, enabled=True):
        s = dict(self.status)
        if time.monotonic() - self.stamp > 0.30:
            raise MotionRejected("arm driver status is stale")
        if s.get("profile_hash") != self.config.profile_hash or s.get(
            "joint_names"
        ) != list(self.config.joint_names):
            raise MotionRejected("ROS driver profile/joint mapping mismatch")
        if (
            s.get("state_source") != "commanded"
            or s.get("actuator_backend") != "pca9685"
        ):
            raise MotionRejected("unexpected arm controller capabilities")
        if s.get("controller_fault") != "none" or s.get("last_error"):
            raise MotionRejected("arm controller reported a fault")
        if enabled and not s.get("controller_enabled"):
            raise MotionRejected("PWM is not enabled")
        q = np.asarray(s.get("q_commanded"), float)
        if q.shape != (self.config.njoints,) or not np.isfinite(q).all():
            raise MotionRejected("invalid commanded joint state")
        for joint, value in zip(self.config.joints, q):
            joint.require_rad(value)
        return s

    @staticmethod
    def wait(future, timeout, guard=None):
        end = time.monotonic() + timeout
        while not future.done():
            if guard:
                guard()
            if time.monotonic() > end:
                raise MotionRejected("ROS operation timed out")
            time.sleep(0.02)
        return future.result()

    def enable(self):
        self.checked(enabled=False)
        if not self.enable_client.wait_for_service(timeout_sec=1):
            raise MotionRejected("arm enable service unavailable")
        result = self.wait(
            self.enable_client.call_async(self.enable_type.Request(data=True)), 2
        )
        if not result.success:
            raise MotionRejected(result.message)

    def disable(self):
        if self.goal is not None:
            self.goal.cancel_goal_async()
        if self.enable_client.service_is_ready():
            self.enable_client.call_async(self.enable_type.Request(data=False))

    def heartbeat(self):
        self.heartbeat_pub.publish(self.empty_type())

    def move(self, q, duration, guard):
        from trajectory_msgs.msg import JointTrajectoryPoint

        if not self.action.wait_for_server(timeout_sec=1):
            raise MotionRejected("arm action server unavailable")
        goal = self.action_type.Goal()
        goal.trajectory.joint_names = list(self.config.joint_names)
        p = JointTrajectoryPoint()
        p.positions = list(map(float, q))
        p.time_from_start.sec = int(duration)
        p.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [p]
        self.goal = self.wait(self.action.send_goal_async(goal), 2, guard)
        if not self.goal.accepted:
            raise MotionRejected("driver rejected trajectory")
        try:
            response = self.wait(self.goal.get_result_async(), duration + 3, guard)
            if response.status != 4 or response.result.error_code != 0:
                raise MotionRejected(
                    response.result.error_string or "trajectory did not succeed"
                )
        finally:
            self.goal = None

    def gripper(self, value, guard):
        self.gripper_pub.publish(self.float_type(data=float(value)))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            guard()
            status = self.checked()
            if abs(status["gripper_commanded"] - value) < 0.01:
                return
            time.sleep(0.05)
        raise MotionRejected(
            "gripper command timed out (state is commanded, not measured)"
        )


class HardwareArm(SimArm):
    def __init__(self, config, directory, webcam, transport=None):
        if config.simulation or not config.calibrated:
            raise MotionRejected(
                "hardware mode requires a fully verified physical profile"
            )
        webcam.camera.require_hardware()
        self.webcam = webcam
        self.enabled = False
        self.transport = None
        super().__init__(config, directory)
        self.transport = transport or RosTransport(config)
        # Never enable or send a movement here.

    def _observation(self, tool=False):
        obs = self.webcam.observe()
        require_fresh(obs, calibration_id=self.webcam.camera.id, require_tool=tool)
        return obs

    def enable(self):
        self._observation(tool=True)
        self.last_command = np.asarray(
            self.transport.checked(enabled=False)["q_commanded"], float
        )
        self.transport.enable()
        self.enabled = True
        self.aborted = False

    def keepalive(self):
        if self.enabled:
            try:
                self._observation(tool=True)
                self.transport.heartbeat()
            except Exception:
                self.abort()
                raise

    def tcp(self):
        if self.transport is None:
            return super().tcp()
        obs = self._observation(tool=True)
        return (self.webcam.camera.base @ np.r_[obs["tcp_mat_m"], 1])[:3]

    def object_position(self):
        if self.transport is None:
            return super().object_position()
        obs = self._observation()
        candidates = [o for o in obs.get("objects", []) if o["id"] == "red_box_0"]
        if (
            len(candidates) != 1
            or candidates[0]["pose_source"] != "fiducial_known_box_height"
        ):
            raise MotionRejected("physical pick/place requires visible box tag 70")
        return (self.webcam.camera.base @ np.r_[candidates[0]["position_mat_m"], 1])[:3]

    def move_joints(
        self, q, *, compensate=False, allow_grasp=False, guard=None, callback=None
    ):
        if not self.enabled:
            raise MotionRejected("explicit hardware enable required")

        def checked():
            if self.aborted:
                raise MotionRejected("task aborted")
            self.transport.checked()
            self._observation(tool=True)
            if guard:
                guard()

        checked()
        q = np.asarray(q, float)
        if compensate:
            identification = self.config.profile_data.get("actuator_identification", {})
            if not identification.get("verified") or not identification.get("record"):
                raise MotionRejected(
                    "B/C requires measured actuator identification; provisional compensation is simulation-only"
                )
            predicted = mujoco.MjData(self.model)
            predicted.qpos[:] = self.data.qpos
            predicted.qpos[: self.n] = q
            mujoco.mj_forward(self.model, predicted)
            q = (
                q
                + predicted.qfrc_bias[: self.n]
                / self.geometry["actuator_reference"]["kp"]
            )
        for joint, value in zip(self.config.joints, q):
            joint.require_rad(value)
        self.check_path(q, allow_grasp=allow_grasp)
        duration = (
            max(
                0.3,
                float(
                    np.max(
                        np.abs(q - self.last_command)
                        / [j.max_velocity_rad_s for j in self.config.joints]
                    )
                ),
            )
            + 0.1
        )
        try:
            self.transport.move(q, duration, checked)
            self.last_command = q.copy()
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                checked()
                if callback:
                    callback(self.observation())
                time.sleep(0.05)
        except Exception:
            self.abort()
            raise

    def set_gripper(self, value, *, guard=None):
        self.config.gripper.logical_to_wire(value)
        if not self.enabled:
            raise MotionRejected("explicit hardware enable required")

        def checked():
            self.transport.checked()
            self._observation(tool=True)
            if guard:
                guard()

        try:
            self.transport.gripper(value, checked)
            self.gripper = value
        except Exception:
            self.abort()
            raise

    def abort(self):
        self.aborted = True
        self.enabled = False
        if self.transport is not None:
            self.transport.disable()

    def observation(self):
        if self.transport is None:
            return super().observation()
        status = self.transport.status
        obs = self.webcam.observe()
        tcp = obs.get("tcp_mat_m")
        return {
            "domain": "hardware",
            "state_source": "commanded",
            "q_commanded": status.get("q_commanded"),
            "tcp_position_m": (self.webcam.camera.base @ np.r_[tcp, 1])[:3].tolist()
            if tcp is not None
            else None,
            "tcp_source": "webcam" if tcp is not None else "unavailable",
            "gripper_command": status.get("gripper_commanded"),
            "profile_hash": self.config.profile_hash,
            "calibration_id": self.webcam.camera.id,
        }

    def render(self):
        if self.transport is not None:
            q = self.transport.status.get("q_commanded")
            if q is not None and len(q) == self.n:
                with self.lock:
                    self.data.qpos[: self.n] = q
                    mujoco.mj_forward(self.model, self.data)
        return super().render()

    def close(self):
        self.abort()
        super().close()

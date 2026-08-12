#!/usr/bin/env python3
"""ROS 2 JointTrajectory bridge for the physical RoboLLM arm."""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from .arm_serial import ArmSerial
from .config import ConfigError
from .safety import TrajectorySampler, validate_trajectory


class ArmBridge(Node):
    def __init__(self):
        super().__init__("robo_arm_driver")
        self.declare_parameter("port", "")
        self.declare_parameter("config_file", "")
        self.declare_parameter("enable_on_start", True)

        port = self.get_parameter("port").value or None
        config_file = self.get_parameter("config_file").value or None
        self.arm = ArmSerial(port=port, config_path=config_file)
        self.config = self.arm.config
        self.last_state = self.arm.get_state()
        self.sampler: TrajectorySampler | None = None
        self.last_error = ""

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/arm/status", 10)
        self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.on_trajectory,
            10,
        )
        self.create_timer(1.0 / self.config.control_rate_hz, self.control_tick)

        self.get_logger().info(
            f"Arduino on {self.arm.port}: {self.arm.ping()}; "
            f"config={self.config.source}; calibrated={self.config.calibrated}")
        if self.config.calibrated and self.get_parameter("enable_on_start").value:
            self.last_state = self.arm.enable()
        elif not self.config.calibrated:
            self.get_logger().warn(
                "commissioning lock is active: state is readable but trajectories are rejected")

    @staticmethod
    def _point_time(point) -> float:
        return float(point.time_from_start.sec) + point.time_from_start.nanosec / 1e9

    def on_trajectory(self, msg: JointTrajectory) -> None:
        try:
            self.config.require_calibrated()
            points = validate_trajectory(
                joint_names=msg.joint_names,
                positions=[point.positions for point in msg.points],
                times_s=[self._point_time(point) for point in msg.points],
                initial_positions=self.last_state.q,
                config=self.config,
            )
            self.sampler = TrajectorySampler(self.last_state.q, points, time.monotonic())
            self.last_error = ""
        except (ConfigError, ValueError) as exc:
            self.sampler = None
            self.last_error = str(exc)
            self.get_logger().warn(f"trajectory rejected: {exc}")

    def control_tick(self) -> None:
        try:
            if self.sampler is None:
                self.last_state = self.arm.get_state()
            else:
                target, done = self.sampler.sample(time.monotonic())
                self.last_state = self.arm.set_action(target, self.last_state.gripper)
                if done:
                    self.sampler = None
            self.last_error = ""
        except Exception as exc:
            self.sampler = None
            self.last_error = str(exc)
            self.get_logger().error(f"arm I/O stopped: {exc}")
            try:
                self.arm.relax()
            except Exception:
                pass
        self.publish_state()
        self.publish_status()

    def publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.config.joint_names)
        msg.position = list(self.last_state.q)
        self.joint_pub.publish(msg)

    def publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "calibrated": self.config.calibrated,
            "state_source": self.config.state_source,
            "trajectory_active": self.sampler is not None,
            "last_error": self.last_error,
            "arduino_time_ms": self.last_state.t_arduino_ms,
        }, separators=(",", ":"))
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

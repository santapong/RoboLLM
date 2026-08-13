#!/usr/bin/env python3
"""ROS 2 trajectory controller boundary for the physical RoboLLM arm."""
from __future__ import annotations

import json
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
        self._last_target = tuple(self.last_state.q)
        self._active_goal_handle = None
        self._action_terminal: tuple[str, str] | None = None
        self._action_wake: Future | None = None
        self._action_group = ReentrantCallbackGroup()

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/arm/status", 10)
        self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.on_trajectory,
            10,
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_trajectory,
            goal_callback=self.on_action_goal,
            cancel_callback=self.on_action_cancel,
            handle_accepted_callback=self.on_action_accepted,
            callback_group=self._action_group,
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

    def validate_message(self, msg: JointTrajectory):
        """Validate a ROS trajectory without changing driver state."""
        self.config.require_calibrated()
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            raise ConfigError("scheduled trajectory start times are not supported")
        return validate_trajectory(
            joint_names=msg.joint_names,
            positions=[point.positions for point in msg.points],
            times_s=[self._point_time(point) for point in msg.points],
            initial_positions=self.last_state.q,
            config=self.config,
        )

    def start_trajectory(self, msg: JointTrajectory) -> None:
        points = self.validate_message(msg)
        self.sampler = TrajectorySampler(self.last_state.q, points, time.monotonic())
        self._last_target = tuple(self.last_state.q)
        self._action_terminal = None
        self.last_error = ""

    def on_trajectory(self, msg: JointTrajectory) -> None:
        if self._active_goal_handle is not None:
            self.last_error = "topic trajectory rejected while an action goal is active"
            self.get_logger().warn(self.last_error)
            return
        try:
            self.start_trajectory(msg)
        except (ConfigError, ValueError) as exc:
            self.sampler = None
            self.last_error = str(exc)
            self.get_logger().warn(f"trajectory rejected: {exc}")

    def on_action_goal(self, goal_request) -> GoalResponse:
        if self._active_goal_handle is not None or self.sampler is not None:
            self.get_logger().warn("action goal rejected: another trajectory is active")
            return GoalResponse.REJECT
        if (goal_request.path_tolerance or goal_request.goal_tolerance or
                goal_request.component_path_tolerance or
                goal_request.component_goal_tolerance):
            self.get_logger().warn(
                "action goal rejected: path/goal tolerances require measured joint feedback")
            return GoalResponse.REJECT
        if (goal_request.multi_dof_trajectory.joint_names or
                goal_request.multi_dof_trajectory.points):
            self.get_logger().warn(
                "action goal rejected: multi-DOF trajectories are not supported")
            return GoalResponse.REJECT
        if (goal_request.goal_time_tolerance.sec or
                goal_request.goal_time_tolerance.nanosec):
            self.get_logger().warn(
                "action goal rejected: goal_time_tolerance is not supported")
            return GoalResponse.REJECT
        try:
            self.validate_message(goal_request.trajectory)
        except (ConfigError, ValueError) as exc:
            self.get_logger().warn(f"action goal rejected: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_action_cancel(self, goal_handle) -> CancelResponse:
        if goal_handle is self._active_goal_handle:
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def on_action_accepted(self, goal_handle) -> None:
        """Reserve the controller before scheduling the execution coroutine."""
        self._active_goal_handle = goal_handle
        goal_handle.execute()

    @staticmethod
    def _make_action_result(code: int, message: str):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    def publish_action_feedback(self, goal_handle) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(self.config.joint_names)
        feedback.desired = JointTrajectoryPoint(positions=list(self._last_target))
        feedback.actual = JointTrajectoryPoint(positions=list(self.last_state.q))
        feedback.error = JointTrajectoryPoint(
            positions=[desired - actual for desired, actual in
                       zip(self._last_target, self.last_state.q)])
        goal_handle.publish_feedback(feedback)

    async def wait_for_control_tick(self) -> None:
        """Yield to rclpy until the serial/control timer has run once."""
        wake = Future()
        self._action_wake = wake
        await wake

    async def execute_trajectory(self, goal_handle):
        """Execute one accepted goal; the control timer owns serial I/O."""
        try:
            try:
                self.start_trajectory(goal_handle.request.trajectory)
            except (ConfigError, ValueError) as exc:
                goal_handle.abort()
                return self._make_action_result(
                    FollowJointTrajectory.Result.INVALID_GOAL, str(exc))

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.sampler = None
                    self._action_terminal = None
                    self.last_error = ""
                    goal_handle.canceled()
                    return self._make_action_result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        "trajectory canceled; holding the last commanded state",
                    )

                terminal = self._action_terminal
                if terminal is not None:
                    state, message = terminal
                    if state == "succeeded":
                        goal_handle.succeed()
                        return self._make_action_result(
                            FollowJointTrajectory.Result.SUCCESSFUL, message)
                    goal_handle.abort()
                    return self._make_action_result(
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                        message,
                    )

                self.publish_action_feedback(goal_handle)
                await self.wait_for_control_tick()

            goal_handle.abort()
            return self._make_action_result(
                FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                "ROS shutdown interrupted trajectory execution",
            )
        finally:
            if self._action_wake is not None and not self._action_wake.done():
                self._action_wake.cancel()
            self._action_wake = None
            self.sampler = None
            self._action_terminal = None
            self._active_goal_handle = None

    def control_tick(self) -> None:
        try:
            if self.sampler is None:
                self.last_state = self.arm.get_state()
            else:
                target, done = self.sampler.sample(time.monotonic())
                self._last_target = target
                self.last_state = self.arm.set_action(target, self.last_state.gripper)
                if done:
                    self.sampler = None
                    if self._active_goal_handle is not None:
                        self._action_terminal = ("succeeded", "trajectory completed")
            self.last_error = ""
        except Exception as exc:
            self.sampler = None
            self.last_error = str(exc)
            if self._active_goal_handle is not None:
                self._action_terminal = ("aborted", self.last_error)
            self.get_logger().error(f"arm I/O stopped: {exc}")
            try:
                self.arm.relax()
            except Exception:
                pass
        self.publish_state()
        self.publish_status()
        if self._action_wake is not None and not self._action_wake.done():
            self._action_wake.set_result(None)

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
            "action_active": self._active_goal_handle is not None,
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

#!/usr/bin/env python3
"""In-process rclpy tests for robot_bridge.Bridge — CONTAINER-BOUND.

Run inside the ros2-arm:jazzy image with the repo mounted, e.g.:

    docker run --rm -e ROS_DOMAIN_ID=<id> -e ROS_LOCALHOST_ONLY=1 \
        --net=host --ipc=host -v <repo>:/repo -w /repo ros2-arm:jazzy \
        bash -lc 'source /opt/ros/jazzy/setup.bash && pytest tests/integration/ros/ -q'

This directory is deliberately SEPARATE from the native ``tests/`` tree so the
fast, no-ROS CI gate (``pytest tests/``) never collects an rclpy suite.

What is covered (all deterministic, no real sleeping near the deadman window):
  * 20 Hz teleop republish tick — the watchdog timer is wired at 20 Hz and each
    tick republishes the current target velocity.
  * deadman stop — after the *injected* clock advances past ``deadman_s`` the
    next tick publishes a zero Twist and deactivates teleop, with real wall time
    kept far below ``deadman_s``.
  * safe_mode forward-block vs /scan — a close forward obstacle zeroes forward
    motion; a far one leaves it untouched. /scan is published with the SAME QoS
    (qos_profile_sensor_data) robot_bridge subscribes with.
  * get_bridge() singleton identity.

Graceful degrade: if rclpy or any message package robot_bridge needs is missing
in the container, the whole module is skipped with a recorded reason rather than
erroring.
"""
from __future__ import annotations

import math
import threading
import time

import pytest

# Threads alive before the suite touches ROS — used to join get_bridge()'s
# background spin thread on teardown (see _rclpy_session).
_BASELINE_THREADS = set(threading.enumerate())

# --- graceful degrade: skip (with reason) if the ROS stack isn't importable ---
rclpy = pytest.importorskip("rclpy", reason="rclpy not available (run inside ros2-arm:jazzy)")
pytest.importorskip("geometry_msgs.msg", reason="geometry_msgs missing")
pytest.importorskip("sensor_msgs.msg", reason="sensor_msgs missing")

try:
    from robollm import bridge as robot_bridge
    from robollm.bridge import Bridge, get_bridge
except Exception as exc:  # pragma: no cover - environment guard
    pytest.skip(f"robot_bridge import failed: {exc!r}", allow_module_level=True)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


# --------------------------------------------------------------------------- #
# Fixtures / helpers
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="session", autouse=True)
def _rclpy_session():
    """Bring rclpy up once for the whole module.

    Importing ``ros2_mcp_server`` would call get_bridge() -> rclpy.init() at
    import time (it drives a live node); we do NOT import it here. Instead the
    suite owns rclpy's lifecycle so nothing depends on a live ROS graph.
    """
    started = False
    if not rclpy.ok():
        rclpy.init()
        started = True
    yield
    # Clean teardown. get_bridge() (exercised by the singleton test) spins a
    # SingleThreadedExecutor in a daemon thread; if that thread is still spinning
    # when rclpy shuts down, C-level finalisation aborts (SIGABRT / exit 134).
    # Stop it explicitly so the process exits 0.
    node = getattr(robot_bridge, "_node", None)
    if node is not None:
        ex = getattr(node, "executor", None)
        if ex is not None:
            ex.shutdown()  # signal the spin loop to exit
        # Join the daemon spin thread BEFORE rclpy.shutdown so no thread is
        # still touching the context during C-level finalisation (that race is
        # what aborts the process). Baseline diff isolates get_bridge's thread.
        for t in threading.enumerate():
            if t not in _BASELINE_THREADS and t is not threading.current_thread():
                t.join(timeout=5.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        robot_bridge._node = None
    if started and rclpy.ok():
        rclpy.shutdown()


class FakeClock:
    """A manually advanced monotonic clock, injected as Bridge(time_source=...)."""

    def __init__(self, t0: float = 1000.0) -> None:
        self.t = float(t0)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


class CmdVelSink:
    """A tiny node that records every Twist seen on /cmd_vel (matching QoS)."""

    def __init__(self, name: str) -> None:
        self.node = rclpy.create_node(name)
        self.msgs: list[Twist] = []
        # robot_bridge publishes /cmd_vel with default reliable depth-10 QoS.
        self.node.create_subscription(Twist, "/cmd_vel", self._cb, 10)

    def _cb(self, msg: Twist) -> None:
        self.msgs.append(msg)

    def pump(self, seconds: float = 0.05) -> None:
        rclpy.spin_once(self.node, timeout_sec=seconds)

    def drain(self, rounds: int = 5) -> None:
        for _ in range(rounds):
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def clear(self) -> None:
        self.msgs.clear()

    def destroy(self) -> None:
        self.node.destroy_node()


_UID = [0]


def _uniq(prefix: str) -> str:
    _UID[0] += 1
    return f"{prefix}_{_UID[0]}"


def _make_scan(front_range: float) -> LaserScan:
    """A 360-beam scan; only beam 0 (0 deg, dead ahead) carries `front_range`."""
    s = LaserScan()
    s.angle_min = 0.0
    s.angle_increment = math.radians(1.0)
    s.range_min = 0.05
    s.range_max = 10.0
    s.ranges = [float("inf")] * 360
    s.ranges[0] = float(front_range)
    return s


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #
def test_teleop_20hz_republish_tick():
    """The watchdog timer is registered at 20 Hz and each tick republishes the
    current target velocity."""
    clock = FakeClock()
    bridge = Bridge(node_name=_uniq("br_tick"), time_source=clock)
    sink = CmdVelSink(_uniq("sink_tick"))
    try:
        # 20 Hz proof: a timer with a 50 ms (50_000_000 ns) period exists.
        periods = [t.timer_period_ns for t in bridge.timers]
        assert 50_000_000 in periods, f"expected a 20 Hz (50ms) timer, got {periods}"

        bridge.safe_mode = False  # isolate republish from obstacle logic
        bridge.set_velocity(0.25, 0.1)

        # Drive the tick callback directly (deterministic — no wall-clock racing).
        for _ in range(3):
            bridge._teleop_tick()
            sink.drain()

        assert sink.msgs, "no /cmd_vel republished by teleop tick"
        last = sink.msgs[-1]
        assert last.linear.x == pytest.approx(0.25)
        assert last.angular.z == pytest.approx(0.1)
        # Every republished message carried the same target (true republish).
        assert all(m.linear.x == pytest.approx(0.25) for m in sink.msgs)
    finally:
        sink.destroy()
        bridge.destroy_node()


def test_deadman_stops_after_injected_time_advance():
    """After the injected clock passes deadman_s, the next tick stops the robot —
    and the test itself never sleeps anywhere near deadman_s of real time."""
    clock = FakeClock()
    bridge = Bridge(node_name=_uniq("br_dead"), time_source=clock)
    sink = CmdVelSink(_uniq("sink_dead"))
    try:
        bridge.safe_mode = False
        wall_start = time.monotonic()

        # Fresh command -> tick republishes the (non-zero) target.
        bridge.set_velocity(0.3, 0.0)
        bridge._teleop_tick()
        sink.drain()
        assert sink.msgs[-1].linear.x == pytest.approx(0.3)
        assert bridge._teleop_active is True

        # Advance ONLY the injected clock past the deadman window.
        sink.clear()
        clock.advance(bridge._deadman_s + 0.5)
        bridge._teleop_tick()
        sink.drain()

        wall_elapsed = time.monotonic() - wall_start
        assert sink.msgs, "deadman tick published nothing"
        stop = sink.msgs[-1]
        assert stop.linear.x == 0.0 and stop.angular.z == 0.0, "deadman did not zero the Twist"
        assert bridge._teleop_active is False, "teleop should deactivate after deadman"
        # The whole test advanced simulated time by >deadman_s but real time by
        # only a sliver — prove we never slept the deadman window away.
        assert wall_elapsed < bridge._deadman_s, (
            f"test used {wall_elapsed:.3f}s real >= deadman_s {bridge._deadman_s}s"
        )
    finally:
        sink.destroy()
        bridge.destroy_node()


def _feed_scan_until(bridge, sink, front_range, predicate, tries=200):
    """Publish a /scan (sensor-data QoS) until robot_bridge reflects `predicate`."""
    pub = sink.node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    scan = _make_scan(front_range)
    try:
        for _ in range(tries):
            pub.publish(scan)
            rclpy.spin_once(bridge, timeout_sec=0.01)
            if bridge._last_scan is not None and predicate(bridge._front_distance()):
                return True
        return False
    finally:
        sink.node.destroy_publisher(pub)


def test_safe_mode_blocks_forward_near_obstacle():
    """safe_mode zeroes forward motion when /scan shows a close forward obstacle,
    and leaves it untouched when the obstacle is far. /scan uses the same QoS
    (qos_profile_sensor_data) robot_bridge subscribes with."""
    clock = FakeClock()
    bridge = Bridge(node_name=_uniq("br_safe"), time_source=clock)
    sink = CmdVelSink(_uniq("sink_safe"))
    try:
        assert bridge.safe_mode is True  # default
        assert bridge.min_obstacle_m == pytest.approx(0.35)

        # ---- close obstacle (0.20 m < 0.35 m) -> forward blocked, turn kept ----
        got = _feed_scan_until(bridge, sink, 0.20, lambda d: d < bridge.min_obstacle_m)
        assert got, "close /scan never reached the bridge (QoS mismatch?)"
        sink.clear()
        bridge.set_velocity(0.4, 0.2)
        bridge._teleop_tick()
        sink.drain()
        blocked = sink.msgs[-1]
        assert blocked.linear.x == 0.0, "safe_mode failed to block forward motion"
        assert blocked.angular.z == pytest.approx(0.2), "turning should still be allowed"

        # ---- far obstacle (5.0 m) -> forward motion allowed through ----
        got = _feed_scan_until(bridge, sink, 5.0, lambda d: d > bridge.min_obstacle_m)
        assert got, "far /scan never reached the bridge"
        sink.clear()
        bridge.set_velocity(0.4, 0.0)
        bridge._teleop_tick()
        sink.drain()
        allowed = sink.msgs[-1]
        assert allowed.linear.x == pytest.approx(0.4), "forward motion wrongly blocked when clear"
    finally:
        sink.destroy()
        bridge.destroy_node()


def test_get_bridge_singleton_identity():
    """get_bridge() returns the one shared node regardless of the name argument."""
    a = get_bridge(_uniq("singleton"))
    b = get_bridge("a-different-name")
    assert a is b
    assert isinstance(a, Bridge)

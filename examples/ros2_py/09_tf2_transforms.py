#!/usr/bin/env python3
"""09 · TF2 — the transform tree, the backbone of ALL robot geometry.

Every robot is a tree of coordinate frames (map → odom → base_link → laser →
camera → gripper…). TF2 tracks how they move relative to each other over time,
so you can ask: "this obstacle is 1 m in front of the LASER — where is that in
the MAP?" Nav2, MoveIt, SLAM — everything is built on this.

Self-contained (no sim needed): this script
  1. broadcasts a STATIC transform  base_link → laser   (sensor bolted on)
  2. broadcasts a DYNAMIC transform map → base_link     (robot driving a circle)
  3. uses a listener to look up map → laser and convert a point the laser sees
     into map coordinates — watch the same laser point trace the circle.

    .venv/bin/python examples/ros2_py/09_tf2_transforms.py

With the sim running instead, inspect the real tree:
    ros2 run tf2_tools view_frames        # renders frames.pdf
    ros2 run tf2_ros tf2_echo odom base_link
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)


class TfDemo(Node):
    def __init__(self):
        super().__init__("tf2_demo")
        # --- 1) static: the laser is bolted 5 cm forward, 10 cm up from base
        static = TransformStamped()
        static.header.frame_id = "base_link"
        static.child_frame_id = "laser"
        static.transform.translation.x = 0.05
        static.transform.translation.z = 0.10
        static.transform.rotation.w = 1.0
        self.static_bc = StaticTransformBroadcaster(self)
        self.static_bc.sendTransform(static)

        # --- 2) dynamic: the robot drives a 0.5 m circle in the map
        self.bc = TransformBroadcaster(self)
        self.t0 = time.time()
        self.create_timer(1.0 / 20.0, self.tick)   # broadcast at 20 Hz

        # --- 3) listener side: buffer collects everything broadcast
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def tick(self):
        t = time.time() - self.t0
        yaw = t  # robot also rotates as it circles
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = 0.5 * math.cos(t)
        tf.transform.translation.y = 0.5 * math.sin(t)
        tf.transform.rotation.z = math.sin(yaw / 2)
        tf.transform.rotation.w = math.cos(yaw / 2)
        self.bc.sendTransform(tf)


def main():
    rclpy.init()
    node = TfDemo()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # a point the laser "sees" 1 m straight ahead of itself
    pt = PointStamped()
    pt.header.frame_id = "laser"
    pt.point.x = 1.0

    print("point at 1 m in front of the LASER, expressed in MAP coordinates:")
    successes = 0
    for i in range(6):
        time.sleep(0.5)
        try:
            # where is the laser in the map right now?
            tf = node.buffer.lookup_transform("map", "laser", Time())
            # convert the laser-frame point into the map frame
            in_map = node.buffer.transform(pt, "map")
            x, y = tf.transform.translation.x, tf.transform.translation.y
            print(f"  laser at map({x:+.2f}, {y:+.2f})  →  point at map"
                  f"({in_map.point.x:+.2f}, {in_map.point.y:+.2f})")
            successes += 1
        except Exception as e:  # buffer may need a moment to fill
            print(f"  (waiting for tf: {type(e).__name__})")

    ok = successes >= 3
    print("\nRESULT:", "PASS — transform tree built, looked up, and used" if ok else "FAIL")
    executor.shutdown()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()

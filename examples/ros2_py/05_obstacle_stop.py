#!/usr/bin/env python3
"""05 · Reactive control: drive forward, STOP before hitting a wall.

Needs the sim running: scripts/launch/simulation/turtlebot.sh
Concepts: combining a subscription (/scan) with a publisher (/cmd_vel) — this is
the heart of a robot behavior. "Closed loop": what we sense changes what we do.

The lidar's first ranges point forward; we watch the narrow front cone and stop
when the nearest reading drops below STOP_DISTANCE.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

STOP_DISTANCE = 0.5   # metres
SPEED = 0.15          # m/s


class ObstacleStop(Node):
    def __init__(self):
        super().__init__("obstacle_stop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.control)   # 10 Hz control loop
        self.front = float("inf")

    def on_scan(self, s: LaserScan):
        # nearest valid reading within ±15° of straight ahead
        best = float("inf")
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r <= s.range_min:
                continue
            ang = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
            if ang <= 15 or ang >= 345:
                best = min(best, r)
        self.front = best

    def control(self):
        t = Twist()
        if self.front > STOP_DISTANCE:
            t.linear.x = SPEED
            self.get_logger().info(f"clear ahead ({self.front:.2f} m) — driving", throttle_duration_sec=1.0)
        else:
            self.get_logger().warn(f"obstacle at {self.front:.2f} m — stopping", throttle_duration_sec=1.0)
        self.pub.publish(t)


def main():
    rclpy.init()
    node = ObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

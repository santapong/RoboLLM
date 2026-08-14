#!/usr/bin/env python3
"""04 · Make the TurtleBot drive in a square (open-loop timed control).

Needs the sim running: scripts/launch/simulation/turtlebot.sh
Concepts: publishing geometry_msgs/Twist to /cmd_vel, sequencing motions with time.
This is "open loop" — no feedback — so the square will drift. 05 does it better.
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Square(Node):
    def __init__(self):
        super().__init__("drive_square")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def send(self, lin, ang, secs):
        t = Twist(); t.linear.x = float(lin); t.angular.z = float(ang)
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            self.pub.publish(t); time.sleep(0.1)
        self.pub.publish(Twist())                     # stop between legs
        time.sleep(0.3)

    def run(self):
        for leg in range(4):
            self.get_logger().info(f"leg {leg+1}/4: forward")
            self.send(0.15, 0.0, 3.0)                 # ~0.45 m forward
            self.get_logger().info("turn 90°")
            self.send(0.0, 0.5, 3.14)                 # ~90° at 0.5 rad/s


def main():
    rclpy.init()
    node = Square()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

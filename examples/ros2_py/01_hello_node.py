#!/usr/bin/env python3
"""01 · The smallest useful ROS 2 node: a timer that logs.

Run (with ROS 2 sourced):
    python3 01_hello_node.py
Concepts: rclpy.init, a Node subclass, create_timer, get_logger, spin.
"""
import rclpy
from rclpy.node import Node


class Hello(Node):
    def __init__(self):
        super().__init__("hello_node")
        self.n = 0
        self.create_timer(1.0, self.tick)          # call tick() every 1 s
        self.get_logger().info("hello_node started — Ctrl-C to quit")

    def tick(self):
        self.n += 1
        self.get_logger().info(f"tick {self.n}")


def main():
    rclpy.init()
    node = Hello()
    try:
        rclpy.spin(node)                            # process callbacks until Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

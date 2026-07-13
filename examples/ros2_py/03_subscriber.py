#!/usr/bin/env python3
"""03 · Subscriber: react to messages on a topic.

Run 02_publisher.py in one terminal and this in another — watch it receive.
Concepts: create_subscription, the callback pattern.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.create_subscription(String, "/chatter", self.on_msg, 10)
        self.get_logger().info("listening on /chatter …")

    def on_msg(self, msg: String):
        self.get_logger().info(f"heard: {msg.data}")


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

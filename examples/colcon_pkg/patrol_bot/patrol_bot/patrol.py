"""patrol node — drive a square patrol, size/speed set by ROS parameters.

This is example 04 (drive a square) grown up into a real package:
  - installed as an executable:   ros2 run patrol_bot patrol
  - configured by parameters:     ros2 run patrol_bot patrol --ros-args -p side_m:=0.6
  - started by a launch file:     ros2 launch patrol_bot patrol.launch.py
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Patrol(Node):
    def __init__(self):
        super().__init__("patrol")
        self.declare_parameter("side_m", 0.45)     # square side length
        self.declare_parameter("speed", 0.15)      # forward m/s
        self.declare_parameter("laps", 1)          # how many squares
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.speed = self.get_parameter("speed").value
        self.forward_s = self.get_parameter("side_m").value / self.speed
        self.turn_s = (math.pi / 2) / 0.5          # 90° at 0.5 rad/s
        self.legs_left = 4 * self.get_parameter("laps").value

        # state machine driven by a timer: FORWARD <-> TURN until legs run out
        self.state = "FORWARD"
        self.state_end = self.now_s() + self.forward_s
        self.create_timer(0.1, self.tick)          # 10 Hz control loop
        self.get_logger().info(
            f"patrolling: {self.legs_left // 4} lap(s), side {self.forward_s * self.speed:.2f} m")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        t = Twist()
        if self.legs_left <= 0:
            self.pub.publish(t)                    # zero twist = stop
            self.get_logger().info("patrol complete")
            raise SystemExit(0)
        if self.now_s() >= self.state_end:         # switch state
            if self.state == "FORWARD":
                self.state, dur = "TURN", self.turn_s
            else:
                self.state, dur = "FORWARD", self.forward_s
                self.legs_left -= 1
            self.state_end = self.now_s() + dur
        if self.state == "FORWARD":
            t.linear.x = self.speed
        else:
            t.angular.z = 0.5
        self.pub.publish(t)


def main():
    rclpy.init()
    node = Patrol()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

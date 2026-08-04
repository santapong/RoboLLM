#!/usr/bin/env python3
"""ROS 2 <-> Arduino arm bridge (arm-fw 2.0). Run on the Pi 5 (or laptop).

Subscribes  /arm/command       std_msgs/Float64MultiArray  (target angles, deg)
Publishes   /arm/joint_states  sensor_msgs/JointState      (10 Hz, radians,
            MEASURED state — with encoders wired this is the truth, not the
            command; until then the firmware stub reports the slewed target)

    bash -c 'source /opt/ros/jazzy/setup.bash && python3 arm_bridge_node.py'
    ros2 topic pub -1 /arm/command std_msgs/msg/Float64MultiArray \
        "{data: [90, 45, 120, 90, 90, 90]}"

Params: port (auto), joint_names (joint0..joint5), enable_on_start (true).
This is the real-hardware twin of robot_bridge.py's sim arm path: the LLM/ROS
side plans, a thin node streams targets to the actuator.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from arm_serial import NJOINTS, ArmSerial


class ArmBridge(Node):
    def __init__(self):
        super().__init__("arm_bridge")
        self.declare_parameter("port", "")
        self.declare_parameter("joint_names",
                               [f"joint{i}" for i in range(NJOINTS)])
        self.declare_parameter("enable_on_start", True)

        port = self.get_parameter("port").value or None
        self.names = list(self.get_parameter("joint_names").value)
        self.arm = ArmSerial(port)
        self.get_logger().info(f"Arduino on {self.arm.port}: {self.arm.ping()}")
        if self.get_parameter("enable_on_start").value:
            self.arm.enable()

        self.create_subscription(Float64MultiArray, "/arm/command",
                                 self.on_command, 10)
        self.pub = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.create_timer(0.1, self.publish_states)

    def on_command(self, msg: Float64MultiArray):
        if len(msg.data) < NJOINTS:
            self.get_logger().warn(f"/arm/command needs {NJOINTS} values")
            return
        # topic speaks firmware degrees; the driver owns the rad<->deg mapping
        q = [self.arm._deg_to_rad(i, float(msg.data[i])) for i in range(NJOINTS)]
        self.arm.set_action(q, self.arm.get_state().gripper)

    def publish_states(self):
        st = self.arm.get_state()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.names
        js.position = list(st.q)
        self.pub.publish(js)


def main():
    rclpy.init()
    node = ArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.close()   # close() relaxes the servos — safe on exit


if __name__ == "__main__":
    main()

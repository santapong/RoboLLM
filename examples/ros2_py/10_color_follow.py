#!/usr/bin/env python3
"""10 · Vision → action: chase a colored object with the camera.

The first CLOSED LOOP THROUGH PIXELS: camera image → find the biggest red blob →
steer toward it → drive until it looks close → stop. This tiny pipeline
(perceive → decide → act) is the same shape as the big VLA models, just with
HSV thresholds instead of a neural network.

Self-test (no sim, verifies the vision math on a synthetic image):
    .venv/bin/python examples/ros2_py/10_color_follow.py --test

Live (needs a camera model — burger has none):
    TURTLEBOT3_MODEL=waffle_pi scripts/launch/simulation/turtlebot.sh     # terminal 1
    .venv/bin/python examples/ros2_py/10_color_follow.py   # terminal 2
Then drop something red in front of the robot in Gazebo and watch it chase.
"""
import argparse

import cv2
import numpy as np


# ---------- pure vision (testable without ROS) ------------------------------
def find_red_blob(bgr: np.ndarray):
    """Return (cx_norm, area_frac) of the largest red region, or None.
    cx_norm: horizontal center, -1 (far left) .. +1 (far right).
    area_frac: blob area as a fraction of the image (a crude 'how close')."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # red wraps around the hue axis, so combine both ends
    mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) \
         | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    big = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(big)
    if area < 50:                      # ignore speckle
        return None
    m = cv2.moments(big)
    cx = m["m10"] / m["m00"]
    h, w = bgr.shape[:2]
    return (cx / w) * 2.0 - 1.0, area / (w * h)


def control_law(blob, cruise=0.15):
    """Vision result → (linear, angular). The 'decide' step."""
    if blob is None:
        return 0.0, 0.4                # lost it: rotate in place to search
    cx_norm, area = blob
    if area > 0.10:                    # blob fills 10% of the view: close enough
        return 0.0, 0.0
    return cruise, -1.2 * cx_norm      # steer proportionally toward the blob


# ---------- self-test --------------------------------------------------------
def self_test():
    img = np.zeros((240, 320, 3), np.uint8)
    cv2.rectangle(img, (200, 80), (260, 160), (0, 0, 255), -1)   # red box, right side
    blob = find_red_blob(img)
    assert blob is not None, "blob not found"
    cx, area = blob
    assert cx > 0.2, f"blob should be right of center, got {cx:+.2f}"
    lin, ang = control_law(blob)
    assert lin > 0 and ang < 0, "should drive forward while turning right"
    assert control_law(None) == (0.0, 0.4), "lost blob should search"
    big = np.zeros((240, 320, 3), np.uint8)
    big[:, :] = (0, 0, 255)                                       # all red = very close
    assert control_law(find_red_blob(big)) == (0.0, 0.0), "close blob should stop"
    print(f"blob at cx={cx:+.2f}, area={area:.3f} → drive({lin:.2f}, {ang:+.2f})")
    print("RESULT: PASS — vision + control law verified on synthetic images")


# ---------- live ROS node ----------------------------------------------------
def live():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image

    class ColorFollow(Node):
        def __init__(self):
            super().__init__("color_follow")
            self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Image, "/camera/image_raw", self.on_image,
                                     qos_profile_sensor_data)
            self.get_logger().info("chasing red… (Ctrl-C to stop)")

        def on_image(self, msg: Image):
            if msg.encoding not in ("rgb8", "bgr8"):
                return
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            lin, ang = control_law(find_red_blob(img))
            t = Twist()
            t.linear.x, t.angular.z = lin, ang
            self.pub.publish(t)

    rclpy.init()
    node = ColorFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(__import__("geometry_msgs.msg", fromlist=["Twist"]).Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", action="store_true", help="verify vision math, no ROS")
    if ap.parse_args().test:
        self_test()
    else:
        live()

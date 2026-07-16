#!/usr/bin/env python3
"""Vision-guided sorting v2 — the camera REALLY solves the positions.

Differences from sort_demo.py (which faked detection as truth + noise):

  1. A synthetic camera IMAGE is rendered (pinhole projection of the scene).
  2. Real OpenCV runs on that image: HSV threshold -> contours ->
     minAreaRect -> centroid pixel.
  3. Position is SOLVED from pixels: back-project the centroid ray and
     intersect it with the table plane (see table_camera.py for the math).
  4. Motion uses a TRAPEZOIDAL VELOCITY PROFILE — how real robots follow
     trajectories: accelerate at a_max, cruise at v_max, decelerate so you
     arrive at zero velocity:   v_allowed = min(v_max, sqrt(2 a d_remaining))
     The Cartesian path is still a straight line; IK converts each
     interpolated pose to joint angles at 50 Hz, and THOSE go to the servos.

The GUI shows: the camera image with detection overlays, the solved
pixel->3D math for every cube, the live commanded speed, and the serial
stream to the (virtual) Arduino.

Run via: ros2 launch examples/panda_arm/05_vision_sort.launch.py
"""
import math
import os
import sys
import threading

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from PyQt5 import QtCore, QtGui, QtWidgets
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from panda_kinematics import PandaKinematics, rot_axis, rpy_from_matrix
from sort_demo import (BINS, CUBE_SIZE, CUBE_Z, FINGER_LEN, GRASP_Z, GRIP_CLOSED,
                       GRIP_OPEN, HOVER_Z, READY_POSE, SERIAL_PORT, TICK_HZ,
                       RosBackend, SerialLink, make_link, spawn_cubes)
from table_camera import TableCamera

V_MAX = 0.25      # m/s  cruise speed
A_MAX = 0.6       # m/s^2 accel/decel
YAW_SPEED = math.radians(60) / TICK_HZ


class VisionSortWindow(QtWidgets.QWidget):
    def __init__(self, ros, kin):
        super().__init__()
        self.ros, self.kin = ros, kin
        self.img_pub = ros.create_publisher(Image, 'table_camera/image_raw', 2)
        self.camera = TableCamera()
        self.R_down = kin.fk(READY_POSE)[:3, :3]
        self.q = READY_POSE.copy()
        self.yaw = 0.0
        self.v = 0.0                       # current Cartesian speed (trapezoid)
        self.finger = GRIP_OPEN
        self.finger_target = GRIP_OPEN
        self.serial = make_link()

        self.round_no = 0
        self.objects = []
        self.frame = None                  # annotated camera frame (BGR)
        self.bin_counts = {k: 0 for k in BINS}
        self.steps = []
        self.step_idx = 0
        self.dwell = 0
        self.tx_count = self.rx_count = 0

        self.setWindowTitle('Panda Vision Sort v2 — position solved from camera pixels')
        self._build_ui()
        self._new_round()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / TICK_HZ))

    # ---------- UI ----------
    def _build_ui(self):
        mono = 'font-family: monospace;'
        root = QtWidgets.QHBoxLayout(self)

        left = QtWidgets.QVBoxLayout()
        left.addWidget(QtWidgets.QLabel(
            '<b>Camera view</b> (synthetic image → real OpenCV detection)'))
        self.img_lbl = QtWidgets.QLabel()
        self.img_lbl.setFixedSize(560, 420)
        self.img_lbl.setScaledContents(True)
        left.addWidget(self.img_lbl)
        self.speed_lbl = QtWidgets.QLabel('v = 0.000 m/s')
        self.speed_lbl.setStyleSheet(mono)
        left.addWidget(self.speed_lbl)
        root.addLayout(left, 3)

        right = QtWidgets.QVBoxLayout()
        self.phase_lbl = QtWidgets.QLabel('starting…')
        self.phase_lbl.setStyleSheet('font-size: 15px; font-weight: bold;')
        right.addWidget(self.phase_lbl)
        self.round_lbl = QtWidgets.QLabel('round 0')
        right.addWidget(self.round_lbl)

        right.addWidget(QtWidgets.QLabel(
            '<b>How each position was solved</b> (pixel → ray → table plane):'))
        self.math_box = QtWidgets.QPlainTextEdit()
        self.math_box.setReadOnly(True)
        self.math_box.setStyleSheet(mono + 'font-size: 11px;')
        self.math_box.setFixedHeight(220)
        right.addWidget(self.math_box)

        right.addWidget(QtWidgets.QLabel('<b>Serial → Arduino</b>'))
        self.monitor = QtWidgets.QPlainTextEdit()
        self.monitor.setReadOnly(True)
        self.monitor.setMaximumBlockCount(200)
        self.monitor.setStyleSheet(mono + 'font-size: 10px;')
        right.addWidget(self.monitor)
        self.stats_lbl = QtWidgets.QLabel('')
        self.stats_lbl.setStyleSheet(mono)
        right.addWidget(self.stats_lbl)
        root.addLayout(right, 3)
        self.resize(1300, 620)

    # ---------- vision: one "camera frame" per round ----------
    def _new_round(self):
        self.round_no += 1
        self.round_lbl.setText(f'round {self.round_no}')
        self.objects = spawn_cubes()
        self.bin_counts = {k: 0 for k in BINS}
        self.steps, self.step_idx = [], 0

        raw = self.camera.render(self.objects, CUBE_SIZE)
        detections = self.camera.detect(raw, CUBE_SIZE)

        # annotate the frame with what the detector found
        frame = raw.copy()
        lines = [f'K: fx=fy={self.camera.FX:.0f}  cx={self.camera.CX:.0f} '
                 f'cy={self.camera.CY:.0f}   camera at {self.camera.POS}']
        for o in self.objects:
            d = detections.get(o['color'])
            if d is None:
                o['status'] = 'failed'
                o['det'] = (0, 0, 0)
                lines.append(f'{o["color"]:6s}: NOT DETECTED — skipping')
                continue
            o['det'] = (d['pos'][0], d['pos'][1], d['yaw'])
            u, v = d['pixel']
            box = cv2.boxPoints(cv2.minAreaRect(d['contour'])).astype(int)
            cv2.polylines(frame, [box], True, (255, 255, 255), 2)
            cv2.drawMarker(frame, (int(u), int(v)), (255, 255, 255),
                           cv2.MARKER_CROSS, 14, 2)
            cv2.putText(frame, f'({u:.0f},{v:.0f})', (int(u) + 10, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            tx, ty, tyaw = o['true']
            err = math.hypot(d['pos'][0] - tx, d['pos'][1] - ty) * 1000
            lines.append(
                f'{o["color"]:6s}: pixel ({u:6.1f},{v:6.1f})'
                f'  →  ray ∩ plane z={CUBE_SIZE:.2f}'
                f'  →  x={d["pos"][0]:.4f} y={d["pos"][1]:.4f}'
                f'  yaw={math.degrees(d["yaw"]):+5.1f}°'
                f'   [err vs truth: {err:.1f} mm]')
        self.frame = frame
        self.math_box.setPlainText('\n'.join(lines))
        self._show_frame()
        self._publish_frame()

    def _show_frame(self):
        rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        self.img_lbl.setPixmap(QtGui.QPixmap.fromImage(
            QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)))

    def _publish_frame(self):
        msg = Image()
        msg.header.stamp = self.ros.get_clock().now().to_msg()
        msg.header.frame_id = 'table_camera'
        msg.height, msg.width = self.frame.shape[:2]
        msg.encoding, msg.step = 'bgr8', self.frame.shape[1] * 3
        msg.data = self.frame.tobytes()
        self.img_pub.publish(msg)

    # ---------- planning (same idea as sort_demo, poses from VISION) ----------
    def _plan_object(self, obj):
        x, y, yaw = obj['det']                 # <- solved from pixels!
        bx, by = BINS[obj['color']]['xy']
        stack = self.bin_counts[obj['color']]
        drop_z = CUBE_Z + stack * CUBE_SIZE + FINGER_LEN
        self.steps = [
            {'type': 'grip', 'label': 'open gripper', 'value': GRIP_OPEN},
            {'type': 'cart', 'label': f'approach {obj["color"]} cube', 'pos': (x, y, HOVER_Z), 'yaw': yaw},
            {'type': 'cart', 'label': 'descend to grasp', 'pos': (x, y, GRASP_Z), 'yaw': yaw},
            {'type': 'grip', 'label': 'grasp', 'value': GRIP_CLOSED, 'attach': obj},
            {'type': 'cart', 'label': 'lift', 'pos': (x, y, HOVER_Z), 'yaw': yaw},
            {'type': 'cart', 'label': f'transfer to {obj["color"].upper()} bin', 'pos': (bx, by, HOVER_Z), 'yaw': 0.0},
            {'type': 'cart', 'label': f'descend (stack {stack})', 'pos': (bx, by, drop_z + 0.005), 'yaw': 0.0},
            {'type': 'grip', 'label': 'release', 'value': GRIP_OPEN, 'detach': obj,
             'bin_pos': (bx, by, CUBE_Z + stack * CUBE_SIZE)},
            {'type': 'cart', 'label': 'retreat', 'pos': (bx, by, HOVER_Z), 'yaw': 0.0},
        ]
        self.step_idx = 0

    # ---------- sequence engine with trapezoidal velocity ----------
    def _advance(self):
        if self.step_idx >= len(self.steps):
            pending = [o for o in self.objects if o['status'] == 'pending']
            if pending:
                self._plan_object(pending[0])
            else:
                self.dwell += 1
                self.phase_lbl.setText('round complete — new cubes + new camera frame…')
                if self.dwell > TICK_HZ * 2:
                    self.dwell = 0
                    self._new_round()
                return
        step = self.steps[self.step_idx]
        self.phase_lbl.setText(step['label'])
        done = False

        if step['type'] == 'cart':
            dyaw = step['yaw'] - self.yaw
            self.yaw += max(-YAW_SPEED, min(YAW_SPEED, dyaw))
            T = self.kin.fk(self.q)
            target = np.array(step['pos'])
            d = target - T[:3, 3]
            dist = np.linalg.norm(d)
            if dist < 2e-3 and abs(dyaw) < 1e-3:
                self.v = 0.0
                done = True
            else:
                # trapezoidal speed profile: ramp up at A_MAX, cruise at
                # V_MAX, and slow down so v hits 0 exactly at the target
                v_allowed = min(V_MAX, math.sqrt(2 * A_MAX * max(dist, 0.0)))
                dv = max(-A_MAX / TICK_HZ, min(A_MAX / TICK_HZ, v_allowed - self.v))
                self.v = max(0.0, self.v + dv)
                step_len = min(self.v / TICK_HZ, dist)
                T_next = np.eye(4)
                T_next[:3, :3] = rot_axis([0, 0, 1], self.yaw) @ self.R_down
                T_next[:3, 3] = T[:3, 3] + (d / dist * step_len if dist > 1e-9 else 0)
                q_sol, ok, _ = self.kin.ik(T_next, q0=self.q, max_iters=30)
                if ok:
                    self.q = q_sol

        elif step['type'] == 'grip':
            self.finger_target = step['value']
            self.dwell += 1
            if self.dwell > TICK_HZ * 0.7:
                self.dwell = 0
                if 'attach' in step:
                    step['attach']['status'] = 'grasped'
                if 'detach' in step:
                    obj = step['detach']
                    obj['status'] = 'sorted'
                    obj['bin_pos'] = step['bin_pos']
                    self.bin_counts[obj['color']] += 1
                done = True

        if done:
            self.step_idx += 1

    # ---------- main tick ----------
    def _tick(self):
        self._advance()
        df = self.finger_target - self.finger
        self.finger += max(-0.002, min(0.002, df))

        self.ros.publish_joints(self.kin.joint_names, self.q, self.finger)
        self.ros.publish_scene(self.objects)
        self.speed_lbl.setText(
            f'commanded speed v = {self.v:.3f} m/s   '
            f'(trapezoid: a={A_MAX} m/s², v_max={V_MAX} m/s)')

        tx = self.serial.send_joints(self.kin, self.q, self.finger)
        self.tx_count += 1
        rx = self.serial.read_rx()
        self.rx_count += len(rx)
        if self.tx_count % (TICK_HZ // 5) == 0:
            self.monitor.appendPlainText('TX  ' + tx)
            for l in rx[-1:]:
                self.monitor.appendPlainText('RX  ' + l)
        if self.tx_count % TICK_HZ == 0:
            self._publish_frame()
        self.stats_lbl.setText(f'TX {self.tx_count}   RX {self.rx_count}')


def main():
    rclpy.init()
    ros = RosBackend()
    threading.Thread(target=rclpy.spin, args=(ros,), daemon=True).start()

    urdf_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'),
        'urdf', 'panda.urdf')
    kin = PandaKinematics(open(urdf_path).read())

    app = QtWidgets.QApplication(sys.argv)
    win = VisionSortWindow(ros, kin)
    win.show()
    code = app.exec_()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

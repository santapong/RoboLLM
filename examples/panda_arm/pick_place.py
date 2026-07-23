#!/usr/bin/env python3
"""Pick-and-place demo: ROS2 + IK + simulated Arduino + data GUI.

Pipeline demonstrated (identical to a real robot):

    [this node / PC]                                [Arduino]
    task plan (pick/place waypoints)
      -> Cartesian interpolation (straight lines)
      -> IK each control tick  -> joint angles (rad)
      -> publish /joint_states  ................ RViz shows the arm
      -> convert rad -> servo degrees 0-180
      -> serial line  "S,d1,...,d7,grip\\n" ....> parses, drives servos
      <- "ACK,<seq>,<millis>"  <................ acknowledgement

The GUI shows every stage: current phase, joint angles in radians AND the
servo degrees actually transmitted, plus a serial monitor of TX/RX traffic.

Run via: ros2 launch examples/panda_arm/03_pick_place.launch.py
"""
import math
import os
import sys
import threading

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from PyQt5 import QtCore, QtWidgets
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from panda_kinematics import PandaKinematics

try:
    import serial
except ImportError:
    serial = None

SERIAL_PORT = os.environ.get('PANDA_SERIAL', '/tmp/ttyPANDA')
TICK_HZ = 50
CART_SPEED = 0.18        # m/s end-effector speed
READY_POSE = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# Scene layout (meters, in panda_link0 frame)
CUBE_SIZE = 0.04
PICK_XY = (0.45, 0.25)
PLACE_XY = (0.45, -0.25)
TABLE_TOP = 0.0
CUBE_Z = TABLE_TOP + CUBE_SIZE / 2          # cube center resting on table
FINGER_LEN = 0.105                          # panda_hand origin -> fingertips
GRASP_Z = CUBE_Z + FINGER_LEN               # hand height when fingers reach cube
HOVER_Z = GRASP_Z + 0.15
GRIP_OPEN, GRIP_CLOSED = 0.035, CUBE_SIZE / 2   # finger joint values


def build_sequence():
    px, py = PICK_XY
    qx, qy = PLACE_XY
    return [
        {'type': 'joint',   'label': '1. HOME (ready pose)', 'q': READY_POSE},
        {'type': 'grip',    'label': '2. OPEN gripper', 'value': GRIP_OPEN},
        {'type': 'cart',    'label': '3. APPROACH above cube', 'pos': (px, py, HOVER_Z)},
        {'type': 'cart',    'label': '4. DESCEND to cube', 'pos': (px, py, GRASP_Z)},
        {'type': 'grip',    'label': '5. GRASP (close gripper)', 'value': GRIP_CLOSED,
         'attach': True},
        {'type': 'cart',    'label': '6. LIFT cube', 'pos': (px, py, HOVER_Z)},
        {'type': 'cart',    'label': '7. TRANSFER to place location', 'pos': (qx, qy, HOVER_Z)},
        {'type': 'cart',    'label': '8. DESCEND to table', 'pos': (qx, qy, GRASP_Z)},
        {'type': 'grip',    'label': '9. RELEASE (open gripper)', 'value': GRIP_OPEN,
         'detach': True},
        {'type': 'cart',    'label': '10. RETREAT', 'pos': (qx, qy, HOVER_Z)},
        {'type': 'joint',   'label': '11. HOME', 'q': READY_POSE},
    ]


class RosBackend(Node):
    def __init__(self):
        super().__init__('panda_pick_place')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'pick_scene', 10)

    def publish_joints(self, names, q, finger):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names) + ['panda_finger_joint1', 'panda_finger_joint2']
        msg.position = list(map(float, q)) + [float(finger)] * 2
        self.joint_pub.publish(msg)

    def publish_scene(self, cube_state, cube_world_xy):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        table = Marker()
        table.header.frame_id = 'panda_link0'
        table.header.stamp = now
        table.ns, table.id, table.type = 'scene', 0, Marker.CUBE
        table.pose.position.x = 0.45
        table.pose.position.z = TABLE_TOP - 0.015
        table.pose.orientation.w = 1.0
        table.scale.x, table.scale.y, table.scale.z = 0.4, 0.75, 0.03
        table.color.r = table.color.g = table.color.b = 0.55
        table.color.a = 1.0
        arr.markers.append(table)

        cube = Marker()
        cube.header.stamp = now
        cube.ns, cube.id, cube.type = 'scene', 1, Marker.CUBE
        cube.scale.x = cube.scale.y = cube.scale.z = CUBE_SIZE
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.1, 0.8, 0.2, 1.0
        cube.pose.orientation.w = 1.0
        if cube_state == 'grasped':
            cube.header.frame_id = 'panda_hand'   # rides along with the hand
            cube.pose.position.z = FINGER_LEN
        else:
            cube.header.frame_id = 'panda_link0'
            cube.pose.position.x, cube.pose.position.y = cube_world_xy
            cube.pose.position.z = CUBE_Z
        arr.markers.append(cube)
        self.marker_pub.publish(arr)


class SerialLink:
    """Streams servo commands to the (real or simulated) Arduino."""

    def __init__(self, port):
        self.ser = None
        self.error = None
        try:
            self.ser = serial.Serial(port, 115200, timeout=0)
        except Exception as e:  # port not up yet — GUI will show it
            self.error = str(e)

    def send_joints(self, kin, q, finger):
        """Convert joint radians to servo degrees and transmit one line."""
        servo = [(qi - lo) / (hi - lo) * 180.0
                 for qi, lo, hi in zip(q, kin.lower, kin.upper)]
        grip = (GRIP_OPEN - finger) / (GRIP_OPEN - GRIP_CLOSED) * 60.0
        line = 'S,' + ','.join(f'{v:.1f}' for v in servo) + f',{grip:.1f}\n'
        if self.ser:
            try:
                self.ser.write(line.encode())
            except Exception as e:
                self.error = str(e)
        return line.strip()

    def read_rx(self):
        if not self.ser:
            return []
        try:
            data = self.ser.read(1024).decode(errors='ignore')
        except Exception:
            return []
        return [l for l in data.split('\n') if l.strip()]


def make_link():
    """Serial link factory: the educational streaming protocol by default,
    or the project's REAL hardware protocol (../../hardware, request-reply
    via ArmSerial) when PANDA_PROTOCOL=uno — see uno_link.py."""
    if os.environ.get('PANDA_PROTOCOL', '').lower() == 'uno':
        from uno_link import UnoLink
        return UnoLink()
    return SerialLink(SERIAL_PORT)



class PickPlaceWindow(QtWidgets.QWidget):
    def __init__(self, ros, kin):
        super().__init__()
        self.ros, self.kin = ros, kin
        self.q = READY_POSE.copy()
        self.finger = GRIP_OPEN
        self.finger_target = GRIP_OPEN
        self.serial = make_link()

        self.seq = build_sequence()
        self.step_idx = 0
        self.running = True  # start the demo immediately
        self.cycles = 0
        self.dwell = 0
        self.cube_state = 'world'      # world | grasped
        self.cube_xy = PICK_XY
        self.tx_count = self.rx_count = 0

        self.setWindowTitle('Panda Pick & Place — PC↔Arduino data flow')
        self._build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / TICK_HZ))

    def _build_ui(self):
        mono = 'font-family: monospace;'
        root = QtWidgets.QHBoxLayout(self)

        left = QtWidgets.QVBoxLayout()
        self.phase_lbl = QtWidgets.QLabel('idle — press Start')
        self.phase_lbl.setStyleSheet('font-size: 16px; font-weight: bold;')
        left.addWidget(self.phase_lbl)
        self.cycle_lbl = QtWidgets.QLabel('cycles completed: 0')
        left.addWidget(self.cycle_lbl)

        self.btn = QtWidgets.QPushButton('⏸ Pause')
        self.btn.clicked.connect(self._toggle)
        left.addWidget(self.btn)

        grp = QtWidgets.QGroupBox('Joint data  (what IK produced → what gets sent)')
        g = QtWidgets.QGridLayout(grp)
        for c, h in enumerate(['joint', 'q [rad]', 'q [deg]', 'servo cmd [0-180°]']):
            g.addWidget(QtWidgets.QLabel(f'<b>{h}</b>'), 0, c)
        self.joint_cells = []
        for i, name in enumerate(self.kin.joint_names):
            g.addWidget(QtWidgets.QLabel(name.replace('panda_', '')), i + 1, 0)
            row = []
            for c in range(1, 4):
                lbl = QtWidgets.QLabel('-')
                lbl.setStyleSheet(mono)
                g.addWidget(lbl, i + 1, c)
                row.append(lbl)
            self.joint_cells.append(row)
        g.addWidget(QtWidgets.QLabel('gripper'), 8, 0)
        self.grip_cell = QtWidgets.QLabel('-')
        self.grip_cell.setStyleSheet(mono)
        g.addWidget(self.grip_cell, 8, 3)
        left.addWidget(grp)
        left.addStretch()
        root.addLayout(left, 2)

        right = QtWidgets.QVBoxLayout()
        port_state = self.serial.error or f'connected: {SERIAL_PORT}'
        hdr = QtWidgets.QLabel(
            f'<b>Serial monitor</b> — 115200 baud, {port_state}<br>'
            'TX = PC→Arduino (servo degrees) · RX = Arduino→PC (ACKs)')
        right.addWidget(hdr)
        self.monitor = QtWidgets.QPlainTextEdit()
        self.monitor.setReadOnly(True)
        self.monitor.setMaximumBlockCount(400)
        self.monitor.setStyleSheet(mono + 'font-size: 11px;')
        right.addWidget(self.monitor)
        self.stats_lbl = QtWidgets.QLabel('TX: 0 lines   RX: 0 lines')
        self.stats_lbl.setStyleSheet(mono)
        right.addWidget(self.stats_lbl)
        root.addLayout(right, 3)
        self.resize(1250, 600)

    def _toggle(self):
        self.running = not self.running
        self.btn.setText('⏸ Pause' if self.running else '▶ Resume')

    # ---------- sequence engine ----------
    def _advance(self):
        step = self.seq[self.step_idx]
        self.phase_lbl.setText(step['label'])
        done = False

        if step['type'] == 'joint':
            d = step['q'] - self.q
            self.q += np.clip(d, -0.02, 0.02)
            done = bool(np.linalg.norm(d) < 1e-3)

        elif step['type'] == 'cart':
            T = self.kin.fk(self.q)
            target = np.array(step['pos'])
            d = target - T[:3, 3]
            dist = np.linalg.norm(d)
            if dist < 2e-3:
                done = True
            else:
                step_len = min(CART_SPEED / TICK_HZ, dist)
                T_next = T.copy()
                T_next[:3, :3] = self.kin.fk(READY_POSE)[:3, :3]  # keep hand down
                T_next[:3, 3] = T[:3, 3] + d / dist * step_len
                q_sol, ok, _ = self.kin.ik(T_next, q0=self.q, max_iters=30)
                if ok:
                    self.q = q_sol

        elif step['type'] == 'grip':
            self.finger_target = step['value']
            self.dwell += 1
            if self.dwell > TICK_HZ * 0.8:      # give fingers time to move
                self.dwell = 0
                if step.get('attach'):
                    self.cube_state = 'grasped'
                if step.get('detach'):
                    self.cube_state = 'world'
                    self.cube_xy = PLACE_XY
                done = True

        if done:
            self.step_idx += 1
            if self.step_idx >= len(self.seq):
                self.step_idx = 0
                self.cycles += 1
                self.cycle_lbl.setText(f'cycles completed: {self.cycles}')
                self.cube_xy = PICK_XY          # reset cube for next cycle
                self.cube_state = 'world'

    # ---------- main tick ----------
    def _tick(self):
        if self.running:
            self._advance()

        df = self.finger_target - self.finger
        self.finger += max(-0.002, min(0.002, df))

        self.ros.publish_joints(self.kin.joint_names, self.q, self.finger)
        self.ros.publish_scene(self.cube_state, self.cube_xy)

        tx_line = self.serial.send_joints(self.kin, self.q, self.finger)
        self.tx_count += 1
        rx = self.serial.read_rx()
        self.rx_count += len(rx)

        # Throttle monitor output to ~5 TX lines/s so it stays readable
        if self.tx_count % (TICK_HZ // 5) == 0:
            self.monitor.appendPlainText('TX  ' + tx_line)
            for l in rx[-1:]:
                self.monitor.appendPlainText('RX  ' + l)
        self.stats_lbl.setText(
            f'TX: {self.tx_count} lines   RX: {self.rx_count} lines '
            f'({TICK_HZ} Hz command stream)')

        for row, qi, lo, hi in zip(self.joint_cells, self.q,
                                   self.kin.lower, self.kin.upper):
            row[0].setText(f'{qi:+.3f}')
            row[1].setText(f'{math.degrees(qi):+7.1f}')
            row[2].setText(f'{(qi - lo) / (hi - lo) * 180.0:6.1f}')
        grip_deg = (GRIP_OPEN - self.finger) / (GRIP_OPEN - GRIP_CLOSED) * 60.0
        self.grip_cell.setText(f'{grip_deg:6.1f}')


def main():
    rclpy.init()
    ros = RosBackend()
    threading.Thread(target=rclpy.spin, args=(ros,), daemon=True).start()

    urdf_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'),
        'urdf', 'panda.urdf')
    kin = PandaKinematics(open(urdf_path).read())

    app = QtWidgets.QApplication(sys.argv)
    win = PickPlaceWindow(ros, kin)
    win.show()
    code = app.exec_()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

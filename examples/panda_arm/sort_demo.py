#!/usr/bin/env python3
"""Advanced demo: vision-guided pick-and-sort with computed grasp poses.

Answers the question "how do you set up the pick location — just xyz/rpy?":

  A grasp target IS a 6-DOF pose (x, y, z, roll, pitch, yaw) — but you don't
  hardcode it. The chain is:

    camera detects object  ->  object pose (x, y, yaw) + color
    grasp pose  = f(object pose):
        position    = (x, y, cube_top + finger_length)
        orientation = top-down, with the wrist YAW-ALIGNED to the cube
                      so the fingers straddle its faces:  R = Rz(yaw) @ R_down
    plus derived poses: approach (above), lift, transfer, drop (stacked by
    how many cubes are already in the bin).

  Here the "camera" is simulated: cubes spawn at random x/y/yaw each round,
  and detection adds realistic measurement noise (±2 mm, ±1°). With a real
  camera you'd get the same numbers from AprilTags or OpenCV.

Everything still streams to the (virtual) Arduino exactly as before.

Run via: ros2 launch examples/panda_arm/04_sort.launch.py
"""
import math
import os
import random
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
from panda_kinematics import PandaKinematics, rot_axis, rpy_from_matrix

try:
    import serial
except ImportError:
    serial = None

SERIAL_PORT = os.environ.get('PANDA_SERIAL', '/tmp/ttyPANDA')
TICK_HZ = 50
CART_SPEED = 0.20          # m/s
YAW_SPEED = math.radians(60) / TICK_HZ   # wrist re-orientation per tick
READY_POSE = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

CUBE_SIZE = 0.04
TABLE_TOP = 0.0
CUBE_Z = TABLE_TOP + CUBE_SIZE / 2
FINGER_LEN = 0.105
GRASP_Z = CUBE_Z + FINGER_LEN
HOVER_Z = GRASP_Z + 0.14
GRIP_OPEN, GRIP_CLOSED = 0.035, CUBE_SIZE / 2

# Cubes spawn in this window (must be reachable and clear of the bins)
SPAWN_X = (0.38, 0.55)
SPAWN_Y = (0.05, 0.32)
SPAWN_YAW = (-math.pi / 4, math.pi / 4)

BINS = {
    'red':   {'xy': (0.30, -0.30), 'rgb': (0.85, 0.15, 0.15)},
    'green': {'xy': (0.45, -0.32), 'rgb': (0.15, 0.75, 0.20)},
    'blue':  {'xy': (0.60, -0.30), 'rgb': (0.20, 0.35, 0.90)},
}
N_CUBES = 3


def spawn_cubes():
    """Random poses (x, y, yaw) + colors — what the world looks like."""
    colors = random.sample(list(BINS.keys()), N_CUBES)
    objs = []
    while len(objs) < N_CUBES:
        x = random.uniform(*SPAWN_X)
        y = random.uniform(*SPAWN_Y)
        if all(math.hypot(x - o['true'][0], y - o['true'][1]) > 0.09 for o in objs):
            objs.append({
                'id': len(objs),
                'color': colors[len(objs)],
                'true': (x, y, random.uniform(*SPAWN_YAW)),
                'det': None,          # filled in by "camera"
                'status': 'pending',  # pending | grasped | sorted
                'bin_pos': None,
            })
    return objs


def simulate_camera(objs):
    """Detection = truth + measurement noise (a real camera is never exact)."""
    for o in objs:
        x, y, yaw = o['true']
        o['det'] = (x + random.gauss(0, 0.002),
                    y + random.gauss(0, 0.002),
                    yaw + random.gauss(0, math.radians(1.0)))


class RosBackend(Node):
    def __init__(self):
        super().__init__('panda_sort_demo')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'pick_scene', 10)

    def publish_joints(self, names, q, finger):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names) + ['panda_finger_joint1', 'panda_finger_joint2']
        msg.position = list(map(float, q)) + [float(finger)] * 2
        self.joint_pub.publish(msg)

    def publish_scene(self, objs):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def base_marker(mid, mtype):
            m = Marker()
            m.header.frame_id = 'panda_link0'
            m.header.stamp = now
            m.ns, m.id, m.type = 'scene', mid, mtype
            m.pose.orientation.w = 1.0
            return m

        table = base_marker(0, Marker.CUBE)
        table.pose.position.x, table.pose.position.z = 0.45, TABLE_TOP - 0.015
        table.scale.x, table.scale.y, table.scale.z = 0.45, 0.85, 0.03
        table.color.r = table.color.g = table.color.b = 0.55
        table.color.a = 1.0
        arr.markers.append(table)

        mid = 1
        for name, b in BINS.items():
            tray = base_marker(mid, Marker.CUBE)
            tray.pose.position.x, tray.pose.position.y = b['xy']
            tray.pose.position.z = TABLE_TOP + 0.002
            tray.scale.x = tray.scale.y = 0.11
            tray.scale.z = 0.004
            tray.color.r, tray.color.g, tray.color.b = b['rgb']
            tray.color.a = 0.55
            arr.markers.append(tray)
            label = base_marker(mid + 10, Marker.TEXT_VIEW_FACING)
            label.text = name.upper()
            label.pose.position.x, label.pose.position.y = b['xy']
            label.pose.position.z = 0.06
            label.scale.z = 0.03
            label.color.r, label.color.g, label.color.b = b['rgb']
            label.color.a = 1.0
            arr.markers.append(label)
            mid += 1

        for o in objs:
            c = base_marker(30 + o['id'], Marker.CUBE)
            c.scale.x = c.scale.y = c.scale.z = CUBE_SIZE
            c.color.r, c.color.g, c.color.b = BINS[o['color']]['rgb']
            c.color.a = 1.0
            if o['status'] == 'grasped':
                c.header.frame_id = 'panda_hand'
                c.pose.position.z = FINGER_LEN
            elif o['status'] == 'sorted':
                c.pose.position.x, c.pose.position.y, c.pose.position.z = o['bin_pos']
            else:
                x, y, yaw = o['true']
                c.pose.position.x, c.pose.position.y, c.pose.position.z = x, y, CUBE_Z
                c.pose.orientation.z = math.sin(yaw / 2)
                c.pose.orientation.w = math.cos(yaw / 2)
            arr.markers.append(c)
        self.marker_pub.publish(arr)


class SerialLink:
    def __init__(self, port):
        self.ser, self.error = None, None
        try:
            self.ser = serial.Serial(port, 115200, timeout=0)
        except Exception as e:
            self.error = str(e)

    def send_joints(self, kin, q, finger):
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



class SortWindow(QtWidgets.QWidget):
    def __init__(self, ros, kin):
        super().__init__()
        self.ros, self.kin = ros, kin
        self.R_down = kin.fk(READY_POSE)[:3, :3]   # top-down hand orientation
        self.q = READY_POSE.copy()
        self.yaw = 0.0                              # current wrist yaw offset
        self.finger = GRIP_OPEN
        self.finger_target = GRIP_OPEN
        self.serial = make_link()

        self.round_no = 1
        self.objects = spawn_cubes()
        simulate_camera(self.objects)
        self.bin_counts = {k: 0 for k in BINS}
        self.steps = []
        self.step_idx = 0
        self.current_obj = None
        self.dwell = 0
        self.tx_count = self.rx_count = 0

        self.setWindowTitle('Panda Vision-Guided Sorting — grasp poses computed from detections')
        self._build_ui()
        self._refresh_table()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / TICK_HZ))

    # ---------- UI ----------
    def _build_ui(self):
        mono = 'font-family: monospace;'
        root = QtWidgets.QHBoxLayout(self)

        left = QtWidgets.QVBoxLayout()
        self.phase_lbl = QtWidgets.QLabel('starting…')
        self.phase_lbl.setStyleSheet('font-size: 15px; font-weight: bold;')
        left.addWidget(self.phase_lbl)
        self.round_lbl = QtWidgets.QLabel('round 1')
        left.addWidget(self.round_lbl)

        left.addWidget(QtWidgets.QLabel('<b>Camera detections</b> (what "vision" reported):'))
        self.table = QtWidgets.QTableWidget(N_CUBES, 5)
        self.table.setHorizontalHeaderLabels(['color', 'x [m]', 'y [m]', 'yaw [°]', 'status'])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setFixedHeight(140)
        left.addWidget(self.table)

        grp = QtWidgets.QGroupBox('Grasp pose computed from detection (the full 6-DOF target)')
        v = QtWidgets.QVBoxLayout(grp)
        self.grasp_lbl = QtWidgets.QLabel('—')
        self.grasp_lbl.setStyleSheet(mono + 'font-size: 12px;')
        v.addWidget(self.grasp_lbl)
        left.addWidget(grp)
        left.addStretch()
        root.addLayout(left, 3)

        right = QtWidgets.QVBoxLayout()
        port_state = self.serial.error or f'connected: {SERIAL_PORT}'
        right.addWidget(QtWidgets.QLabel(
            f'<b>Serial → Arduino</b> ({port_state})'))
        self.monitor = QtWidgets.QPlainTextEdit()
        self.monitor.setReadOnly(True)
        self.monitor.setMaximumBlockCount(300)
        self.monitor.setStyleSheet(mono + 'font-size: 11px;')
        right.addWidget(self.monitor)
        self.stats_lbl = QtWidgets.QLabel('')
        self.stats_lbl.setStyleSheet(mono)
        right.addWidget(self.stats_lbl)
        root.addLayout(right, 2)
        self.resize(1250, 560)

    def _refresh_table(self):
        for r, o in enumerate(self.objects):
            x, y, yaw = o['det']
            vals = [o['color'], f'{x:.3f}', f'{y:.3f}',
                    f'{math.degrees(yaw):+.1f}', o['status']]
            for c, v in enumerate(vals):
                self.table.setItem(r, c, QtWidgets.QTableWidgetItem(v))

    # ---------- grasp-pose computation (the answer to "just xyz/rpy?") ----------
    def _plan_object(self, obj):
        """Turn ONE detected object pose into a full motion plan."""
        x, y, yaw = obj['det']                     # <- from the camera
        bx, by = BINS[obj['color']]['xy']
        stack = self.bin_counts[obj['color']]
        drop_z = CUBE_Z + stack * CUBE_SIZE + FINGER_LEN

        # 6-DOF grasp pose: position above the cube, top-down orientation
        # yaw-aligned so the fingers straddle the cube faces.
        r0, p0, y0 = rpy_from_matrix(rot_axis([0, 0, 1], yaw) @ self.R_down)
        self.grasp_lbl.setText(
            f'object #{obj["id"]} ({obj["color"]}) detected at '
            f'x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):+.1f}°\n'
            f'→ grasp pose:   x={x:.3f}  y={y:.3f}  z={GRASP_Z:.3f}\n'
            f'                roll={math.degrees(r0):+.1f}°  '
            f'pitch={math.degrees(p0):+.1f}°  yaw={math.degrees(y0):+.1f}°\n'
            f'→ drop pose:    x={bx:.3f}  y={by:.3f}  z={drop_z:.3f}'
            f'   (stack level {stack})')

        self.steps = [
            {'type': 'grip', 'label': f'open gripper', 'value': GRIP_OPEN},
            {'type': 'cart', 'label': f'approach {obj["color"]} cube (yaw-aligned)',
             'pos': (x, y, HOVER_Z), 'yaw': yaw},
            {'type': 'cart', 'label': 'descend to grasp pose',
             'pos': (x, y, GRASP_Z), 'yaw': yaw},
            {'type': 'grip', 'label': 'close gripper (grasp)',
             'value': GRIP_CLOSED, 'attach': obj},
            {'type': 'cart', 'label': 'lift', 'pos': (x, y, HOVER_Z), 'yaw': yaw},
            {'type': 'cart', 'label': f'transfer to {obj["color"].upper()} bin',
             'pos': (bx, by, HOVER_Z), 'yaw': 0.0},
            {'type': 'cart', 'label': f'descend to stack level {stack}',
             'pos': (bx, by, drop_z + 0.005), 'yaw': 0.0},
            {'type': 'grip', 'label': 'open gripper (release)',
             'value': GRIP_OPEN, 'detach': obj,
             'bin_pos': (bx, by, CUBE_Z + stack * CUBE_SIZE)},
            {'type': 'cart', 'label': 'retreat', 'pos': (bx, by, HOVER_Z), 'yaw': 0.0},
        ]
        self.step_idx = 0

    # ---------- sequence engine ----------
    def _advance(self):
        if self.step_idx >= len(self.steps):
            pending = [o for o in self.objects if o['status'] == 'pending']
            if pending:
                self.current_obj = pending[0]
                self._plan_object(self.current_obj)
            else:
                self.dwell += 1
                self.phase_lbl.setText('round complete — respawning cubes…')
                if self.dwell > TICK_HZ * 2:
                    self.dwell = 0
                    self.round_no += 1
                    self.round_lbl.setText(f'round {self.round_no}')
                    self.objects = spawn_cubes()
                    simulate_camera(self.objects)   # "camera takes a new frame"
                    self.bin_counts = {k: 0 for k in BINS}
                    self._refresh_table()
                return
        step = self.steps[self.step_idx]
        self.phase_lbl.setText(step['label'])
        done = False

        if step['type'] == 'cart':
            # interpolate wrist yaw alongside the straight-line position move
            dyaw = step['yaw'] - self.yaw
            self.yaw += max(-YAW_SPEED, min(YAW_SPEED, dyaw))
            T = self.kin.fk(self.q)
            target = np.array(step['pos'])
            d = target - T[:3, 3]
            dist = np.linalg.norm(d)
            T_next = np.eye(4)
            T_next[:3, :3] = rot_axis([0, 0, 1], self.yaw) @ self.R_down
            if dist < 2e-3 and abs(dyaw) < 1e-3:
                done = True
            else:
                step_len = min(CART_SPEED / TICK_HZ, dist)
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
                self._refresh_table()
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

        tx = self.serial.send_joints(self.kin, self.q, self.finger)
        self.tx_count += 1
        rx = self.serial.read_rx()
        self.rx_count += len(rx)
        if self.tx_count % (TICK_HZ // 5) == 0:
            self.monitor.appendPlainText('TX  ' + tx)
            for l in rx[-1:]:
                self.monitor.appendPlainText('RX  ' + l)
        self.stats_lbl.setText(f'TX {self.tx_count}   RX {self.rx_count}   ({TICK_HZ} Hz)')


def main():
    rclpy.init()
    ros = RosBackend()
    threading.Thread(target=rclpy.spin, args=(ros,), daemon=True).start()

    urdf_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'),
        'urdf', 'panda.urdf')
    kin = PandaKinematics(open(urdf_path).read())

    app = QtWidgets.QApplication(sys.argv)
    win = SortWindow(ros, kin)
    win.show()
    code = app.exec_()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Interactive FK/IK demo GUI for the Franka Panda (ROS2 + PyQt5).

What it does:
  * Publishes /joint_states so robot_state_publisher + RViz animate the arm.
  * FK panel: move joint sliders -> shows the end-effector pose computed by
    OUR forward kinematics (position, RPY, full 4x4 homogeneous matrix), and
    cross-checks it live against TF (what RViz displays). The two should
    agree to ~1e-6 m, proving the math.
  * IK panel: type a Cartesian target -> damped-least-squares IK solves for
    joint angles, printing every iteration's position/orientation error so
    you can watch it converge. Then animates the arm to the solution.
  * Trajectory demo: solves IK for a circle of Cartesian waypoints and
    animates the arm around it, streaming per-waypoint stats.

Run via: ros2 launch examples/panda_arm/02_kinematics.launch.py
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
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from panda_kinematics import PandaKinematics, make_transform, rpy_from_matrix

READY_POSE = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
PUBLISH_HZ = 50
ANIM_STEP = 0.02  # rad per tick toward goal


class RosBackend(Node):
    """Publishes /joint_states; listens to TF for the FK cross-check."""

    def __init__(self):
        super().__init__('panda_kinematics_gui')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish(self, joint_names, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_names) + ['panda_finger_joint1', 'panda_finger_joint2']
        msg.position = list(map(float, q)) + [0.02, 0.02]
        self.pub.publish(msg)

    def tf_hand_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('panda_link0', 'panda_hand', rclpy.time.Time())
            tr = t.transform.translation
            return np.array([tr.x, tr.y, tr.z])
        except Exception:
            return None


class KinematicsWindow(QtWidgets.QWidget):
    def __init__(self, ros: RosBackend, kin: PandaKinematics):
        super().__init__()
        self.ros = ros
        self.kin = kin
        self.q = READY_POSE.copy()
        self.q_goal = None          # joint-space animation target
        self.traj = None            # list of joint configs to loop through
        self.traj_idx = 0
        self.traj_stats = []

        self.setWindowTitle('Panda FK / IK Demo — ROS2')
        self._build_ui()
        self._sync_sliders_to_q()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / PUBLISH_HZ))

    # ---------- UI construction ----------
    def _build_ui(self):
        mono = 'font-family: monospace;'
        root = QtWidgets.QHBoxLayout(self)

        # -- left: joint sliders (FK input) --
        fk_in = QtWidgets.QGroupBox('Joints  (FK input — drag me)')
        form = QtWidgets.QGridLayout(fk_in)
        self.sliders, self.slider_labels = [], []
        for i, name in enumerate(self.kin.joint_names):
            lo, hi = math.degrees(self.kin.lower[i]), math.degrees(self.kin.upper[i])
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setRange(int(lo * 10), int(hi * 10))
            s.valueChanged.connect(self._on_slider)
            lbl = QtWidgets.QLabel()
            lbl.setStyleSheet(mono)
            lbl.setMinimumWidth(70)
            form.addWidget(QtWidgets.QLabel(name.replace('panda_', '')), i, 0)
            form.addWidget(s, i, 1)
            form.addWidget(lbl, i, 2)
            self.sliders.append(s)
            self.slider_labels.append(lbl)
        btn_ready = QtWidgets.QPushButton('Reset to ready pose')
        btn_ready.clicked.connect(lambda: self._set_q(READY_POSE.copy()))
        form.addWidget(btn_ready, len(self.sliders), 0, 1, 3)
        root.addWidget(fk_in, 2)

        # -- middle: FK output --
        fk_out = QtWidgets.QGroupBox('Forward kinematics  T = Π  Trans·Rot(rpy)·Rot(axis, qᵢ)')
        v = QtWidgets.QVBoxLayout(fk_out)
        self.fk_pose_lbl = QtWidgets.QLabel()
        self.fk_pose_lbl.setStyleSheet(mono + 'font-size: 13px;')
        self.fk_matrix_lbl = QtWidgets.QLabel()
        self.fk_matrix_lbl.setStyleSheet(mono)
        self.tf_check_lbl = QtWidgets.QLabel()
        self.tf_check_lbl.setStyleSheet(mono)
        v.addWidget(self.fk_pose_lbl)
        v.addWidget(QtWidgets.QLabel('Homogeneous transform  T(base→hand):'))
        v.addWidget(self.fk_matrix_lbl)
        v.addWidget(QtWidgets.QLabel('Cross-check vs TF (robot_state_publisher):'))
        v.addWidget(self.tf_check_lbl)
        v.addStretch()

        traj_box = QtWidgets.QGroupBox('Trajectory demo (IK per waypoint)')
        tv = QtWidgets.QVBoxLayout(traj_box)
        self.btn_traj = QtWidgets.QPushButton('▶ Run circle trajectory')
        self.btn_traj.clicked.connect(self._toggle_trajectory)
        self.traj_lbl = QtWidgets.QLabel('idle')
        self.traj_lbl.setStyleSheet(mono)
        tv.addWidget(self.btn_traj)
        tv.addWidget(self.traj_lbl)
        v.addWidget(traj_box)
        root.addWidget(fk_out, 3)

        # -- right: IK --
        ik_box = QtWidgets.QGroupBox('Inverse kinematics  dq = Jᵀ(JJᵀ+λ²I)⁻¹·e')
        iv = QtWidgets.QVBoxLayout(ik_box)
        grid = QtWidgets.QGridLayout()
        self.ik_fields = {}
        for col, (key, unit) in enumerate([('x', 'm'), ('y', 'm'), ('z', 'm'),
                                           ('roll', '°'), ('pitch', '°'), ('yaw', '°')]):
            grid.addWidget(QtWidgets.QLabel(f'{key} [{unit}]'), 0, col)
            f = QtWidgets.QDoubleSpinBox()
            f.setRange(-1000, 1000)
            f.setDecimals(3)
            f.setSingleStep(0.01 if unit == 'm' else 5.0)
            grid.addWidget(f, 1, col)
            self.ik_fields[key] = f
        iv.addLayout(grid)
        row = QtWidgets.QHBoxLayout()
        btn_cur = QtWidgets.QPushButton('Target = current pose')
        btn_cur.clicked.connect(self._ik_target_from_current)
        self.btn_solve = QtWidgets.QPushButton('Solve IK + move')
        self.btn_solve.clicked.connect(self._solve_ik)
        row.addWidget(btn_cur)
        row.addWidget(self.btn_solve)
        iv.addLayout(row)
        iv.addWidget(QtWidgets.QLabel('Convergence log (per iteration):'))
        self.ik_log = QtWidgets.QPlainTextEdit()
        self.ik_log.setReadOnly(True)
        self.ik_log.setStyleSheet(mono + 'font-size: 11px;')
        iv.addWidget(self.ik_log)
        root.addWidget(ik_box, 3)

        self.resize(1500, 620)

    # ---------- state helpers ----------
    def _set_q(self, q):
        self.q = np.clip(q, self.kin.lower, self.kin.upper)
        self.q_goal = None
        self._sync_sliders_to_q()

    def _sync_sliders_to_q(self):
        for s, qi in zip(self.sliders, self.q):
            s.blockSignals(True)
            s.setValue(int(math.degrees(qi) * 10))
            s.blockSignals(False)
        self._refresh_fk()

    def _on_slider(self):
        self.q = np.array([math.radians(s.value() / 10.0) for s in self.sliders])
        self.q_goal = None
        self.traj = None
        self.btn_traj.setText('▶ Run circle trajectory')
        self._refresh_fk()

    # ---------- FK display ----------
    def _refresh_fk(self):
        for lbl, qi in zip(self.slider_labels, self.q):
            lbl.setText(f'{math.degrees(qi):7.1f}°')
        T = self.kin.fk(self.q)
        p = T[:3, 3]
        r, pt, y = (math.degrees(a) for a in rpy_from_matrix(T[:3, :3]))
        self.fk_pose_lbl.setText(
            f'position  x={p[0]:+.4f}  y={p[1]:+.4f}  z={p[2]:+.4f}  [m]\n'
            f'rpy       r={r:+8.2f}  p={pt:+8.2f}  y={y:+8.2f}  [°]')
        rows = ['  '.join(f'{v:+8.4f}' for v in T[i]) for i in range(4)]
        self.fk_matrix_lbl.setText('\n'.join(rows))

    def _refresh_tf_check(self):
        tf_p = self.ros.tf_hand_pose()
        if tf_p is None:
            self.tf_check_lbl.setText('TF: (waiting…)')
            return
        fk_p = self.kin.fk(self.q)[:3, 3]
        d = np.linalg.norm(tf_p - fk_p)
        self.tf_check_lbl.setText(
            f'TF  pos  x={tf_p[0]:+.4f}  y={tf_p[1]:+.4f}  z={tf_p[2]:+.4f}\n'
            f'|FK − TF| = {d:.2e} m   {"✓ match" if d < 1e-3 else "(arm still moving)"}')

    # ---------- IK ----------
    def _ik_target_from_current(self):
        T = self.kin.fk(self.q)
        r, p, y = rpy_from_matrix(T[:3, :3])
        for key, val in zip(('x', 'y', 'z'), T[:3, 3]):
            self.ik_fields[key].setValue(val)
        for key, val in zip(('roll', 'pitch', 'yaw'), (r, p, y)):
            self.ik_fields[key].setValue(math.degrees(val))

    def _solve_ik(self):
        xyz = [self.ik_fields[k].value() for k in ('x', 'y', 'z')]
        rpy = [math.radians(self.ik_fields[k].value()) for k in ('roll', 'pitch', 'yaw')]
        T_target = make_transform(xyz, rpy)
        q_sol, ok, log = self.kin.ik(T_target, q0=self.q)

        lines = [f'target: pos {np.round(xyz, 3)}  rpy(°) '
                 f'{np.round([math.degrees(a) for a in rpy], 1)}']
        for e in log:
            lines.append(f'  it {e["iter"]:3d}   |e_pos| {e["pos_err"]:.6f} m'
                         f'   |e_rot| {e["rot_err"]:.6f} rad')
        if ok:
            lines.append(f'CONVERGED in {len(log)} iterations')
            lines.append('q_sol (°): ' + '  '.join(f'{math.degrees(v):+.1f}' for v in q_sol))
            lines.append('→ animating arm to solution')
            self.traj = None
            self.q_goal = q_sol
        else:
            lines.append('DID NOT CONVERGE — target may be unreachable '
                         '(outside ~0.85 m reach or violating joint limits)')
        self.ik_log.setPlainText('\n'.join(lines))
        self.ik_log.verticalScrollBar().setValue(self.ik_log.verticalScrollBar().maximum())

    # ---------- trajectory demo ----------
    def _toggle_trajectory(self):
        if self.traj is not None:
            self.traj = None
            self.btn_traj.setText('▶ Run circle trajectory')
            self.traj_lbl.setText('stopped')
            return
        # Circle in the horizontal plane around the ready-pose hand position,
        # keeping the ready-pose orientation. Each waypoint is an IK solve
        # seeded with the previous solution.
        T0 = self.kin.fk(READY_POSE)
        center, R = T0[:3, 3].copy(), T0[:3, :3]
        radius, n_pts = 0.12, 48
        traj, stats, q_seed = [], [], READY_POSE.copy()
        for k in range(n_pts):
            a = 2 * math.pi * k / n_pts
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = center + [radius * math.cos(a), radius * math.sin(a), 0.0]
            q_sol, ok, log = self.kin.ik(T, q0=q_seed, max_iters=80)
            if not ok:
                self.traj_lbl.setText(f'waypoint {k}: IK failed — aborting')
                return
            traj.append(q_sol)
            stats.append((len(log), log[-1]['pos_err']))
            q_seed = q_sol
        self.traj, self.traj_stats, self.traj_idx = traj, stats, 0
        self.q_goal = None
        self.btn_traj.setText('⏹ Stop trajectory')
        self.traj_lbl.setText(f'{n_pts} waypoints solved, radius {radius} m — looping')

    # ---------- main tick: animate + publish + refresh ----------
    def _tick(self):
        if self.traj is not None:
            target = self.traj[self.traj_idx]
            if np.linalg.norm(target - self.q) < 1e-3:
                self.traj_idx = (self.traj_idx + 1) % len(self.traj)
                iters, err = self.traj_stats[self.traj_idx]
                self.traj_lbl.setText(
                    f'waypoint {self.traj_idx + 1}/{len(self.traj)}   '
                    f'IK: {iters} iters, err {err:.1e} m')
            self._step_towards(target, factor=4.0)
        elif self.q_goal is not None:
            self._step_towards(self.q_goal)
            if np.linalg.norm(self.q_goal - self.q) < 1e-4:
                self.q_goal = None

        self.ros.publish(self.kin.joint_names, self.q)
        self._refresh_tf_check()

    def _step_towards(self, target, factor=1.0):
        d = target - self.q
        step = np.clip(d, -ANIM_STEP * factor, ANIM_STEP * factor)
        self.q = np.clip(self.q + step, self.kin.lower, self.kin.upper)
        self._sync_sliders_to_q()


def main():
    rclpy.init()
    ros = RosBackend()
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros,), daemon=True)
    spin_thread.start()

    urdf_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'),
        'urdf', 'panda.urdf')
    kin = PandaKinematics(open(urdf_path).read())

    app = QtWidgets.QApplication(sys.argv)
    win = KinematicsWindow(ros, kin)
    win.show()
    code = app.exec_()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

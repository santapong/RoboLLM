"""Forward/inverse kinematics for the Franka Panda, computed from its URDF.

Forward kinematics (FK):
    The URDF defines each joint as a fixed transform (origin xyz/rpy) from its
    parent link, followed by a rotation of angle q_i about the joint axis.
    The end-effector pose is the product of those transforms down the chain:

        T_0_ee(q) = PROD_i [ Trans(xyz_i) * Rot(rpy_i) * Rot(axis_i, q_i) ]

Jacobian (geometric, 6x7):
    For each revolute joint i with world-frame axis z_i at position p_i:
        J_v[:, i] = z_i x (p_ee - p_i)     (linear velocity part)
        J_w[:, i] = z_i                    (angular velocity part)

Inverse kinematics (IK):
    Damped least squares (Levenberg-Marquardt style) iteration:
        e  = [ p_target - p(q) ,  rotvec( R_target * R(q)^T ) ]   (6-vector)
        dq = J^T (J J^T + lambda^2 I)^-1 e
        q <- clamp(q + dq, joint limits)
    repeated until |e| < tol. Every iteration is recorded so the GUI can
    show exactly how the solution converges.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def rot_axis(axis, angle):
    """Rotation matrix about an arbitrary unit axis (Rodrigues' formula)."""
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [x * x * C + c,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


def rot_rpy(r, p, y):
    """URDF fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return rot_axis([0, 0, 1], y) @ rot_axis([0, 1, 0], p) @ rot_axis([1, 0, 0], r)


def rpy_from_matrix(R):
    """Extract fixed-axis roll/pitch/yaw from a rotation matrix."""
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    if abs(math.cos(pitch)) < 1e-9:  # gimbal lock
        return math.atan2(-R[1, 2], R[1, 1]), pitch, 0.0
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def rotvec_from_matrix(R):
    """Axis-angle (rotation vector) of R — the orientation error term in IK."""
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - math.pi) < 1e-6:  # 180 deg: extract axis from R + I
        axis = np.sqrt(np.maximum(np.zeros(3), (np.diag(R) + 1.0) / 2.0))
        axis *= np.sign([R[2, 1], R[0, 2], R[1, 0]] + np.ones(3) * 1e-12)[:3]
        return axis / (np.linalg.norm(axis) + 1e-12) * angle
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * math.sin(angle)) * angle


def make_transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rot_rpy(*rpy)
    T[:3, 3] = xyz
    return T


class PandaKinematics:
    """FK/IK for the kinematic chain base_link -> tip_link parsed from a URDF string."""

    def __init__(self, urdf_string, base_link='panda_link0', tip_link='panda_hand'):
        root = ET.fromstring(urdf_string)
        by_child = {}
        for j in root.findall('joint'):
            origin = j.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis_el = j.find('axis')
            axis = [float(v) for v in axis_el.get('xyz').split()] if axis_el is not None else [0.0, 0.0, 1.0]
            limit = j.find('limit')
            lower = float(limit.get('lower')) if limit is not None and limit.get('lower') else -math.pi
            upper = float(limit.get('upper')) if limit is not None and limit.get('upper') else math.pi
            by_child[j.find('child').get('link')] = {
                'name': j.get('name'), 'type': j.get('type'),
                'T_fixed': make_transform(xyz, rpy),
                'axis': np.array(axis, dtype=float),
                'lower': lower, 'upper': upper,
                'parent': j.find('parent').get('link'),
            }

        chain, link = [], tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f'No joint chain from {base_link} to {tip_link} (stuck at {link})')
            chain.append(by_child[link])
            link = by_child[link]['parent']
        chain.reverse()
        self.chain = chain
        self.joints = [j for j in chain if j['type'] in ('revolute', 'continuous')]
        self.joint_names = [j['name'] for j in self.joints]
        self.lower = np.array([j['lower'] for j in self.joints])
        self.upper = np.array([j['upper'] for j in self.joints])
        self.ndof = len(self.joints)

    def fk(self, q, return_all=False):
        """Forward kinematics. Returns 4x4 T_base_tip.

        With return_all=True also returns, per revolute joint, its world-frame
        origin p_i and axis z_i (used for the geometric Jacobian), plus the
        list of every intermediate frame for display.
        """
        T = np.eye(4)
        qi = 0
        joint_origins, joint_axes, frames = [], [], []
        for j in self.chain:
            T = T @ j['T_fixed']
            if j['type'] in ('revolute', 'continuous'):
                joint_origins.append(T[:3, 3].copy())
                joint_axes.append(T[:3, :3] @ j['axis'])
                Tq = np.eye(4)
                Tq[:3, :3] = rot_axis(j['axis'], q[qi])
                T = T @ Tq
                qi += 1
            frames.append((j['name'], T.copy()))
        if return_all:
            return T, joint_origins, joint_axes, frames
        return T

    def jacobian(self, q):
        """Geometric Jacobian (6 x ndof) at configuration q."""
        T, origins, axes, _ = self.fk(q, return_all=True)
        p_ee = T[:3, 3]
        J = np.zeros((6, self.ndof))
        for i, (p, z) in enumerate(zip(origins, axes)):
            J[:3, i] = np.cross(z, p_ee - p)
            J[3:, i] = z
        return J

    def ik(self, T_target, q0=None, max_iters=200, tol=1e-5, damping=0.08):
        """Damped-least-squares IK. Returns (q, converged, log).

        log is a list of dicts per iteration: {iter, pos_err, rot_err, q}
        so a GUI can display the whole convergence history.
        """
        q = np.array(q0, dtype=float) if q0 is not None else (self.lower + self.upper) / 2.0
        log = []
        converged = False
        for it in range(max_iters):
            T = self.fk(q)
            e_pos = T_target[:3, 3] - T[:3, 3]
            e_rot = rotvec_from_matrix(T_target[:3, :3] @ T[:3, :3].T)
            e = np.hstack([e_pos, e_rot])
            log.append({'iter': it, 'pos_err': float(np.linalg.norm(e_pos)),
                        'rot_err': float(np.linalg.norm(e_rot)), 'q': q.copy()})
            if np.linalg.norm(e_pos) < tol and np.linalg.norm(e_rot) < 10 * tol:
                converged = True
                break
            J = self.jacobian(q)
            dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), e)
            dq = np.clip(dq, -0.5, 0.5)  # limit step size for stability
            q = np.clip(q + dq, self.lower, self.upper)
        return q, converged, log


if __name__ == '__main__':
    # Self-test: FK at a known pose, then an IK round-trip back to it.
    import subprocess
    urdf_path = subprocess.run(
        ['python3', '-c',
         "from ament_index_python.packages import get_package_share_directory as g;"
         "print(g('moveit_resources_panda_description') + '/urdf/panda.urdf')"],
        capture_output=True, text=True).stdout.strip()
    kin = PandaKinematics(open(urdf_path).read())
    print(f'Chain has {kin.ndof} revolute joints: {kin.joint_names}')

    q_ref = np.array([0.3, -0.5, 0.2, -2.0, 0.1, 1.8, 0.7])
    T_ref = kin.fk(q_ref)
    print(f'FK({np.round(q_ref, 2)}) ->\n  pos {np.round(T_ref[:3, 3], 4)}')

    q_sol, ok, log = kin.ik(T_ref, q0=np.zeros(7))
    T_sol = kin.fk(q_sol)
    print(f'IK converged={ok} in {len(log)} iters, '
          f'final pos_err={log[-1]["pos_err"]:.2e} m, rot_err={log[-1]["rot_err"]:.2e} rad')
    print(f'  target pos {np.round(T_ref[:3, 3], 4)} vs IK pos {np.round(T_sol[:3, 3], 4)}')

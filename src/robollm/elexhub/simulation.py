"""CPU MuJoCo reference implementation with bounded, collision-checked IK."""

from __future__ import annotations

from collections import deque
from functools import wraps
from pathlib import Path
from threading import RLock

import mink
import mujoco
import numpy as np
from robo_arm_driver.config import ConfigError

from .model import build_assets, geometry, mat_to_base, rotation


class MotionRejected(ValueError):
    pass


def synchronized(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)

    return wrapped


class SimArm:
    def __init__(self, config, directory: Path):
        self.config = config
        self.lock = RLock()
        self.geometry = geometry(config)
        build_assets(config, directory)
        self.model = mujoco.MjModel.from_xml_path(str(directory / "elexhub.xml"))
        self.data = mujoco.MjData(self.model)
        self.n = config.njoints
        self.home = np.array([j.raw_to_rad(j.home_deg) for j in config.joints])
        self.last_command = self.home.copy()
        self.gripper = 0.0
        self.renderer = None
        self.aborted = False
        self.reset()

    @synchronized
    def reset(self, object_xy=(0.09, 0.08)):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[: self.n] = self.home
        self.data.ctrl[: self.n] = self.home
        self.data.ctrl[self.n] = 0
        adr = self.model.joint("object_free").qposadr[0]
        self.data.qpos[adr : adr + 3] = mat_to_base(
            self.config, object_xy, self.geometry["object"]["edge_m"] / 2 + 0.0002
        )
        self.data.qpos[adr + 3 : adr + 7] = [1, 0, 0, 0]
        self.last_command = self.home.copy()
        self.gripper = 0
        self.carrying = False
        self.aborted = False
        delay = self.geometry["actuator_reference"]["delay_s"]
        self.queue = deque(
            [
                self.home.copy()
                for _ in range(max(1, round(delay / self.model.opt.timestep)))
            ]
        )
        mujoco.mj_forward(self.model, self.data)

    @synchronized
    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    @synchronized
    def tcp(self):
        return self.data.site("tcp").xpos.copy()

    @synchronized
    def object_position(self):
        return self.data.body("red_box").xpos.copy()

    @synchronized
    def collisions(self, data=None, allow_grasp=False):
        d = self.data if data is None else data
        bad = []
        for c in d.contact:
            if c.dist >= -0.0005:
                continue
            names = {
                self.model.geom(int(c.geom1)).name,
                self.model.geom(int(c.geom2)).name,
            }
            if names == {"red_box_geom", "table"}:
                continue
            if (
                allow_grasp
                and "red_box_geom" in names
                and names.intersection({"finger_left_pad", "finger_right_pad"})
            ):
                continue
            bad.append(sorted(names))
        return bad

    def solve(self, target, *, start=None, downward=False, allow_grasp=False):
        target = np.asarray(target, float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise MotionRejected("non-finite or malformed target")
        if not 0.008 <= target[2] <= 0.42 or np.linalg.norm(target[:2]) > 0.34:
            raise MotionRejected("target outside bounded arm workspace")
        with self.lock:
            q = self.data.qpos.copy()
        q[: self.n] = self.last_command if start is None else start
        kin = mink.Configuration(self.model, q)
        task = mink.FrameTask(
            "tcp",
            "site",
            position_cost=1.0,
            orientation_cost=[0.3, 0.3, 0] if downward else 0.0,
            lm_damping=1e-3,
        )
        rot = (
            np.diag([1, -1, -1])
            if downward
            else kin.data.site("tcp").xmat.reshape(3, 3)
        )
        task.set_target(
            mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rot), target)
        )
        posture = mink.PostureTask(self.model, cost=1e-4)
        posture.set_target(q)
        limits = [
            mink.ConfigurationLimit(self.model),
            mink.VelocityLimit(
                self.model, {j.name: j.max_velocity_rad_s for j in self.config.joints}
            ),
        ]
        for _ in range(400):
            velocity = mink.solve_ik(kin, [task, posture], 0.05, "daqp", limits=limits)
            kin.integrate_inplace(velocity, 0.05)
            err = np.linalg.norm(kin.data.site("tcp").xpos - target)
            alignment = kin.data.site("tcp").xmat.reshape(3, 3)[2, 2]
            if err < 0.0008 and (not downward or alignment < -0.98):
                result = kin.q[: self.n].copy()
                for joint, value in zip(self.config.joints, result):
                    joint.require_rad(value)
                checked = self._carried_pose(kin.data) if self.carrying else kin.data
                if self.collisions(checked, allow_grasp):
                    raise MotionRejected("IK pose intersects robot/table/object")
                return result
        raise MotionRejected(f"IK cannot reach target (residual {err * 1000:.1f} mm)")

    def check_path(self, goal, *, start=None, allow_grasp=False):
        temp = mujoco.MjData(self.model)
        with self.lock:
            temp.qpos[:] = self.data.qpos
        start = self.last_command if start is None else np.asarray(start)
        steps = max(2, int(np.max(np.abs(goal - start)) / 0.01) + 1)
        for alpha in np.linspace(0, 1, steps):
            temp.qpos[: self.n] = start + alpha * (goal - start)
            mujoco.mj_forward(self.model, temp)
            if self.carrying:
                temp = self._carried_pose(temp)
            if self.collisions(temp, allow_grasp):
                raise MotionRejected("interpolated trajectory has a collision")

    def _carried_pose(self, planned):
        """Carry the observed object pose in collision prediction only.

        The live simulator remains free-body contact physics: no weld, reset,
        attachment or success shortcut is applied to the actual object.
        """

        def frame(data, name, site=False):
            view = data.site(name) if site else data.body(name)
            result = np.eye(4)
            result[:3, :3] = view.xmat.reshape(3, 3)
            result[:3, 3] = view.xpos
            return result

        with self.lock:
            offset = np.linalg.inv(frame(self.data, "tcp", True)) @ frame(
                self.data, "red_box"
            )
        result = mujoco.MjData(self.model)
        result.qpos[:] = planned.qpos
        target = frame(planned, "tcp", True) @ offset
        address = self.model.joint("object_free").qposadr[0]
        result.qpos[address : address + 3] = target[:3, 3]
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, target[:3, :3].ravel())
        result.qpos[address + 3 : address + 7] = quat
        mujoco.mj_forward(self.model, result)
        return result

    @synchronized
    def _command(self, q, *, compensate=False):
        q = np.asarray(q, float)
        if compensate:
            predicted = mujoco.MjData(self.model)
            predicted.qpos[:] = self.data.qpos
            predicted.qpos[: self.n] = q
            mujoco.mj_forward(self.model, predicted)
            # Model-based compensation, not a claim of measured servo identification.
            q = (
                q
                + predicted.qfrc_bias[: self.n]
                / self.geometry["actuator_reference"]["kp"]
            )
        for joint, value in zip(self.config.joints, q):
            joint.require_rad(value)
        self.queue.append(q.copy())
        delayed = self.queue.popleft()
        deadband = self.geometry["actuator_reference"]["deadband_rad"]
        update = np.abs(delayed - self.data.ctrl[: self.n]) >= deadband
        self.data.ctrl[: self.n][update] = delayed[update]
        self.data.ctrl[self.n] = (
            self.gripper * self.geometry["gripper"]["opening_m"] / 2
        )
        mujoco.mj_step(self.model, self.data)

    def move_joints(
        self, q, *, compensate=False, allow_grasp=False, guard=None, callback=None
    ):
        q = np.asarray(q, float)
        if q.shape != (self.n,) or not np.isfinite(q).all():
            raise MotionRejected("invalid joint targets")
        self.check_path(q, allow_grasp=allow_grasp)
        start = self.last_command.copy()
        rates = np.array([j.max_velocity_rad_s for j in self.config.joints])
        duration = max(0.25, float(np.max(np.abs(q - start) / rates)))
        count = int(np.ceil(duration / self.model.opt.timestep))
        for i in range(count + 1):
            if self.aborted:
                raise MotionRejected("task aborted")
            if guard is not None and i % 25 == 0:
                guard()
            target = start + min(1, i / count) * (q - start)
            self._command(target, compensate=compensate)
            if self.collisions(allow_grasp=allow_grasp):
                self.abort()
                raise MotionRejected("contact detected during trajectory")
            if callback is not None and i % 25 == 0:
                callback(self.observation())
        self.last_command = q.copy()
        for i in range(250):
            if self.aborted:
                raise MotionRejected("task aborted")
            if guard is not None and i % 25 == 0:
                guard()
            self._command(q, compensate=compensate)
            if self.collisions(allow_grasp=allow_grasp):
                self.abort()
                raise MotionRejected("contact detected while settling")
            if callback is not None and i % 25 == 0:
                callback(self.observation())

    def reach(
        self,
        target,
        condition="A",
        *,
        downward=False,
        allow_grasp=False,
        guard=None,
        callback=None,
    ):
        if condition not in {"A", "B", "C"}:
            raise ValueError("unknown condition")
        goal = self.solve(target, downward=downward, allow_grasp=allow_grasp)
        self.move_joints(
            goal,
            compensate=condition != "A",
            allow_grasp=allow_grasp,
            guard=guard,
            callback=callback,
        )
        corrected = np.asarray(target, float).copy()
        if condition == "C":
            for _ in range(12):
                error = np.asarray(target) - self.tcp()
                if np.linalg.norm(error) < 0.004:
                    break
                correction = 0.7 * error
                correction *= min(1, 0.002 / max(np.linalg.norm(correction), 1e-12))
                corrected += correction
                goal = self.solve(corrected, downward=downward, allow_grasp=allow_grasp)
                self.move_joints(
                    goal,
                    compensate=True,
                    allow_grasp=allow_grasp,
                    guard=guard,
                    callback=callback,
                )
        return self.observation()

    def set_gripper(self, value, *, guard=None):
        if not np.isfinite(value) or not 0 <= value <= 1:
            raise ConfigError("invalid gripper value")
        start = self.gripper
        for i in range(250):
            if self.aborted:
                raise MotionRejected("task aborted")
            if guard and i % 25 == 0:
                guard()
            self.gripper = start + (value - start) * (i + 1) / 250
            self._command(self.last_command, compensate=True)
            if self.collisions(allow_grasp=True):
                self.abort()
                raise MotionRejected("unexpected contact while gripping")
        self.carrying = value > 0.5

    @synchronized
    def abort(self):
        self.aborted = True
        self.last_command = self.data.qpos[: self.n].copy()
        self.data.ctrl[: self.n] = self.last_command

    def verification_samples(self, *, condition, guard, duration=0.5):
        for i in range(round(duration / self.model.opt.timestep)):
            if self.aborted:
                raise MotionRejected("task aborted during verification")
            if i % 25 == 0:
                guard()
            self._command(self.last_command, compensate=condition != "A")
            if self.collisions():
                raise MotionRejected("unexpected contact during verification")
            yield self.tcp(), self.object_position()

    @synchronized
    def observation(self):
        return {
            "domain": "provisional_simulation"
            if self.geometry["status"] == "provisional"
            else "simulation",
            "time_sim_s": float(self.data.time),
            "q_commanded": self.last_command.tolist(),
            "q_simulated": self.data.qpos[: self.n].tolist(),
            "state_source": "simulator",
            "tcp_position_m": self.tcp().tolist(),
            "object_position_m": self.object_position().tolist(),
            "object_position_mat_m": (
                rotation([0, 0, 1], -self.geometry["table"]["mat_yaw_rad"])
                @ (self.object_position() - self.geometry["table"]["mat_origin_m"])
            ).tolist(),
            "gripper_command": self.gripper,
            "profile_hash": self.config.profile_hash,
        }

    @synchronized
    def render(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=480, width=640)
        self.renderer.update_scene(self.data, camera="webcam")
        return self.renderer.render()

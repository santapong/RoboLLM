"""Hardware-free arm-fw 3.0 protocol peer; never accesses I2C or USB."""

from __future__ import annotations

import math
import os
import pty
import threading
import time
import tty


class Controller:
    def __init__(self, config, clock=time.monotonic):
        self.config, self.clock = config, clock
        self.current = [j.home_deg for j in config.joints] + [
            config.gripper.open_percent
        ]
        self.target = self.current[:]
        self.enabled = False
        self.fault = "none"
        self.commissioning = None
        self.started = self.last_motion = self.last_tick = clock()

    def disable(self):
        self.enabled = False
        self.commissioning = None
        self.target = self.current[:]

    def fail(self, reason):
        self.disable()
        self.fault = reason
        return f"! {reason}\n{self.state()}"

    def tick(self):
        now = self.clock()
        dt = min(max(0, now - self.last_tick), 1 / self.config.control_rate_hz)
        self.last_tick = now
        if (
            self.enabled
            and now - self.last_motion > self.config.command_timeout_ms / 1000
        ):
            self.fail("watchdog")
        if not self.enabled:
            return
        rates = [j.max_velocity_deg_s for j in self.config.joints] + [
            self.config.gripper.max_velocity_percent_s
        ]
        for i, rate in enumerate(rates):
            if self.commissioning is None or self.commissioning == i:
                delta = self.target[i] - self.current[i]
                self.current[i] += min(rate * dt, max(-rate * dt, delta))

    def state(self):
        return (
            "s "
            + " ".join(f"{v:.3f}" for v in self.current)
            + f" {int((self.clock() - self.started) * 1000)} {int(self.enabled)} {self.fault}"
        )

    def handle(self, line):
        self.tick()
        fields = line.split()
        if not fields:
            return ""
        cmd, args = fields[0], fields[1:]
        if cmd == "P" and not args:
            runtime = "simulation" if self.config.simulation else "hardware"
            return f"# pong arm-fw 3.0 joints={self.config.njoints} backend=pca9685 profile={self.config.profile_hash} runtime={runtime}"
        if cmd == "Q" and not args:
            return self.state()
        if cmd == "X" and not args:
            self.disable()
            return self.state()
        if cmd == "E" and not args:
            if not self.config.calibrated:
                return self.fail("not_calibrated")
            self.disable()
            self.fault = "none"
            self.enabled = True
            self.last_motion = self.clock()
            return self.state()
        if cmd == "K" and not args:
            if self.fault != "none":
                return self.fail("fault_latched")
            if self.enabled:
                self.target = self.current[:]
                self.last_motion = self.clock()
            return self.state()
        if cmd == "C":
            if self.config.calibrated or not self.config.commissioning_allowed:
                return self.fail("commissioning_disabled")
            try:
                if len(args) != 2:
                    raise ValueError
                index, value = int(args[0]), float(args[1])
                servos = self.config.profile_data["joints"] + [
                    self.config.profile_data["gripper"]
                ]
                if not 0 <= index < len(servos) or not math.isfinite(value):
                    raise ValueError
                s = servos[index]
                if (
                    not s["commission_min"] <= value <= s["commission_max"]
                    or s["commission_max"] - s["commission_min"] > 4
                ):
                    return self.fail("commission_limit")
            except (ValueError, KeyError, TypeError):
                return self.fail("bad_command")
            if self.fault != "none":
                return self.fail("fault_latched")
            if self.commissioning != index:
                self.disable()
            self.commissioning = index
            self.enabled = True
            self.target[index] = value
            self.last_motion = self.clock()
            return self.state()
        if cmd in {"S", "H"}:
            if not self.config.calibrated:
                return self.fail("not_calibrated")
            if (
                not self.enabled
                or self.commissioning is not None
                or self.fault != "none"
            ):
                return self.fail("not_enabled_or_fault_latched")
            try:
                if cmd == "H":
                    if args:
                        raise ValueError
                    values = [j.home_deg for j in self.config.joints] + [
                        self.config.gripper.open_percent
                    ]
                else:
                    values = [float(x) for x in args]
                if len(values) != len(self.current) or not all(
                    math.isfinite(x) for x in values
                ):
                    raise ValueError
                for j, v in zip(self.config.joints, values):
                    j.require_raw(v)
                g = self.config.gripper
                if (
                    not min(g.open_percent, g.closed_percent)
                    <= values[-1]
                    <= max(g.open_percent, g.closed_percent)
                ):
                    raise ValueError
            except (ValueError, RuntimeError):
                return self.fail("unsafe_command")
            self.target = values
            self.last_motion = self.clock()
            return self.state()
        return self.fail("bad_command")


def serve(config):
    controller = Controller(config)
    master, slave = pty.openpty()
    tty.setraw(slave)
    print(f"SIM PCA on {os.ttyname(slave)}", flush=True)
    lock = threading.Lock()

    def ticks():
        while True:
            with lock:
                controller.tick()
            time.sleep(1 / config.control_rate_hz)

    threading.Thread(target=ticks, daemon=True).start()
    buf = b""
    while True:
        buf += os.read(master, 128)
        while b"\n" in buf:
            line, _, buf = buf.partition(b"\n")
            with lock:
                result = controller.handle(line.decode(errors="replace"))
            if result:
                os.write(master, (result + "\n").encode())

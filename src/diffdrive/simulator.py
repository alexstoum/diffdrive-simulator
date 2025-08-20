from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Literal

from .kinematics import Kinematics

@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians

@dataclass
class Limits:
    max_v: float = 1.0
    max_omega: float = 2.0
    max_accel_v: float | None = None
    max_accel_omega: float | None = None

@dataclass
class Action:
    mode: Literal["body", "wheels"] = "body"
    v: float = 0.0
    omega: float = 0.0
    v_l: float = 0.0
    v_r: float = 0.0

class DiffDriveSimulator:
    """Differential-drive 2D simulator (kinematics-only)."""

    def __init__(
        self,
        kin: Kinematics,
        state: RobotState | None = None,
        limits: Limits | None = None,
        integrator: Literal["euler", "rk4"] = "rk4",
        process_noise_std: Tuple[float, float] = (0.0, 0.0),
        record_history: bool = True,
    ) -> None:
        self.kin = kin
        self.state = state or RobotState()
        self.limits = limits or Limits()
        self.integrator = integrator
        self.process_noise_std = process_noise_std
        self.record_history = record_history
        self.history: list[RobotState] = [RobotState(self.state.x, self.state.y, self.state.theta)] if record_history else []
        self._last_cmd: tuple[float, float] | None = None

    def _apply_limits(self, v: float, omega: float, dt: float) -> Tuple[float, float]:
        v = max(-self.limits.max_v, min(self.limits.max_v, v))
        omega = max(-self.limits.max_omega, min(self.limits.max_omega, omega))

        if self._last_cmd is not None:
            last_v, last_omega = self._last_cmd
            if self.limits.max_accel_v is not None:
                dv = v - last_v
                dv = max(-self.limits.max_accel_v * dt, min(self.limits.max_accel_v * dt, dv))
                v = last_v + dv
            if self.limits.max_accel_omega is not None:
                domega = omega - last_omega
                domega = max(-self.limits.max_accel_omega * dt, min(self.limits.max_accel_omega * dt, domega))
                omega = last_omega + domega

        self._last_cmd = (v, omega)
        return v, omega

    def _f(self, s: RobotState, v: float, omega: float) -> tuple[float, float, float]:
        dx = v * math.cos(s.theta)
        dy = v * math.sin(s.theta)
        dtheta = omega
        return dx, dy, dtheta

    def step(self, action: Action, dt: float) -> RobotState:
        if action.mode == "wheels":
            v, omega = self.kin.body_from_wheels(action.v_l, action.v_r)
        else:
            v, omega = action.v, action.omega

        # process noise (if enabled)
        if any(s > 0 for s in self.process_noise_std):
            import random
            v += random.gauss(0, self.process_noise_std[0])
            omega += random.gauss(0, self.process_noise_std[1])

        v, omega = self._apply_limits(v, omega, dt)

        if self.integrator == "euler":
            dx, dy, dtheta = self._f(self.state, v, omega)
            self.state.x += dx * dt
            self.state.y += dy * dt
            self.state.theta += dtheta * dt
        elif self.integrator == "rk4":
            s = self.state
            k1 = self._f(s, v, omega)
            s2 = RobotState(s.x + 0.5 * dt * k1[0], s.y + 0.5 * dt * k1[1], s.theta + 0.5 * dt * k1[2])
            k2 = self._f(s2, v, omega)
            s3 = RobotState(s.x + 0.5 * dt * k2[0], s.y + 0.5 * dt * k2[1], s.theta + 0.5 * dt * k2[2])
            k3 = self._f(s3, v, omega)
            s4 = RobotState(s.x + dt * k3[0], s.y + dt * k3[1], s.theta + dt * k3[2])
            k4 = self._f(s4, v, omega)

            self.state.x += (dt / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
            self.state.y += (dt / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
            self.state.theta += (dt / 6.0) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        else:
            raise ValueError("Unknown integrator")

        # wrap angle to [-pi, pi)
        self.state.theta = (self.state.theta + math.pi) % (2 * math.pi) - math.pi

        if self.record_history:
            self.history.append(RobotState(self.state.x, self.state.y, self.state.theta))

        return self.state
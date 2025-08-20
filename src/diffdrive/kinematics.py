from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class Kinematics:
    """Differential-drive kinematics helper.

    Args:
        wheel_radius: Radius of wheels [m]. (not used directly unless you work with angular speeds)
        wheel_base: Distance between wheels [m].
    """
    wheel_radius: float
    wheel_base: float

    def body_from_wheels(self, v_l: float, v_r: float) -> Tuple[float, float]:
        """Convert left/right wheel linear velocities (m/s) to body (v, omega)."""
        v = 0.5 * (v_r + v_l)
        omega = (v_r - v_l) / self.wheel_base
        return v, omega

    def wheels_from_body(self, v: float, omega: float) -> Tuple[float, float]:
        """Convert body (v, omega) to left/right wheel linear velocities (m/s)."""
        v_r = v + 0.5 * omega * self.wheel_base
        v_l = v - 0.5 * omega * self.wheel_base
        return v_l, v_r

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class PurePursuit:
    """Simple Pure Pursuit controller.

    Args:
        lookahead: Lookahead distance [m].
        nominal_speed: Forward speed to maintain [m/s].
        min_lookahead: Minimum lookahead distance [m].
    """
    lookahead: float = 0.5
    nominal_speed: float = 0.5
    min_lookahead: float = 0.2

    def compute(self, state, path: List[Tuple[float, float]]) -> Tuple[float, float]:
        import numpy as np

        if len(path) < 2:
            return 0.0, 0.0

        P = np.array(path)
        d = np.hypot(P[:, 0] - state.x, P[:, 1] - state.y)
        i_min = int(np.argmin(d))

        Ld = max(self.min_lookahead, self.lookahead)

        target = P[i_min]
        i = i_min
        while i < len(P) - 1 and np.hypot(target[0] - state.x, target[1] - state.y) < Ld:
            i += 1
            target = P[i]

        dx = target[0] - state.x
        dy = target[1] - state.y
        _xt = math.cos(-state.theta) * dx - math.sin(-state.theta) * dy
        yt = math.sin(-state.theta) * dx + math.cos(-state.theta) * dy

        if Ld < 1e-6:
            return 0.0, 0.0

        kappa = 2.0 * yt / (Ld * Ld)
        v = self.nominal_speed
        omega = v * kappa
        return v, omega

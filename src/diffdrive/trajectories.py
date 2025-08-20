from __future__ import annotations
import math
from typing import List, Tuple

def circle_waypoints(radius: float, num_points: int = 200, center=(0.0, 0.0)) -> List[Tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(num_points):
        th = 2.0 * math.pi * i / num_points
        pts.append((center[0] + radius * math.cos(th), center[1] + radius * math.sin(th)))
    return pts

def line_waypoints(start: Tuple[float, float], end: Tuple[float, float], num_points: int = 100) -> List[Tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(num_points):
        t = i / (num_points - 1)
        pts.append((start[0] + t*(end[0]-start[0]), start[1] + t*(end[1]-start[1])))
    return pts

def figure8_waypoints(a: float = 2.0, b: float = 1.0, num_points: int = 400) -> List[Tuple[float, float]]:
    # Lemniscate of Gerono
    pts: list[tuple[float, float]] = []
    for i in range(num_points):
        t = 2.0 * math.pi * i / num_points
        x = a * math.sin(t)
        y = b * math.sin(t) * math.cos(t)
        pts.append((x, y))
    return pts
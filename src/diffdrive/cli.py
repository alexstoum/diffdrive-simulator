from __future__ import annotations
import argparse
from typing import Tuple

from .kinematics import Kinematics
from .simulator import DiffDriveSimulator, RobotState, Limits, Action
from .controllers import PurePursuit
from .trajectories import circle_waypoints, line_waypoints, figure8_waypoints
from .plotting import plot_path_and_traj

def parse_pair(s: str) -> Tuple[float, float]:
    x, y = s.split(",")
    return float(x), float(y)

def main() -> None:
    p = argparse.ArgumentParser(description="Differential Drive Simulator")
    p.add_argument("--controller", default="pure_pursuit", choices=["pure_pursuit"])
    p.add_argument("--path", default="circle", choices=["circle", "line", "figure8"])
    p.add_argument("--radius", type=float, default=3.0, help="Circle radius [m]")
    p.add_argument("--start", type=parse_pair, default="-3,0", help="Line start 'x,y'")
    p.add_argument("--end", type=parse_pair, default="3,0", help="Line end 'x,y'")
    p.add_argument("--duration", type=float, default=20.0, help="Simulation time [s]")
    p.add_argument("--dt", type=float, default=0.02, help="Time step [s]")
    p.add_argument("--integrator", default="rk4", choices=["euler", "rk4"])
    p.add_argument("--lookahead", type=float, default=0.6)
    p.add_argument("--speed", type=float, default=0.7)
    p.add_argument("--save", type=str, default=None, help="Save plot to file")
    p.add_argument("--no-show", action="store_true", help="Do not display the plot window")
    args = p.parse_args()

    # Parse start/end if given as strings by default in argparse defaults
    if isinstance(args.start, str):
        args.start = parse_pair(args.start)
    if isinstance(args.end, str):
        args.end = parse_pair(args.end)

    kin = Kinematics(wheel_radius=0.05, wheel_base=0.3)
    sim = DiffDriveSimulator(kin, state=RobotState(-args.radius, 0.0, 0.0), limits=Limits(max_v=1.0, max_omega=3.0), integrator=args.integrator)

    if args.path == "circle":
        path = circle_waypoints(args.radius, num_points=300)
    elif args.path == "line":
        path = line_waypoints(args.start, args.end, num_points=200)
    else:
        path = figure8_waypoints(a=2.0, b=1.2, num_points=500)

    if args.controller == "pure_pursuit":
        ctrl = PurePursuit(lookahead=args.lookahead, nominal_speed=args.speed)
    else:
        raise ValueError("Unknown controller")

    steps = int(args.duration / args.dt)
    for _ in range(steps):
        v, omega = ctrl.compute(sim.state, path)
        sim.step(Action(mode="body", v=v, omega=omega), args.dt)

    plot_path_and_traj(path, sim.history, save=args.save, show=not args.no_show)

if __name__ == "__main__":
    main()
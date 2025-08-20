import argparse
from diffdrive.kinematics import Kinematics
from diffdrive.simulator import DiffDriveSimulator, RobotState, Limits, Action
from diffdrive.controllers import PurePursuit
from diffdrive.trajectories import circle_waypoints, line_waypoints, figure8_waypoints
from diffdrive.plotting import plot_path_and_traj

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", choices=["circle", "line", "figure8"], default="circle")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--dt", type=float, default=0.02)
    ap.add_argument("--save", type=str, default=None)
    args = ap.parse_args()

    kin = Kinematics(wheel_radius=0.05, wheel_base=0.3)
    sim = DiffDriveSimulator(kin, state=RobotState(-3.0, 0.0, 0.0), limits=Limits(max_v=1.0, max_omega=3.0))

    if args.path == "circle":
        path = circle_waypoints(3.0, 300)
    elif args.path == "line":
        path = line_waypoints((-3.0, 0.0), (3.0, 0.0), 200)
    else:
        path = figure8_waypoints(2.0, 1.2, 500)

    ctrl = PurePursuit(lookahead=0.6, nominal_speed=0.7)
    steps = int(args.duration / args.dt)
    for _ in range(steps):
        v, omega = ctrl.compute(sim.state, path)
        sim.step(Action(mode="body", v=v, omega=omega), args.dt)

    plot_path_and_traj(path, sim.history, save=args.save, show=True)

if __name__ == "__main__":
    main()
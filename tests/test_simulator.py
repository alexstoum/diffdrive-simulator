import math
from diffdrive.kinematics import Kinematics
from diffdrive.simulator import DiffDriveSimulator, RobotState, Limits, Action

def test_straight_line():
    kin = Kinematics(0.05, 0.3)
    sim = DiffDriveSimulator(kin, state=RobotState(0.0, 0.0, 0.0), integrator="rk4")
    dt = 0.01
    for _ in range(100):
        sim.step(Action(mode="body", v=1.0, omega=0.0), dt)
    assert math.isclose(sim.state.x, 1.0, rel_tol=1e-9)
    assert math.isclose(sim.state.y, 0.0, abs_tol=1e-12)

def test_rotation_in_place():
    kin = Kinematics(0.05, 0.3)
    sim = DiffDriveSimulator(kin, state=RobotState(0.0, 0.0, 0.0), integrator="rk4")
    dt = 0.01
    for _ in range(100):
        sim.step(Action(mode="body", v=0.0, omega=math.pi/2), dt)  # 90 deg/s
    assert math.isclose(sim.state.theta, math.pi/2, rel_tol=1e-9)
    assert math.isclose(sim.state.x, 0.0, abs_tol=1e-12)
    assert math.isclose(sim.state.y, 0.0, abs_tol=1e-12)
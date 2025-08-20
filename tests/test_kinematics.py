import math
from diffdrive.kinematics import Kinematics

def test_body_wheels_roundtrip():
    kin = Kinematics(0.05, 0.3)
    v, omega = 0.8, 1.2
    v_l, v_r = kin.wheels_from_body(v, omega)
    v2, o2 = kin.body_from_wheels(v_l, v_r)
    assert abs(v - v2) < 1e-9
    assert abs(omega - o2) < 1e-9

def test_straight_line_mapping():
    kin = Kinematics(0.05, 0.3)
    v_l, v_r = 1.0, 1.0
    v, omega = kin.body_from_wheels(v_l, v_r)
    assert math.isclose(omega, 0.0, abs_tol=1e-12)
    assert math.isclose(v, 1.0, rel_tol=1e-12)
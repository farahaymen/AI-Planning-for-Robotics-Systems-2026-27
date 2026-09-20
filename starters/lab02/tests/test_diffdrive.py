import math, sys; sys.path.insert(0, "starters/lab02")
import pytest
from diffdrive import (WHEEL_RADIUS, WHEEL_SEPARATION, forward_kinematics,
                       integrate_pose, inverse_kinematics, odometry_scale_error,
                       wheel_speed_limits)


def test_equal_wheel_speeds_give_pure_translation():
    v, w = forward_kinematics(10.0, 10.0)
    assert v == pytest.approx(0.5) and w == pytest.approx(0.0)


def test_opposite_wheel_speeds_give_pure_rotation():
    v, w = forward_kinematics(-10.0, 10.0)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(2 * 10.0 * WHEEL_RADIUS / WHEEL_SEPARATION)


def test_kinematics_roundtrip():
    for v, w in [(0.3, 0.0), (0.0, 1.2), (0.45, -0.8), (-0.1, 0.3)]:
        assert forward_kinematics(*inverse_kinematics(v, w)) == pytest.approx((v, w))


def test_straight_line_integration():
    x, y, th = integrate_pose(0, 0, 0, 0.5, 0.0, 2.0)
    assert (x, y, th) == pytest.approx((1.0, 0.0, 0.0))


def test_quarter_circle_lands_on_the_arc_not_the_chord():
    """v=0.5, omega=1.0 for pi/2 seconds traces a quarter circle of radius 0.5."""
    x, y, th = integrate_pose(0, 0, 0, 0.5, 1.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.5, 0.5), abs=1e-9)
    assert th == pytest.approx(math.pi / 2)


def test_full_circle_returns_to_the_start():
    x = y = th = 0.0
    for _ in range(1000):
        x, y, th = integrate_pose(x, y, th, 0.5, 1.0, 2 * math.pi / 1000)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-6)


def test_ten_percent_radius_error_gives_ten_percent_distance_error():
    assert odometry_scale_error(0.055, 0.050) == pytest.approx(1.10)


def test_peak_wheel_speed_is_within_the_command_interface_limit():
    """The ros2_control block allows +/- 20 rad/s. Confirm the limits fit."""
    assert wheel_speed_limits() <= 20.0, "velocity limits demand more than the interface allows"

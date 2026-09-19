"""
Tests for Lab 4, Exercise 4.6. The controllers.

These are the specification. Read them before writing code.

    ARC_CONTROL=control_skeleton python3 -m pytest tests/test_control.py -x -q

Several assert numbers rather than "it worked". That is deliberate. A
controller that drives roughly in the right direction passes any loose test and
is still wrong in a way you will meet in Project 1 rather than here.
"""

from __future__ import annotations

import importlib
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motion import ROBOT, Pose, simulate, step, wrap_angle

C = importlib.import_module(os.environ.get("ARC_CONTROL", "control"))


def straight_path(length=3.0, n=60):
    xs = np.linspace(0.0, length, n)
    return np.column_stack([xs, np.zeros_like(xs)])


def corner_path():
    """Three metres east, then two metres north. A right angle to cut."""
    xs = np.linspace(0.0, 3.0, 60)
    ys = np.linspace(0.0, 2.0, 40)
    return np.vstack([np.column_stack([xs, np.zeros_like(xs)]),
                      np.column_stack([np.full_like(ys, 3.0), ys])])


# ---------------------------------------------------------------------------
# The motion model
# ---------------------------------------------------------------------------

def test_driving_straight_goes_straight():
    p = step(Pose(0, 0, 0), v=0.5, w=0.0, dt=1.0)
    assert p.x == pytest.approx(0.5)
    assert p.y == pytest.approx(0.0)


def test_turning_in_place_does_not_translate():
    p = step(Pose(0, 0, 0), v=0.0, w=1.0, dt=1.0)
    assert p.x == pytest.approx(0.0)
    assert p.y == pytest.approx(0.0)
    assert p.theta == pytest.approx(1.0)


def test_commands_are_clamped_to_what_the_robot_can_do():
    """A controller that asks for 6 rad/s looks excellent and transfers badly."""
    p = step(Pose(0, 0, 0), v=99.0, w=99.0, dt=1.0)
    assert p.x <= ROBOT.max_linear_velocity + 1e-9
    assert abs(p.theta) <= ROBOT.max_angular_velocity + 1e-9


def test_wrap_angle_takes_the_short_way_round():
    """The bug this prevents: a two degree turn executed as 358 degrees."""
    assert wrap_angle(math.radians(181)) == pytest.approx(math.radians(-179), abs=1e-6)
    assert wrap_angle(math.radians(-181)) == pytest.approx(math.radians(179), abs=1e-6)
    assert wrap_angle(0.5) == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# PID
# ---------------------------------------------------------------------------

def test_proportional_term_is_proportional():
    pid = C.PID(kp=2.0, ki=0.0, kd=0.0, output_limit=100.0)
    assert pid(0.5) == pytest.approx(1.0)


def test_zero_error_gives_zero_output():
    pid = C.PID(kp=2.0, ki=1.0, kd=1.0, output_limit=100.0)
    assert pid(0.0) == pytest.approx(0.0)


def test_the_first_call_does_not_produce_a_derivative_kick():
    """With no previous error the derivative is undefined; it must not be huge.

    Using the error itself as the first derivative gives kd*e/dt on cycle one,
    which at dt=0.05 is twenty times the intended gain. On a real robot that is
    a visible lurch the moment the controller starts.
    """
    pid = C.PID(kp=0.0, ki=0.0, kd=1.0, output_limit=1e6)
    assert pid(1.0) == pytest.approx(0.0)


def test_integral_windup_is_clamped():
    """The bug: a robot stuck against a wall storing error, then spinning free.

    Drive a constant error for ten seconds and the integral must stop at the
    limit rather than growing without bound.
    """
    pid = C.PID(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5, output_limit=1e6)
    for _ in range(200):
        out = pid(1.0)
    assert abs(out) <= 0.5 + 1e-9


def test_output_is_clamped_to_the_robots_turn_rate():
    pid = C.PID(kp=100.0, output_limit=ROBOT.max_angular_velocity)
    assert abs(pid(math.pi)) <= ROBOT.max_angular_velocity + 1e-9


def test_pid_wraps_angular_error():
    """An error of 359 degrees is really minus one degree, and must act like it."""
    pid = C.PID(kp=1.0, wrap=True, output_limit=1e6)
    out = pid(math.radians(359))
    assert out < 0
    assert out == pytest.approx(math.radians(-1), abs=1e-6)


def test_reset_clears_state():
    pid = C.PID(kp=0.0, ki=1.0, integral_limit=10.0, output_limit=1e6)
    for _ in range(20):
        pid(1.0)
    pid.reset()
    assert pid(0.0) == pytest.approx(0.0)


def test_heading_controller_reaches_a_point():
    ctl = C.make_heading_controller()
    target = (2.0, 2.0)
    traj = simulate(lambda p: ctl(p, target), Pose(0, 0, 0), steps=600,
                    stop_when=lambda p: math.hypot(target[0] - p.x,
                                                   target[1] - p.y) < 0.15)
    final = traj[-1]
    assert math.hypot(target[0] - final.x, target[1] - final.y) < 0.15


def test_heading_controller_reaches_a_point_behind_it():
    """The case that exposes an unwrapped error: the target is directly behind."""
    ctl = C.make_heading_controller()
    target = (-2.0, 0.0)
    traj = simulate(lambda p: ctl(p, target), Pose(0, 0, 0), steps=900,
                    stop_when=lambda p: math.hypot(target[0] - p.x,
                                                   target[1] - p.y) < 0.15)
    final = traj[-1]
    assert math.hypot(target[0] - final.x, target[1] - final.y) < 0.15


# ---------------------------------------------------------------------------
# Pure Pursuit
# ---------------------------------------------------------------------------

def test_lookahead_point_is_far_enough_ahead():
    path = straight_path()
    point, index = C.find_lookahead(path, Pose(0, 0, 0), lookahead=0.5)
    assert math.hypot(point[0], point[1]) >= 0.5
    assert index > 0


def test_lookahead_falls_back_to_the_last_point_near_the_goal():
    """Otherwise the robot stops short, one lookahead distance from the goal.

    `start_index` is set near the end, as it would be after driving the path.
    From index 0 the point three metres BEHIND the robot is also more than a
    lookahead away and would be returned first, which is precisely the bug the
    carried index exists to prevent.
    """
    path = straight_path()
    point, index = C.find_lookahead(path, Pose(3.0, 0, 0), lookahead=0.5,
                                    start_index=len(path) - 5)
    assert index == len(path) - 1
    assert point[0] == pytest.approx(path[-1][0])


def test_lookahead_does_not_search_backwards():
    """Searching from zero each cycle lets a looping path capture the robot."""
    path = straight_path()
    _, index = C.find_lookahead(path, Pose(1.5, 0, 0), lookahead=0.3, start_index=40)
    assert index >= 40


def test_straight_path_needs_no_steering():
    """On a straight line directly ahead, the curvature must be essentially zero."""
    ctl = C.make_pure_pursuit(lookahead=0.5)
    v, w, _ = ctl(Pose(0, 0, 0), straight_path(), 0)
    assert abs(w) < 1e-6
    assert v > 0


def test_a_path_to_the_left_steers_left():
    """Sign errors here are the most common Pure Pursuit bug and look plausible."""
    ctl = C.make_pure_pursuit(lookahead=0.5, regulate=False)
    path = np.array([[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]])
    _, w, _ = ctl(Pose(0, 0, 0), path, 0)
    assert w > 0


def test_a_path_to_the_right_steers_right():
    ctl = C.make_pure_pursuit(lookahead=0.5, regulate=False)
    path = np.array([[0.0, 0.0], [0.5, -0.5], [1.0, -1.0]])
    _, w, _ = ctl(Pose(0, 0, 0), path, 0)
    assert w < 0


def test_a_target_behind_the_robot_turns_on_the_spot():
    """Driving forwards at something behind you is slow and looks like a bug."""
    ctl = C.make_pure_pursuit(lookahead=0.5)
    path = np.array([[-1.0, 0.0], [-2.0, 0.0]])
    v, w, _ = ctl(Pose(0, 0, 0), path, 0)
    assert v == pytest.approx(0.0)
    assert abs(w) > 0.1


def test_regulation_slows_the_robot_in_a_tight_turn():
    """The Nav2 Regulated Pure Pursuit behaviour, in one assertion."""
    tight = np.array([[0.0, 0.0], [0.30, 0.30], [0.40, 0.60]])
    plain = C.make_pure_pursuit(lookahead=0.4, regulate=False)
    regulated = C.make_pure_pursuit(lookahead=0.4, regulate=True)
    v_plain, _, _ = plain(Pose(0, 0, 0), tight, 0)
    v_reg, _, _ = regulated(Pose(0, 0, 0), tight, 0)
    assert v_reg < v_plain


def test_pure_pursuit_follows_a_straight_path_to_the_end():
    traj, arrived = C.follow_path(straight_path(), Pose(0, 0, 0),
                                  C.make_pure_pursuit())
    assert arrived
    assert C.cross_track_error(straight_path(), traj).max() < 0.10


def test_pure_pursuit_gets_round_a_right_angle():
    path = corner_path()
    traj, arrived = C.follow_path(path, Pose(0, 0, 0), C.make_pure_pursuit(0.45))
    assert arrived
    assert C.cross_track_error(path, traj).max() < 0.30


def test_pure_pursuit_recovers_from_a_bad_start():
    """Real runs do not begin on the path, facing along it."""
    path = corner_path()
    traj, arrived = C.follow_path(path, Pose(-0.4, 0.6, math.pi),
                                  C.make_pure_pursuit(0.45), max_steps=3000)
    assert arrived


# ---------------------------------------------------------------------------
# The measurement, which is the actual lesson
# ---------------------------------------------------------------------------

def test_a_shorter_lookahead_tracks_more_tightly():
    path = corner_path()
    short, _ = C.follow_path(path, Pose(0, 0, 0), C.make_pure_pursuit(0.20), max_steps=3000)
    long_, _ = C.follow_path(path, Pose(0, 0, 0), C.make_pure_pursuit(1.00), max_steps=3000)
    assert C.cross_track_error(path, short).mean() < C.cross_track_error(path, long_).mean()


def test_a_shorter_lookahead_costs_far_more_control_effort():
    """The other half of the trade, and the half a single metric hides.

    With a realistic 5 cm pose uncertainty, a 0.10 m lookahead tracks about
    twice as well as a 1.00 m one and jitters roughly six times as hard. A
    student who optimises cross track error alone builds a robot that follows
    the line beautifully and shakes itself to pieces.
    """
    path = corner_path()
    short, _ = C.follow_path(path, Pose(0, 0, 0), C.make_pure_pursuit(0.10),
                             max_steps=3000, pose_noise=0.05, seed=1)
    long_, _ = C.follow_path(path, Pose(0, 0, 0), C.make_pure_pursuit(1.00),
                             max_steps=3000, pose_noise=0.05, seed=1)
    assert C.control_jitter(short) > 2.0 * C.control_jitter(long_)


def test_control_jitter_ignores_the_wrap_at_pi():
    """Without unwrapping, every pass through pi registers as a vast fake jerk."""
    spin = [Pose(0, 0, wrap_angle(0.2 * i)) for i in range(200)]
    assert C.control_jitter(spin) < 1e-6


def test_cross_track_error_is_zero_on_the_path():
    path = straight_path()
    on_path = [Pose(x, 0.0, 0.0) for x in np.linspace(0, 3, 30)]
    assert C.cross_track_error(path, on_path).max() < 1e-9


# ---------------------------------------------------------------------------
# Actuator lag, and why the derivative term exists
# ---------------------------------------------------------------------------

def test_actuator_lag_delays_the_response():
    from motion import Actuator
    act = Actuator(tau=0.15)
    v, w = act.apply(0.0, 1.0)
    assert 0.0 < w < 1.0            # it starts moving, but does not arrive
    for _ in range(50):
        v, w = act.apply(0.0, 1.0)
    assert w == pytest.approx(1.0, abs=1e-3)


def test_zero_tau_is_instant():
    from motion import Actuator
    act = Actuator(tau=0.0)
    assert act.apply(0.3, 1.0)[1] == pytest.approx(1.0)


def _step_response(control_module, kp, kd=0.0, steps=170, tau=0.15):
    from motion import Actuator, step as motion_step
    pid = control_module.PID(kp=kp, kd=kd)
    act = Actuator(tau)
    pose, target, errors = Pose(0, 0, 0), math.pi / 2, []
    for _ in range(steps):
        _, w = act.apply(0.0, pid(target - pose.theta))
        pose = motion_step(pose, 0.0, w)
        errors.append(target - pose.theta)
    return np.array(errors)


def test_without_lag_proportional_control_cannot_overshoot():
    """The modelling point. A plant with no inertia makes every gain look safe.

    Tune against this and you will conclude more gain is always better, which
    is exactly the wrong lesson and it does not survive contact with a motor.
    """
    e = _step_response(C, kp=6.0, tau=0.0)
    assert e.min() >= -1e-6


def test_with_lag_too_much_gain_overshoots():
    e = _step_response(C, kp=6.0, tau=0.15)
    overshoot = -e.min() / (math.pi / 2)
    assert overshoot > 0.03


def test_the_derivative_term_removes_the_overshoot():
    """The entire argument for kd, in one comparison at equal kp."""
    plain = -_step_response(C, kp=6.0, kd=0.0).min()
    damped = -_step_response(C, kp=6.0, kd=0.4).min()
    assert damped < plain / 2.0

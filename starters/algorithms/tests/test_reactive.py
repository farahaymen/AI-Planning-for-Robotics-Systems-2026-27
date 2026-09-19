"""
Tests for Lab 3, Exercise 3.5 and Lab 4, Exercise 4.8. The reactive behaviours.

    ARC_REACTIVE=reactive_skeleton python3 -m pytest tests/test_reactive.py -x -q

A safety layer is the one piece of code where "it seemed to work when I tried
it" is not good enough, so these are strict. Several of them encode failures
that are easy to write, pass a casual look, and are dangerous.
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

from gridmap import empty_room
from motion import ROBOT, Pose, step
from sensing import LIDAR, beam_angles, in_corridor, raycast, scan_to_points

R = importlib.import_module(os.environ.get("ARC_REACTIVE", "reactive"))

FORWARD = int(np.argmin(np.abs(beam_angles())))     # the beam pointing at +x


def clear_scan(distance=5.0):
    return np.full(LIDAR.n_beams, float(distance))


def scan_with_wall_ahead(distance):
    """Everything far away except a patch straight in front."""
    r = clear_scan(8.0)
    r[FORWARD - 5:FORWARD + 6] = distance
    return r


# ---------------------------------------------------------------------------
# The simulated sensor
# ---------------------------------------------------------------------------

def test_raycast_measures_a_known_wall():
    """A 4 m room, robot 1 m in, facing the far wall. The answer is arithmetic."""
    grid = empty_room(80, 80)                       # 0.05 m cells
    r = raycast(grid, Pose(1.0, 1.0, 0.0), 0.05)
    assert r[FORWARD] == pytest.approx(2.95, abs=0.1)


def test_beams_that_hit_nothing_return_infinity():
    """Not max_range. "Nothing there" and "a wall at 12 m" are different facts,
    and Lab 4's occupancy mapping builds walls out of empty space if you mix
    them up."""
    grid = np.zeros((40, 40), dtype=bool)           # no walls at all
    r = raycast(grid, Pose(1.0, 1.0, 0.0), 0.05)
    assert np.isinf(r).all()


def test_scan_to_points_drops_infinities_and_places_them_correctly():
    r = np.full(LIDAR.n_beams, np.inf)
    r[FORWARD] = 2.0
    pts = scan_to_points(r)
    assert len(pts) == 1
    assert pts[0][0] == pytest.approx(2.0, abs=1e-6)
    assert pts[0][1] == pytest.approx(0.0, abs=1e-6)


def test_the_corridor_depends_on_range_not_on_a_fixed_cone():
    """An obstacle 30 degrees off is in the way at 0.3 m and not at 3 m.

    A fixed angular cone cannot express that, which is why `in_corridor` uses
    each beam's measured range.
    """
    angles = beam_angles()
    i = int(np.argmin(np.abs(angles - math.radians(30))))

    near = clear_scan(8.0); near[i] = 0.30
    far = clear_scan(8.0); far[i] = 3.00
    assert in_corridor(near, half_width=0.25)[i]
    assert not in_corridor(far, half_width=0.25)[i]


# ---------------------------------------------------------------------------
# Time to collision
# ---------------------------------------------------------------------------

def test_a_stationary_robot_has_infinite_time_to_collision():
    assert math.isinf(R.time_to_collision(scan_with_wall_ahead(0.3), v=0.0))


def test_an_empty_corridor_has_infinite_time_to_collision():
    """Infinite means nothing is in the way at all, which is what `inf` returns
    mean. A wall at 8 m is not "clear", it is far: that case is below."""
    assert math.isinf(R.time_to_collision(np.full(LIDAR.n_beams, np.inf), v=0.5))


def test_a_distant_wall_gives_a_large_but_finite_time():
    """8 m at 0.5 m/s is about 15.6 s once the footprint is subtracted. Real,
    and nowhere near any sane braking threshold."""
    ttc = R.time_to_collision(clear_scan(8.0), v=0.5)
    assert math.isfinite(ttc)
    assert ttc > 10.0


def test_time_to_collision_halves_when_speed_doubles():
    """The property that makes this a TIME rather than a distance."""
    r = scan_with_wall_ahead(3.0)
    slow = R.time_to_collision(r, v=0.25)
    fast = R.time_to_collision(r, v=0.50)
    assert fast == pytest.approx(slow / 2.0, rel=0.02)


def test_time_to_collision_measures_to_the_footprint_not_the_centre():
    """The LiDAR sits inside the robot. A return at the footprint radius is
    already touching, so the time remaining is zero, not radius over speed."""
    r = scan_with_wall_ahead(ROBOT.footprint_radius)
    assert R.time_to_collision(r, v=0.4) == pytest.approx(0.0, abs=1e-6)


def test_obstacles_to_the_side_are_ignored():
    """Braking for everything gives a robot that cannot use a doorway."""
    r = np.full(LIDAR.n_beams, np.inf)
    side = int(np.argmin(np.abs(beam_angles() - math.radians(80))))
    r[side] = 0.30
    assert math.isinf(R.time_to_collision(r, v=0.5))


def test_infinite_returns_do_not_break_the_arithmetic():
    r = np.full(LIDAR.n_beams, np.inf)
    r[FORWARD] = 1.0
    assert np.isfinite(R.time_to_collision(r, v=0.5))


# ---------------------------------------------------------------------------
# The safety layer
# ---------------------------------------------------------------------------

def _always_forward(pose=None, path=None, index=0, *, ranges):
    return 0.4, 0.0, index


def test_the_brake_does_not_interfere_in_open_space():
    guarded = R.make_emergency_brake(threshold_s=1.2)(_always_forward)
    v, w, _ = guarded(None, None, 0, ranges=clear_scan(8.0))
    assert v == pytest.approx(0.4)


def test_the_brake_stops_the_robot_before_a_wall():
    grid = empty_room(80, 80)
    guarded = R.make_emergency_brake(threshold_s=1.2)(_always_forward)
    pose = Pose(1.0, 1.0, 0.0)
    for _ in range(400):
        v, w, _ = guarded(pose, None, 0, ranges=raycast(grid, pose, 0.05))
        if v == 0.0:
            gap = 4.0 - pose.x - ROBOT.footprint_radius
            # Braking at 1.2 s from 0.4 m/s means roughly 0.48 m of clearance.
            assert 0.2 < gap < 0.8
            return
        pose = step(pose, v, w)
    pytest.fail("the robot never braked")


def test_the_brake_can_only_ever_slow_the_robot_down():
    """The property that makes it a safety layer rather than a behaviour.

    A layer that can command motion is a layer that can cause an accident.
    """
    def demands_speed(pose=None, path=None, index=0, *, ranges):
        return 0.5, 0.0, index
    guarded = R.make_emergency_brake(threshold_s=1.2)(demands_speed)
    for d in (0.2, 0.5, 1.0, 3.0, 8.0):
        v, _, _ = guarded(None, None, 0, ranges=scan_with_wall_ahead(d))
        assert v <= 0.5 + 1e-9


def test_the_brake_keeps_the_turn_so_the_robot_can_escape():
    """Freezing both would leave the robot pinned against the wall forever."""
    def turning(pose=None, path=None, index=0, *, ranges):
        return 0.4, 0.9, index
    guarded = R.make_emergency_brake(threshold_s=1.2)(turning)
    v, w, _ = guarded(None, None, 0, ranges=scan_with_wall_ahead(0.25))
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(0.9)


def test_a_higher_threshold_brakes_earlier():
    r = scan_with_wall_ahead(1.0)
    cautious = R.make_emergency_brake(threshold_s=3.0)(_always_forward)
    brave = R.make_emergency_brake(threshold_s=0.2)(_always_forward)
    assert cautious(None, None, 0, ranges=r)[0] == pytest.approx(0.0)
    assert brave(None, None, 0, ranges=r)[0] > 0.0


# ---------------------------------------------------------------------------
# Follow the gap
# ---------------------------------------------------------------------------

def test_the_widest_gap_is_the_widest_one():
    r = np.zeros(LIDAR.n_beams)
    r[10:20] = 5.0            # ten beams
    r[100:140] = 5.0          # forty beams
    start, end = R.widest_gap(r, threshold=1.0)
    assert (start, end) == (100, 139)


def test_no_gap_is_reported_when_everything_is_blocked():
    assert R.widest_gap(np.full(LIDAR.n_beams, 0.2), threshold=1.0) == (-1, -1)


def test_infinite_returns_count_as_free_space():
    r = np.zeros(LIDAR.n_beams)
    r[50:80] = np.inf
    start, end = R.widest_gap(r, threshold=1.0)
    assert (start, end) == (50, 79)


def test_the_safety_bubble_is_wider_when_the_obstacle_is_nearer():
    """Trigonometry: the same radius subtends a bigger angle up close."""
    near = clear_scan(8.0); near[FORWARD] = 0.30
    far = clear_scan(8.0); far[FORWARD] = 3.00
    assert (R.safety_bubble(near, 0.35) == 0).sum() > \
           (R.safety_bubble(far, 0.35) == 0).sum()


def test_the_safety_bubble_wraps_around_the_scan():
    """Beam 359 is adjacent to beam 0. Clipping instead of wrapping halves the
    bubble whenever the nearest obstacle is behind the robot."""
    behind = clear_scan(5.0); behind[0] = 0.55
    ahead = clear_scan(5.0); ahead[180] = 0.55
    assert (R.safety_bubble(behind, 0.35) == 0).sum() == \
           (R.safety_bubble(ahead, 0.35) == 0).sum()


def test_the_gap_follower_steers_towards_open_space():
    r = np.full(LIDAR.n_beams, 0.5)
    left = int(np.argmin(np.abs(beam_angles() - math.radians(40))))
    r[left - 15:left + 16] = 6.0
    ctl = R.make_gap_follower()
    _, w, _ = ctl(None, None, 0, ranges=r)
    assert w > 0


def test_the_gap_follower_slows_down_to_turn_hard():
    straight_on = np.full(LIDAR.n_beams, 0.5)
    straight_on[FORWARD - 20:FORWARD + 21] = 6.0
    sideways = np.full(LIDAR.n_beams, 0.5)
    side = int(np.argmin(np.abs(beam_angles() - math.radians(70))))
    sideways[side - 20:side + 21] = 6.0

    ctl = R.make_gap_follower()
    v_straight, _, _ = ctl(None, None, 0, ranges=straight_on)
    v_turn, _, _ = ctl(None, None, 0, ranges=sideways)
    assert v_turn < v_straight


def test_the_gap_follower_does_not_drive_into_things():
    """Sixty seconds in a cluttered room without touching anything.

    Not a proof of safety, and it is not meant to be. It is the floor that any
    working implementation clears, and a broken one does not.
    """
    grid = empty_room(120, 120)
    for (r0, c0, h, w) in [(40, 50, 14, 8), (70, 30, 10, 20),
                           (25, 85, 16, 10), (85, 80, 12, 14)]:
        grid[r0:r0 + h, c0:c0 + w] = True

    ctl = R.make_gap_follower()
    pose = Pose(0.6, 0.6, 0.5)
    for _ in range(1200):
        scan = raycast(grid, pose, 0.05)
        finite = scan[np.isfinite(scan)]
        assert not (finite.size and finite.min() < ROBOT.footprint_radius)
        v, w, _ = ctl(pose, None, 0, ranges=scan)
        pose = step(pose, v, w)


def test_the_gap_follower_commands_are_within_the_robots_limits():
    rng = np.random.default_rng(0)
    ctl = R.make_gap_follower()
    for _ in range(200):
        scan = rng.uniform(0.15, 6.0, LIDAR.n_beams)
        v, w, _ = ctl(None, None, 0, ranges=scan)
        assert -1e-9 <= v <= ROBOT.max_linear_velocity + 1e-9
        assert abs(w) <= ROBOT.max_angular_velocity + 1e-9


def test_a_boxed_in_robot_turns_rather_than_freezing():
    """Stopping dead in a dead end is a robot that needs rescuing by hand."""
    ctl = R.make_gap_follower()
    v, w, _ = ctl(None, None, 0, ranges=np.full(LIDAR.n_beams, 0.3))
    assert v == pytest.approx(0.0)
    assert abs(w) > 0.1

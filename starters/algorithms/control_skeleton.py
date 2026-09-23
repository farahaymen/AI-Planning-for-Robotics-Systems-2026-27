"""
Lab 4, Exercise 4.6. Implement the two controllers.

This is the file you edit. Fill in every block marked TODO.

    ARC_CONTROL=control_skeleton python3 -m pytest tests/test_control.py -x -q
    python3 drive.py --skeleton --lookahead 0.45

A planner gives you a list of points. Without a controller that is a drawing,
not a robot. Project 1 needs both of these working, so this is the file that
stands between you and an autonomous run.

Read `control.py`'s module docstring first. It explains why there are two
controllers here and not one, and the distinction between an error controller
and a geometric controller is the thing worth taking away.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

try:
    from .motion import ROBOT, Pose, wrap_angle
except ImportError:  # Standalone exercise.
    from motion import ROBOT, Pose, wrap_angle


# ---------------------------------------------------------------------------
# Exercise 4.6a. PID
# ---------------------------------------------------------------------------

@dataclass
class PID:
    """A proportional, integral, derivative controller for a scalar error.

        output = kp * e + ki * integral(e) + kd * de/dt

    kp reacts to how wrong you are. ki removes error that refuses to go away on
    its own. kd damps, by reacting to how fast the error is changing.
    """

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    dt: float = ROBOT.control_period
    integral_limit: float = 1.0
    output_limit: float = ROBOT.max_angular_velocity
    wrap: bool = True

    _integral: float = field(default=0.0, init=False)
    _previous: float | None = field(default=None, init=False)

    def reset(self) -> None:
        self._integral = 0.0
        self._previous = None

    def __call__(self, error: float) -> float:
        # TODO 1: if self.wrap is True, fold the error with wrap_angle first.
        #         179 to -179 degrees is a difference of two, not 358. Without
        #         this the robot sometimes spins the long way round.
        #
        # TODO 2: self._integral += error * self.dt, then clamp it to
        #         +/- self.integral_limit. This is anti-windup: a robot held
        #         against a wall otherwise stores up error the whole time and
        #         spins it off when it comes free.
        #
        # TODO 3: derivative is (error - self._previous) / self.dt. Use 0.0 on
        #         the first call, not the error itself, or the first cycle
        #         lurches. Then store the current error in self._previous.
        #
        # TODO 4: combine the three terms, clamp to +/- self.output_limit, and
        #         return a float.
        raise NotImplementedError("TODO: implement PID.__call__")


def make_heading_controller(kp: float = 2.5, ki: float = 0.0, kd: float = 0.15,
                            speed: float = 0.35, slow_on_turn: bool = True):
    """Drive towards a fixed point using PID on the heading error only.

    The simplest closed loop worth writing. Point at the target, drive forwards,
    keep pointing.
    """
    pid = PID(kp=kp, ki=ki, kd=kd)

    def controller(pose: Pose, target: tuple[float, float]):
        # TODO 5: compute the heading you WANT, using atan2 of the offset from
        #         the robot to the target. Take the error against pose.theta,
        #         wrap it, and pass it to pid to get the turn rate.
        #
        # TODO 6: if slow_on_turn, scale the forward speed down as the heading
        #         error grows, with a floor so the robot never stalls. Without
        #         this it drives at full speed while still turning hard, swings
        #         wide, and arrives from the wrong side. Nav2's Regulated Pure
        #         Pursuit does exactly this and calls it a regulation heuristic.
        #
        #         Return (v, w).
        raise NotImplementedError("TODO: implement make_heading_controller")

    controller.pid = pid
    return controller


# ---------------------------------------------------------------------------
# Exercise 4.6b. Pure Pursuit
# ---------------------------------------------------------------------------

def _to_robot_frame(pose: Pose, point: np.ndarray) -> tuple[float, float]:
    """Express a world point in the robot's frame. Forward +x, left +y.

    Complete, and worth reading. Once the target is in the robot's own frame,
    the Pure Pursuit geometry collapses to two lines.
    """
    dx = point[0] - pose.x
    dy = point[1] - pose.y
    c, s = math.cos(-pose.theta), math.sin(-pose.theta)
    return dx * c - dy * s, dx * s + dy * c


def find_lookahead(path: np.ndarray, pose: Pose, lookahead: float,
                   start_index: int = 0) -> tuple[np.ndarray, int]:
    """The first point on the path at least `lookahead` metres ahead."""
    if len(path) == 0:
        raise ValueError("empty path")
    # TODO 7: walk the path from start_index. Return the first point whose
    #         distance from the robot is at least `lookahead`, together with its
    #         index. If none is far enough, return the LAST point and its index,
    #         which is what makes the robot drive in to the goal at the end.
    #
    #         Start from start_index rather than from zero. This is not an
    #         optimisation: it stops the robot latching onto an earlier part of
    #         a path that loops back near itself, which is what makes a naive
    #         implementation drive a figure of eight.
    raise NotImplementedError("TODO: implement find_lookahead")


def make_pure_pursuit(lookahead: float = 0.45, speed: float = 0.35,
                      regulate: bool = True, min_speed: float = 0.08):
    """Steer along the arc that reaches a point ahead on the path.

    With the lookahead point expressed in the robot's frame at (x, y), and L the
    distance to it:

        curvature = 2 * y / L^2
        w         = v * curvature

    That is the entire algorithm. It comes from the circle that passes through
    the robot, is tangent to its current heading, and passes through the
    lookahead point. No error term, no gains, one parameter.
    """

    def controller(pose: Pose, path: np.ndarray, index: int = 0):
        # TODO 8: find the lookahead point and convert it to the robot frame.
        #
        # TODO 9: if the point is BEHIND the robot (x <= 0), the arc formula
        #         would have it creep forwards while barely turning. Turn on the
        #         spot towards the point instead, and return with v = 0.
        #
        # TODO 10: otherwise compute curvature = 2*y / (x*x + y*y) and w = v*curvature.
        #
        # TODO 11: if `regulate`, reduce v as |curvature| grows, with min_speed
        #          as the floor. This is what stops the robot taking a hairpin at
        #          full speed. It is one line and it is the difference between a
        #          demonstration and something you would run near people.
        #
        #          Return (v, w, index).
        raise NotImplementedError("TODO: implement make_pure_pursuit")

    return controller


# ---------------------------------------------------------------------------
# Given to you complete. Do not change these; the tests and figures use them.
# ---------------------------------------------------------------------------

def follow_path(path: np.ndarray, pose: Pose, controller=None,
                goal_tolerance: float = 0.15, max_steps: int = 2000,
                dt: float = ROBOT.control_period,
                pose_noise: float = 0.0, seed: int = 0):
    """Drive a whole path. Returns (trajectory, arrived).

    `pose_noise` models a controller acting on an estimated pose rather than a
    perfect one. Exercise 4.7 asks you to sweep the lookahead distance with and
    without it, and to measure BOTH cross track error and control jitter. The
    two metrics disagree, and which one you optimise is an engineering decision
    rather than something the data settles.
    """
    from motion import step as motion_step

    controller = controller or make_pure_pursuit()
    rng = np.random.default_rng(seed)
    trajectory, index = [pose], 0
    goal = path[-1]

    for _ in range(max_steps):
        if math.hypot(goal[0] - pose.x, goal[1] - pose.y) <= goal_tolerance:
            return trajectory, True
        if pose_noise > 0.0:
            believed = Pose(pose.x + rng.normal(0.0, pose_noise),
                            pose.y + rng.normal(0.0, pose_noise),
                            wrap_angle(pose.theta + rng.normal(0.0, pose_noise)))
        else:
            believed = pose
        v, w, index = controller(believed, path, index)
        pose = motion_step(pose, v, w, dt)
        trajectory.append(pose)

    return trajectory, False


def cross_track_error(path: np.ndarray, trajectory) -> np.ndarray:
    """Distance from each pose to the nearest point ON the path.

    Note "on the path", not "on a path vertex". Measuring to the nearest
    waypoint sounds equivalent and is not: a path sampled every 5 cm reports up
    to 2.5 cm of error for a robot sitting exactly on the line, because the
    nearest vertex is up to half a sample away. That is pure measurement
    artefact, it is the same order as the differences you are trying to detect,
    and it would make every comparison in Exercise 4.7 meaningless.

    So this projects each pose onto every path SEGMENT and takes the smallest
    perpendicular distance, clamped to the segment ends.
    """
    if len(path) == 0 or len(trajectory) == 0:
        return np.zeros(0)
    points = np.array([[p.x, p.y] for p in trajectory])
    if len(path) == 1:
        return np.linalg.norm(points - path[0], axis=1)

    a = path[:-1]                      # segment starts, (S, 2)
    b = path[1:]                       # segment ends
    ab = b - a                         # segment vectors
    ab_len2 = np.einsum("ij,ij->i", ab, ab)
    ab_len2 = np.where(ab_len2 == 0.0, 1e-12, ab_len2)

    ap = points[:, None, :] - a[None, :, :]            # (N, S, 2)
    t = np.einsum("nsi,si->ns", ap, ab) / ab_len2      # projection parameter
    t = np.clip(t, 0.0, 1.0)                           # stay on the segment
    closest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
    return np.linalg.norm(points[:, None, :] - closest, axis=2).min(axis=1)


def control_jitter(trajectory, dt: float = ROBOT.control_period) -> float:
    """Standard deviation of the change in turn rate: how hard the robot works."""
    if len(trajectory) < 3:
        return 0.0
    theta = np.array([p.theta for p in trajectory])
    omega = np.diff(np.unwrap(theta)) / dt
    return float(np.std(np.diff(omega)))

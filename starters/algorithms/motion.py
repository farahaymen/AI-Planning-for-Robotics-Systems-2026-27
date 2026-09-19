"""
The robot, and how it moves. Shared by every controller in this folder.

Nothing here talks to ROS or to Gazebo. That is deliberate: a controller you can
only test by launching a simulator is a controller you will test three times and
then stop testing. These forty lines let you run a thousand control loops in a
second, on any machine, and see the trajectory as a picture.

The constants are the ARC reference robot's, declared in
`docs/00_design_baseline.md` section 5 and mirrored in `arc_rl/nav_core.py`.
Change one here and you have a robot that does not exist, so the tests check
them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RobotSpec:
    """Physical parameters of the ARC reference differential drive robot.

    These are course constants, not suggestions. The planner, the controller,
    the Nav2 costmap footprint and the reinforcement learning environment all
    assume the same numbers, and a mismatch between any two of them produces a
    robot that plans for one body and drives another.
    """

    wheel_radius: float = 0.050          # m
    wheel_separation: float = 0.350      # m
    footprint_radius: float = 0.220      # m, circumscribed
    max_linear_velocity: float = 0.50    # m/s
    min_linear_velocity: float = -0.125  # m/s, limited reverse
    max_angular_velocity: float = 1.80   # rad/s
    control_period: float = 0.05         # s, a 20 Hz command loop


ROBOT = RobotSpec()


@dataclass
class Pose:
    """Where the robot is, in the world frame. Angles in radians."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)


def wrap_angle(a: float) -> float:
    """Fold an angle into (-pi, pi].

    Use this on every angular difference you ever compute. Without it, a robot
    facing 179 degrees asked to face -179 degrees believes it must turn 358
    degrees the long way round. It is the single most common bug in heading
    control and it is invisible until the robot spins on the spot.
    """
    return math.atan2(math.sin(a), math.cos(a))


def step(pose: Pose, v: float, w: float, dt: float = ROBOT.control_period) -> Pose:
    """Advance the unicycle model by one control period.

    A differential drive robot commanded with a forward speed v and a turn rate
    w moves as if it were a unicycle:

        x'     = x     + v cos(theta) dt
        y'     = y     + v sin(theta) dt
        theta' = theta + w dt

    This is the exact model Gazebo's diff_drive_controller integrates, minus
    wheel slip, motor dynamics and the noise of a real floor. A controller that
    cannot drive this cleanly will not drive the real one, so it is a fair place
    to fail fast.

    The commands are clamped to what the robot can actually do. Leaving that out
    lets a controller look excellent in simulation by asking for 6 rad/s, and
    then behave nothing like it on a robot limited to 1.8.
    """
    v = float(np.clip(v, ROBOT.min_linear_velocity, ROBOT.max_linear_velocity))
    w = float(np.clip(w, -ROBOT.max_angular_velocity, ROBOT.max_angular_velocity))
    return Pose(
        x=pose.x + v * math.cos(pose.theta) * dt,
        y=pose.y + v * math.sin(pose.theta) * dt,
        theta=wrap_angle(pose.theta + w * dt),
    )


@dataclass
class Actuator:
    """First order lag between the command and the wheels obeying it.

    `step` applies whatever velocity you ask for instantly. Real motors do not:
    they accelerate towards the commanded speed with a time constant, because
    they have inertia and a finite torque. This models that in one line.

    It matters more than it looks, and here is why. A plant with no lag CANNOT
    overshoot under proportional control. Ask it to turn 90 degrees and it turns
    at whatever rate you command and stops precisely on the mark, so every gain
    looks stable and kd appears to do nothing at all. Tune a controller against
    that model and you will conclude that more gain is always better, which is
    exactly wrong.

    With a realistic tau of 0.15 s the textbook behaviour appears, measured on a
    90 degree step:

        kp 0.8            no overshoot, settles in 3.75 s
        kp 2.5            1.8 % overshoot, 1.15 s
        kp 6.0            6.1 % overshoot, 1.45 s   <- more gain, slower settling
        kp 6.0, kd 0.4    1.8 % overshoot, 1.05 s   <- what kd is for
        kp 12,  kd 0.8    1.6 % overshoot, 1.00 s

    That is the whole argument for the derivative term: it lets you raise the
    gain for speed without paying for it in overshoot. You cannot see it without
    lag, which is why this class exists.
    """

    tau: float = 0.15                    # s, time constant
    v: float = 0.0
    w: float = 0.0

    def reset(self) -> None:
        self.v = 0.0
        self.w = 0.0

    def apply(self, v_command: float, w_command: float,
              dt: float = ROBOT.control_period) -> tuple[float, float]:
        if self.tau <= 0.0:
            self.v, self.w = v_command, w_command
        else:
            alpha = dt / (self.tau + dt)
            self.v += alpha * (v_command - self.v)
            self.w += alpha * (w_command - self.w)
        return self.v, self.w


def wheel_speeds(v: float, w: float, spec: RobotSpec = ROBOT) -> tuple[float, float]:
    """Convert a body twist into left and right wheel angular velocities.

    This is the inverse kinematics from Lab 2, and it is what `ros2_control`
    applies underneath `cmd_vel`. Worth keeping in sight while tuning: a turn
    rate the body can manage may still demand a wheel speed the motor cannot,
    and the symptom is a robot that turns more slowly than commanded rather than
    an error message.
    """
    half = spec.wheel_separation / 2.0
    left = (v - w * half) / spec.wheel_radius
    right = (v + w * half) / spec.wheel_radius
    return left, right


def simulate(controller, pose: Pose, steps: int = 400,
             dt: float = ROBOT.control_period, stop_when=None):
    """Run a controller in closed loop and return the trajectory it produced.

    `controller(pose)` returns `(v, w)`. `stop_when(pose)` ends the run early,
    which is how the tests assert "it arrived" rather than "it did not crash".

    Returns the list of poses visited, starting with the one given.
    """
    trajectory = [pose]
    for _ in range(steps):
        if stop_when is not None and stop_when(pose):
            break
        v, w = controller(pose)
        pose = step(pose, v, w, dt)
        trajectory.append(pose)
    return trajectory


def path_from_cells(cells, resolution: float = 0.05,
                    origin: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """Turn a planner's list of grid cells into metric waypoints.

    Your planner works in (row, col). Your controller works in metres. This is
    the seam between them, and getting it wrong is the classic transposition
    bug: the path looks plausible, the robot drives it mirrored, and everything
    in between looks correct.

    Row is the y axis and column is the x axis. Say that out loud once.
    """
    cells = np.asarray(cells, dtype=float)
    if cells.size == 0:
        return np.zeros((0, 2), dtype=float)
    xs = origin[0] + cells[:, 1] * resolution
    ys = origin[1] + cells[:, 0] * resolution
    return np.column_stack([xs, ys])

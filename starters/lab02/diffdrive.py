"""
Lab 2: differential drive kinematics, worked in code.

These functions are the entire mathematical content of the lab. They are short
enough to read in one sitting and they are the reason a wrong wheel radius shows
up as a robot that ends up in the wrong place rather than as an error message.

Sign convention: x forward, y left, theta counter-clockwise positive, which is
the ROS REP 103 convention and the one the URDF uses.
"""

from __future__ import annotations

import math

WHEEL_RADIUS = 0.050      # metres, course constant
WHEEL_SEPARATION = 0.350  # metres, course constant


def forward_kinematics(omega_left: float, omega_right: float,
                       radius: float = WHEEL_RADIUS,
                       separation: float = WHEEL_SEPARATION) -> tuple[float, float]:
    """Wheel angular velocities (rad/s) to body twist (v m/s, omega rad/s)."""
    v = radius * (omega_right + omega_left) / 2.0
    omega = radius * (omega_right - omega_left) / separation
    return v, omega


def inverse_kinematics(v: float, omega: float,
                       radius: float = WHEEL_RADIUS,
                       separation: float = WHEEL_SEPARATION) -> tuple[float, float]:
    """Body twist to wheel angular velocities. This is what the controller does."""
    omega_left = (v - omega * separation / 2.0) / radius
    omega_right = (v + omega * separation / 2.0) / radius
    return omega_left, omega_right


def integrate_pose(x: float, y: float, theta: float,
                   v: float, omega: float, dt: float) -> tuple[float, float, float]:
    """Advance the pose by one control period using exact arc integration.

    The straight-line approximation x += v*cos(theta)*dt is what most tutorials
    show and it accumulates error on every turn. Using the exact arc costs three
    extra lines and removes an error source you would otherwise spend Lab 5
    blaming on the LiDAR.
    """
    if abs(omega) < 1e-9:
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
    else:
        radius_of_curvature = v / omega
        theta_next = theta + omega * dt
        x += radius_of_curvature * (math.sin(theta_next) - math.sin(theta))
        y -= radius_of_curvature * (math.cos(theta_next) - math.cos(theta))
        theta = theta_next
    return x, y, math.atan2(math.sin(theta), math.cos(theta))


def odometry_scale_error(assumed_radius: float, true_radius: float) -> float:
    """Ratio of reported distance to actual distance for a wheel radius error.

    Lab 2's optional calibration investigation predicts this number. If the
    controller believes the wheels are larger than they are, it reports having
    travelled further than it did, and every downstream consumer inherits the
    error: the costmap, AMCL, and eventually your Nav2 goal tolerance.
    """
    return assumed_radius / true_radius


def wheel_speed_limits(max_v: float = 0.50, max_omega: float = 1.80,
                       radius: float = WHEEL_RADIUS,
                       separation: float = WHEEL_SEPARATION) -> float:
    """Peak wheel speed the robot's velocity limits actually demand, in rad/s.

    Worth computing once. If this exceeds what the command interface allows in
    the ros2_control block, the robot silently saturates and turns more slowly
    than commanded, which looks like a controller tuning problem and is not.
    """
    _, omega_right = inverse_kinematics(max_v, max_omega, radius, separation)
    return abs(omega_right)


if __name__ == "__main__":
    v, w = forward_kinematics(10.0, 12.0)
    print(f"wheels 10.0 / 12.0 rad/s -> v={v:.4f} m/s, omega={w:.4f} rad/s")
    print(f"peak wheel speed demanded by the velocity limits: "
          f"{wheel_speed_limits():.2f} rad/s")

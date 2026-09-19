"""
The reactive figures, generated from the behaviours.

    python3 figures_reactive.py out/

    fig_braking.png      why the safety rule is a time and not a distance
    fig_gap_follow.png   sixty seconds of a robot navigating with no map at all
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from gridmap import empty_room
from motion import ROBOT, Pose, step
from sensing import LIDAR, beam_angles, raycast, scan_to_points

C_WALL = "#33373d"
C_FREE = "#ffffff"
C_GOOD = "#2a9d8f"
C_MID = "#e9a13b"
C_BAD = "#d1495b"
C_SCAN = "#cfe0f3"


def cluttered_room():
    grid = empty_room(120, 120)
    for (r, c, h, w) in [(40, 50, 14, 8), (70, 30, 10, 20),
                         (25, 85, 16, 10), (85, 80, 12, 14)]:
        grid[r:r + h, c:c + w] = True
    return grid


def fig_braking(reactive, out: Path):
    """Stopping distance against speed, for a time rule and a distance rule."""
    grid = empty_room(80, 80)                       # a 4 m room
    wall_x = 4.0 - 0.05

    def run(speed, guard):
        pose = Pose(0.5, 1.0, 0.0)
        for _ in range(900):
            scan = raycast(grid, pose, 0.05)
            v, w, _ = guard(pose, None, 0, ranges=scan)
            if v == 0.0:
                return wall_x - pose.x - ROBOT.footprint_radius
            pose = step(pose, v, w)
        return np.nan

    speeds = [0.10, 0.20, 0.30, 0.40, 0.50]
    time_rule, distance_rule = [], []

    for s in speeds:
        def constant(pose=None, path=None, index=0, *, ranges, _s=s):
            return _s, 0.0, index

        time_rule.append(run(s, reactive.make_emergency_brake(1.2)(constant)))

        # The naive alternative: brake at a fixed distance, whatever the speed.
        def fixed(pose=None, path=None, index=0, *, ranges, _s=s):
            angles = beam_angles()
            ahead = np.isfinite(ranges) & (np.abs(angles) < math.radians(10))
            if ahead.any() and np.min(ranges[ahead]) < 0.40:
                return 0.0, 0.0, index
            return _s, 0.0, index
        distance_rule.append(run(s, fixed))

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11.0, 4.2),
                                  gridspec_kw={"width_ratios": [1.3, 1.0]})

    ax.plot(speeds, time_rule, "o-", color=C_GOOD, linewidth=2,
            label="time rule: brake below 1.2 s to impact")
    ax.plot(speeds, distance_rule, "s--", color=C_BAD, linewidth=2,
            label="distance rule: brake below 0.40 m")
    ax.axhline(0.0, color="#333", linewidth=1.0)
    ax.set_xlabel("speed (m/s)", fontsize=9)
    ax.set_ylabel("clearance when stopped (m)", fontsize=9)
    ax.set_title("How much room is left when the robot stops", fontsize=10)
    ax.legend(fontsize=8, frameon=False)
    ax.grid(alpha=0.25)
    ax.tick_params(labelsize=8)

    ax2.axis("off")
    lines = [f"{'speed':>7}{'time rule':>12}{'distance rule':>15}", "-" * 34]
    for s, t, d in zip(speeds, time_rule, distance_rule):
        lines.append(f"{s:7.2f}{t:12.2f}{d:15.2f}")
    lines += ["",
              "The distance rule leaves the SAME",
              "clearance at every speed, because it",
              "is not looking at speed at all.",
              "",
              "That is too cautious to creep through",
              "a doorway and too late at full speed:",
              "the robot needs time to stop, and",
              "time is what a distance does not give.",
              "",
              "The time rule scales automatically.",
              "It is also what Nav2's Collision",
              "Monitor does, projecting the footprint",
              "forward along the commanded velocity."]
    ax2.text(0.0, 0.98, "\n".join(lines), family="monospace", fontsize=8.5,
             va="top", ha="left", transform=ax2.transAxes)

    fig.suptitle("A safety rule has to be a TIME, not a distance.", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    p = out / "fig_braking.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    return p


def fig_gap_follow(reactive, out: Path, steps: int = 1200):
    """A robot navigating a cluttered room with no map and no memory."""
    grid = cluttered_room()
    controller = reactive.make_gap_follower()
    pose = Pose(0.6, 0.6, 0.5)
    xs, ys, nearest = [], [], []

    for _ in range(steps):
        scan = raycast(grid, pose, 0.05)
        finite = scan[np.isfinite(scan)]
        nearest.append(float(finite.min()) if finite.size else np.inf)
        v, w, _ = controller(pose, None, 0, ranges=scan)
        pose = step(pose, v, w)
        xs.append(pose.x); ys.append(pose.y)

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.6),
                                  gridspec_kw={"width_ratios": [1.0, 1.0]})

    extent = (0, grid.shape[1] * 0.05, 0, grid.shape[0] * 0.05)
    ax.imshow(grid, cmap=ListedColormap([C_FREE, C_WALL]), origin="lower",
              extent=extent, interpolation="nearest")
    ax.plot(xs, ys, color=C_BAD, linewidth=1.4)
    ax.plot(xs[0], ys[0], "o", color=C_GOOD, markersize=9,
            markeredgecolor="white")
    ax.set_title(f"{steps * ROBOT.control_period:.0f} seconds, "
                 f"no map, no plan, no memory", fontsize=10)
    ax.set_xlabel("x (m)", fontsize=9); ax.set_ylabel("y (m)", fontsize=9)
    ax.set_aspect("equal"); ax.tick_params(labelsize=8)

    t = np.arange(len(nearest)) * ROBOT.control_period
    ax2.plot(t, nearest, color=C_MID, linewidth=1.2)
    ax2.axhline(ROBOT.footprint_radius, color=C_BAD, linestyle="--",
                linewidth=1.4, label=f"footprint, {ROBOT.footprint_radius:.2f} m")
    ax2.set_xlabel("time (s)", fontsize=9)
    ax2.set_ylabel("nearest obstacle (m)", fontsize=9)
    ax2.set_title(f"closest approach {min(nearest):.2f} m, no contact",
                  fontsize=10)
    ax2.legend(fontsize=8, frameon=False)
    ax2.grid(alpha=0.25); ax2.tick_params(labelsize=8)

    fig.suptitle("Follow the gap: steer at the middle of the widest opening.\n"
                 "Competent, and with no memory it will re-enter the same dead "
                 "end forever. That is the argument for a map.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    p = out / "fig_gap_follow.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    return p


def write_all(reactive, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    return [fig_braking(reactive, out), fig_gap_follow(reactive, out)]


if __name__ == "__main__":
    import reactive
    target = sys.argv[1] if len(sys.argv) > 1 else "figures"
    for p in write_all(reactive, target):
        print(p)

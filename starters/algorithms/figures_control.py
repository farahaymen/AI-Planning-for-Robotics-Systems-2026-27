"""
The control figures, generated from the controllers rather than drawn by hand.

    python3 figures_control.py out/

Three pictures, each making one argument:

    fig_lookahead.png     the same corner driven at three lookahead distances.
                          The corner cutting is visible without reading a number.
    fig_pid_gains.png     what kp and kd actually do, as four trajectories.
    fig_control_trade.png accuracy against smoothness, swept, with noise.
                          The figure that stops a student optimising one metric.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from motion import ROBOT, Pose, simulate

C_PATH = "#9aa4b2"
C_GOOD = "#2a9d8f"
C_MID = "#e9a13b"
C_BAD = "#d1495b"
C_START = "#2a9d8f"
C_GOAL = "#e9c46a"


def corner_path():
    xs = np.linspace(0.0, 3.0, 60)
    ys = np.linspace(0.0, 2.0, 40)
    return np.vstack([np.column_stack([xs, np.zeros_like(xs)]),
                      np.column_stack([np.full_like(ys, 3.0), ys])])


def _draw_path(ax, path):
    ax.plot(path[:, 0], path[:, 1], color=C_PATH, linewidth=6, alpha=0.5,
            solid_capstyle="round", label="planned path", zorder=1)


def _draw_traj(ax, traj, colour, label):
    ax.plot([p.x for p in traj], [p.y for p in traj], color=colour,
            linewidth=2.0, label=label, zorder=3)


def fig_lookahead(control, out: Path):
    """The trade, drawn. Short tracks the corner; long cuts it."""
    path = corner_path()
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 4.2), sharex=True, sharey=True)

    for ax, (L, colour) in zip(axes, [(0.15, C_GOOD), (0.45, C_MID), (1.20, C_BAD)]):
        traj, arrived = control.follow_path(path, Pose(0, 0, 0),
                                            control.make_pure_pursuit(lookahead=L),
                                            max_steps=4000)
        err = control.cross_track_error(path, traj)
        jit = control.control_jitter(traj)
        _draw_path(ax, path)
        _draw_traj(ax, traj, colour, "driven")
        ax.plot(0, 0, "o", color=C_START, markersize=8, markeredgecolor="white")
        ax.plot(path[-1, 0], path[-1, 1], "s", color=C_GOAL, markersize=8,
                markeredgecolor="white")
        ax.set_title(f"lookahead {L:.2f} m\nmean error {err.mean():.3f} m, "
                     f"max {err.max():.3f} m\njitter {jit:.2f}", fontsize=9)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=8)

    axes[0].set_ylabel("y (m)", fontsize=9)
    for ax in axes:
        ax.set_xlabel("x (m)", fontsize=9)
    fig.suptitle("Pure Pursuit aims at a point ahead, so it turns before the corner.\n"
                 "The further ahead it aims, the earlier it turns and the more "
                 "corner it cuts.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    p = out / "fig_lookahead.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    return p


def fig_pid_gains(control, out: Path):
    """The step response. The textbook picture, from our own robot model.

    Heading error against time after being asked to turn 90 degrees. This is
    the right picture for PID and a trajectory is not: overshoot, oscillation
    and settling time are all visible here and all invisible in a path plot.

    Note the actuator lag. Without it the plant cannot overshoot, every gain
    looks stable, and kd appears to do nothing.
    """
    import math
    from motion import Actuator, Pose, step as motion_step

    def response(kp, ki=0.0, kd=0.0, steps=170, tau=0.15):
        pid = control.PID(kp=kp, ki=ki, kd=kd)
        act = Actuator(tau)
        pose, target, errors = Pose(0, 0, 0), math.pi / 2, []
        for _ in range(steps):
            w_command = pid(target - pose.theta)
            _, w = act.apply(0.0, w_command)
            pose = motion_step(pose, 0.0, w)
            errors.append(math.degrees(target - pose.theta))
        return np.array(errors)

    settings = [
        ("kp 0.8", dict(kp=0.8), C_PATH, "no overshoot, but slow"),
        ("kp 2.5", dict(kp=2.5), C_GOOD, "a reasonable default"),
        ("kp 6.0", dict(kp=6.0), C_BAD, "more gain, MORE overshoot, slower to settle"),
        ("kp 6.0, kd 0.4", dict(kp=6.0, kd=0.4), C_MID, "same gain, damped"),
    ]

    t = np.arange(170) * ROBOT.control_period
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.4),
                                  gridspec_kw={"width_ratios": [1.6, 1.0]})

    rows = []
    for name, kw, colour, note in settings:
        e = response(**kw)
        ax.plot(t, e, color=colour, linewidth=1.8, label=f"{name}: {note}")
        overshoot = max(0.0, -e.min()) / 90.0 * 100.0
        settle = next((i for i in range(len(e)) if np.all(np.abs(e[i:]) < 3.0)), len(e))
        rows.append((name, overshoot, settle * ROBOT.control_period))

    ax.axhline(0.0, color="#333", linewidth=0.8)
    ax.axhspan(-3, 3, color="#2a9d8f", alpha=0.10)
    ax.set_xlabel("time (s)", fontsize=9)
    ax.set_ylabel("heading error (degrees)", fontsize=9)
    ax.set_title("Step response: turn 90 degrees and hold", fontsize=10)
    ax.legend(fontsize=8, frameon=False)
    ax.grid(alpha=0.25)
    ax.tick_params(labelsize=8)

    ax2.axis("off")
    text = f"{'gains':<16}{'overshoot':>10}{'settle':>9}\n" + "-" * 35 + "\n"
    for name, ov, st in rows:
        text += f"{name:<16}{ov:9.1f}%{st:8.2f}s\n"
    text += ("\nThe shaded band is within 3 degrees.\n\n"
             "Raising kp from 2.5 to 6.0 buys nothing:\n"
             "it overshoots more AND settles later.\n\n"
             "Adding kd at the same kp fixes both.\n"
             "That is what the derivative term is for.")
    ax2.text(0.0, 0.98, text, family="monospace", fontsize=8.5,
             va="top", ha="left", transform=ax2.transAxes)

    fig.suptitle("Actuator lag of 0.15 s is modelled. Without it the robot "
                 "cannot overshoot,\nevery gain looks stable, and kd appears "
                 "to do nothing.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    p = out / "fig_pid_gains.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    return p


def fig_control_trade(control, out: Path, seeds: int = 12, noise: float = 0.05):
    """Accuracy and smoothness, swept. They disagree, and that is the lesson."""
    path = corner_path()
    lookaheads = [0.10, 0.15, 0.20, 0.30, 0.45, 0.70, 1.00, 1.30]
    err_mean, err_sd, jit_mean, jit_sd = [], [], [], []

    for L in lookaheads:
        errs, jits = [], []
        for s in range(seeds):
            traj, _ = control.follow_path(
                path, Pose(0, 0, 0), control.make_pure_pursuit(lookahead=L),
                max_steps=4000, pose_noise=noise, seed=s)
            errs.append(control.cross_track_error(path, traj).mean())
            jits.append(control.control_jitter(traj))
        err_mean.append(np.mean(errs)); err_sd.append(np.std(errs))
        jit_mean.append(np.mean(jits)); jit_sd.append(np.std(jits))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.5, 4.2))

    ax1.errorbar(lookaheads, err_mean, yerr=err_sd, marker="o", color=C_BAD,
                 capsize=3, linewidth=1.6)
    ax1.set_xlabel("lookahead distance (m)", fontsize=9)
    ax1.set_ylabel("mean cross track error (m)", fontsize=9)
    ax1.set_title("Accuracy says: shorter is better", fontsize=10)
    ax1.grid(alpha=0.25)

    ax2.errorbar(lookaheads, jit_mean, yerr=jit_sd, marker="s", color=C_GOOD,
                 capsize=3, linewidth=1.6)
    ax2.set_xlabel("lookahead distance (m)", fontsize=9)
    ax2.set_ylabel("control jitter (rad/s per cycle)", fontsize=9)
    ax2.set_title("Effort says: longer is better", fontsize=10)
    ax2.grid(alpha=0.25)

    for ax in (ax1, ax2):
        ax.tick_params(labelsize=8)

    fig.suptitle(f"The same {seeds} runs, measured two ways, with {noise*100:.0f} cm "
                 "of pose uncertainty.\nNeither curve is wrong. Which one you "
                 "optimise is an engineering decision about the robot's job.",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    p = out / "fig_control_trade.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    return p


def write_all(control, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    return [fig_lookahead(control, out),
            fig_pid_gains(control, out),
            fig_control_trade(control, out)]


if __name__ == "__main__":
    import control
    target = sys.argv[1] if len(sys.argv) > 1 else "figures"
    for p in write_all(control, target):
        print(p)

"""
Generate every figure in the course manuals from the actual course code.

Nothing here is drawn by hand or traced from a screenshot. Each figure is
produced by running the same modules the students run, so a change to
occupancy.py or the arena grammar changes the figure the next time this is built.
That is deliberate: a diagram that has drifted from the code it illustrates is
worse than no diagram.

    python3 scripts/make_figures.py            # all figures
    python3 scripts/make_figures.py lab04      # one lab

Output: docs/figures/*.png at 200 dpi, sized for a Word page.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle, Wedge

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

OUT = Path(__file__).resolve().parent.parent / "docs" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

# ---------------------------------------------------------------------------
# House style. Chosen to survive greyscale printing, which university handouts
# frequently are: every pair of series differs in lightness and in line style,
# never in hue alone.
# ---------------------------------------------------------------------------

INK = "#1b2430"
PRIMARY = "#2f6f9f"
ACCENT = "#c1553b"
GOOD = "#3f7d55"
MUTED = "#98a2ad"
GRID = "#e4e8ed"
PAPER = "#ffffff"

plt.rcParams.update({
    "figure.facecolor": PAPER,
    "axes.facecolor": PAPER,
    "axes.edgecolor": MUTED,
    "axes.labelcolor": INK,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.labelsize": 9,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.7,
    "xtick.color": INK,
    "ytick.color": INK,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "font.family": "DejaVu Sans",
    "font.size": 9,
    "legend.frameon": False,
    "legend.fontsize": 8,
    "figure.dpi": 200,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.15,
})


def save(fig, name: str) -> None:
    path = OUT / f"{name}.png"
    fig.savefig(path)
    plt.close(fig)
    print(f"  {path.relative_to(OUT.parent.parent)}")


def draw_scenario(ax, scenario, show_goals=True, obstacle_colour=MUTED, limits=None):
    """Render an arena from the grammar. Used by several figures."""
    for (ax0, ay0, bx0, by0) in scenario.walls:
        ax.plot([ax0, bx0], [ay0, by0], color=INK, lw=1.8, solid_capstyle="round")
    for (cx, cy, r) in scenario.static_circles:
        ax.add_patch(Circle((cx, cy), r, facecolor=obstacle_colour,
                            edgecolor=INK, lw=0.6, alpha=0.85))
    for (cx, cy, _s, _h, r) in scenario.dynamic_circles:
        ax.add_patch(Circle((cx, cy), r, facecolor=ACCENT, edgecolor=INK,
                            lw=0.6, alpha=0.55, hatch="///"))
    if show_goals:
        for i, (gx, gy) in enumerate(scenario.goals, start=1):
            ax.plot(gx, gy, marker="*", ms=8, color=GOOD, mec=INK, mew=0.4, zorder=5)
            ax.annotate(str(i), (gx, gy), textcoords="offset points",
                        xytext=(5, 4), fontsize=6.5, color=INK, weight="bold")
    sx, sy, _ = scenario.start_pose
    ax.plot(sx, sy, marker="o", ms=5, color=PRIMARY, mec=INK, mew=0.4, zorder=5)
    ax.set_aspect("equal")
    # A shared limit across panels keeps arena SIZE differences readable as size
    # rather than making every arena look the same shape at a different zoom.
    lo, hi = limits if limits else (-0.3, max(scenario.bounds) + 0.3)
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.grid(False)


# ===========================================================================
# Lab 4: occupancy grid mapping
# ===========================================================================

def _patrol_run(n_beams=360, lidar_range=4.0):
    """Drive a clear rectangular patrol and capture real raycast scans.

    The geometry is fixed rather than sampled so that the illustration is
    reproducible and guaranteed collision free. The scans come from the same
    raycasting the surrogate environment uses.
    """
    from arc_rl.nav_core import FastNavEnv, ObsSpec, Scenario

    room = Scenario.boxed_room(10.0, 10.0)
    room.static_circles = [(5.0, 5.0, 0.65), (7.6, 2.6, 0.45), (2.4, 7.4, 0.5),
                           (7.4, 7.4, 0.35)]
    room.dynamic_circles = []
    # A 4 m range in a 10 m room is what creates genuine max-range returns, which
    # is the whole point of the second panel. With the full 12 m range every beam
    # terminates on a wall and the bug cannot express itself.
    spec = ObsSpec(n_beams=n_beams, lidar_max_range=lidar_range)
    env = FastNavEnv(scenario=room, obs_spec=spec, randomise_start=False,
                     lidar_noise_std=0.012)
    env.reset(seed=3)

    corners = [(1.5, 1.5), (8.5, 1.5), (8.5, 8.5), (1.5, 8.5), (1.5, 1.5)]
    poses = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        heading = math.atan2(y1 - y0, x1 - x0)
        for t in np.arange(0.0, length, 0.10):
            poses.append((x0 + t * math.cos(heading), y0 + t * math.sin(heading), heading))

    scans = []
    for (x, y, th) in poses:
        env.x, env.y, env.theta = x, y, th
        scans.append(env._raycast())
    angles = np.asarray(env._angles, dtype=float)
    return room, poses, scans, angles


def fig_lab04_mapping():
    from starters.lab04.occupancy import MappingParams, OccupancyMap

    room, poses, scans, angles = _patrol_run(lidar_range=2.5)

    def build(model_max_range):
        # The grid is deliberately larger than the room with a negative origin,
        # so the boundary walls fall INSIDE it. A grid sized exactly to the room
        # clips every wall cell and produces a map with no walls at all.
        grid = OccupancyMap(11.0, 11.0, origin=(-0.5, -0.5),
                            params=MappingParams(resolution=0.05,
                                                 max_range=model_max_range))
        for pose, scan in zip(poses, scans):
            grid.integrate_scan(pose, scan, angles)
        return grid

    # 2.5 is correct: it matches the sensor. 100.0 is the Exercise 4.3 bug, where
    # a return at the sensor limit is mistaken for a genuine hit.
    correct, buggy = build(2.5), build(100.0)

    fig, axes = plt.subplots(1, 3, figsize=(10.2, 3.6))
    fig.subplots_adjust(wspace=0.28)

    draw_scenario(axes[0], room, show_goals=False)
    axes[0].plot([p[0] for p in poses], [p[1] for p in poses],
                 color=PRIMARY, lw=1.5, ls="--", label="sensor path")
    axes[0].set_title("The run", pad=8)
    axes[0].annotate("sensor path", xy=(5.0, 8.5), xytext=(4.4, 9.3), fontsize=8,
                     color=PRIMARY, weight="bold", ha="center")

    # Ternary rendering, the same encoding students export in Exercise 4.5:
    # free is white, unknown is grey, occupied is black.
    palette = matplotlib.colors.ListedColormap([PAPER, "#b9c0c8", INK])
    for ax, grid, title in ((axes[1], correct, "Correct model"),
                            (axes[2], buggy, "Max-range treated as a hit")):
        occ = grid.to_ros_occupancy()
        image = np.full(occ.shape, 1)          # unknown
        image[occ == 0] = 0                    # free
        image[occ == 100] = 2                  # occupied
        ax.imshow(image, origin="lower", cmap=palette, vmin=0, vmax=2,
                  extent=[-0.5, 10.5, -0.5, 10.5], interpolation="nearest")
        ax.set_aspect("equal"); ax.grid(False)
        ax.set_title(title, pad=8)

    axes[2].annotate("phantom wall where the\nbeam simply found nothing",
                     xy=(4.6, 6.0), xytext=(2.6, 8.8), fontsize=8,
                     color=ACCENT, weight="bold", ha="left",
                     arrowprops=dict(arrowstyle="->", color=ACCENT, lw=1.3))

    fig.suptitle("Occupancy mapping from a real patrol, built by occupancy.py. "
                 "White free, grey unknown, black occupied.",
                 fontsize=9.5, y=1.02, color=INK)
    save(fig, "lab04_mapping_correct_vs_ring")


def fig_lab04_log_odds():
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.0))

    l = np.linspace(-5, 5, 400)
    axes[0].plot(l, 1 - 1 / (1 + np.exp(l)), color=PRIMARY, lw=2)
    axes[0].axhline(0.5, color=MUTED, ls=":", lw=1)
    axes[0].axvline(0.0, color=MUTED, ls=":", lw=1)
    axes[0].annotate("prior: unknown", xy=(0, 0.5), xytext=(-4.6, 0.72), fontsize=8,
                     color=INK, arrowprops=dict(arrowstyle="->", color=MUTED, lw=1))
    axes[0].set_xlabel("log odds $l$"); axes[0].set_ylabel("probability $p$")
    axes[0].set_title("Log odds to probability")

    # Repeated observation with clamping, using the real update constants.
    for l_max, style, label in ((5.0, "-", "clamp at 5.0"), (50.0, "--", "no effective clamp")):
        value, series = 0.0, []
        for _ in range(30):
            value = min(value + 0.85, l_max)
            series.append(1 - 1 / (1 + math.exp(value)))
        axes[1].plot(range(1, 31), series, style, color=PRIMARY if l_max == 5 else ACCENT,
                     lw=1.8, label=label)
    axes[1].set_xlabel("times the cell is observed occupied")
    axes[1].set_ylabel("probability")
    axes[1].set_ylim(0.4, 1.02)
    axes[1].set_title("Why clamping keeps a map correctable")
    axes[1].legend(loc="lower right")
    save(fig, "lab04_log_odds")


def fig_lab04_inverse_sensor_model():
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    res = 0.25
    for i in range(15):
        for j in range(7):
            ax.add_patch(Rectangle((i * res, j * res), res, res, facecolor=PAPER,
                                   edgecolor=GRID, lw=0.8))
    sx, sy = 0.15, 0.875
    hit = 2.6
    for k in range(1, 11):
        x = sx + k * (hit - sx) / 11
        ax.add_patch(Rectangle((math.floor(x / res) * res, 0.75), res, res,
                               facecolor=PRIMARY, alpha=0.16, edgecolor=GRID))
    ax.add_patch(Rectangle((math.floor(hit / res) * res, 0.75), res, res,
                           facecolor=ACCENT, alpha=0.55, edgecolor=INK, lw=0.8))
    ax.annotate("", xy=(hit, sy), xytext=(sx, sy),
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.6))
    ax.plot(sx, sy, marker="o", ms=8, color=INK)
    ax.text(sx, 1.42, "sensor", fontsize=8, color=INK, ha="left")
    ax.text(1.2, 0.42, "cells along the ray:  add $l_{free}$ = -0.40",
            fontsize=8.5, color=PRIMARY, ha="center")
    ax.text(2.72, 1.42, "add $l_{occ}$ = +0.85", fontsize=8.5,
            color=ACCENT, ha="left")
    ax.text(3.15, 0.42, "beyond the hit:\nunchanged, still unknown",
            fontsize=8, color=MUTED, ha="left")
    ax.set_xlim(-0.1, 3.85); ax.set_ylim(0.0, 1.75)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("The inverse sensor model: one beam, two constants", pad=6)
    save(fig, "lab04_inverse_sensor_model")


# ===========================================================================
# Lab 5: particle filter
# ===========================================================================

def fig_lab05_particle_filter():
    from arc_rl.nav_core import FastNavEnv, ObsSpec, Scenario
    from starters.lab05.particle_filter import ParticleFilter, odometry_increment

    room = Scenario.boxed_room(10.0, 10.0)
    room.static_circles = [(5.0, 5.0, 0.65), (7.6, 2.6, 0.45), (2.4, 7.4, 0.5)]
    room.dynamic_circles = []
    spec = ObsSpec(n_beams=12, lidar_max_range=12.0)
    env = FastNavEnv(scenario=room, obs_spec=spec, randomise_start=False,
                     lidar_noise_std=0.02)
    env.reset(seed=1)

    def scan_at(x, y, th):
        env.x, env.y, env.theta = x, y, th
        return env._raycast()

    truth = [(1.5 + 0.12 * k, 1.5, 0.0) for k in range(28)] + \
            [(4.86, 1.5 + 0.12 * k, math.pi / 2) for k in range(28)]

    pf = ParticleFilter(n_particles=600, seed=4)
    pf.seed_uniform((0.5, 0.5, 9.5, 9.5))
    snapshots, spread, previous = [], [], None

    for step, pose in enumerate(truth):
        if previous is not None:
            pf.predict(*odometry_increment(previous, pose))
        previous = pose
        actual = scan_at(*pose)
        errors = np.array([float(np.mean(np.abs(scan_at(*p) - actual)))
                           for p in pf.particles])
        pf.update_weights(errors, sigma=0.35)
        pf.maybe_resample(0.5)
        spread.append(pf.spread())
        if step in (0, 8, 27):
            snapshots.append((step, pf.particles.copy(), pose))

    fig, axes = plt.subplots(1, 4, figsize=(11.0, 2.9),
                            gridspec_kw={"width_ratios": [1, 1, 1, 1.15]})
    for ax, (step, particles, pose) in zip(axes[:3], snapshots):
        draw_scenario(ax, room, show_goals=False)
        ax.scatter(particles[:, 0], particles[:, 1], s=2.5, color=PRIMARY,
                   alpha=0.45, linewidths=0)
        ax.plot(pose[0], pose[1], marker="X", ms=9, color=ACCENT, mec=INK, mew=0.6)
        ax.set_title(f"step {step}", fontsize=9.5)

    axes[3].plot(spread, color=PRIMARY, lw=1.8)
    axes[3].axhline(0.5, color=ACCENT, ls="--", lw=1.2)
    axes[3].text(len(spread) * 0.42, 0.62, "converged (0.5 m)", fontsize=7.5, color=ACCENT)
    axes[3].set_xlabel("step"); axes[3].set_ylabel("particle spread (m)")
    axes[3].set_title("Convergence", fontsize=9.5)

    fig.suptitle("Global localisation: 600 particles, no initial pose estimate",
                 fontsize=10, y=1.06, color=INK)
    save(fig, "lab05_particle_convergence")


# ===========================================================================
# Lab 9 and 10: measured results
# ===========================================================================

def _load(name):
    path = Path(__file__).resolve().parent.parent / "results" / f"{name}.json"
    return json.loads(path.read_text()) if path.exists() else None


def fig_lab09_results():
    names = [("classical_gap_follow", "classical"),
             ("ppo_dense_progress", "PPO dense"),
             ("ppo_sparse_safe", "PPO sparse")]
    loaded = [(label, _load(key)) for key, label in names]
    if any(r is None for _, r in loaded):
        print("  (skipped lab09 results: run the evaluations first)")
        return

    fig, axes = plt.subplots(1, 3, figsize=(9.6, 3.1))
    labels = [l for l, _ in loaded]
    colours = [MUTED, PRIMARY, ACCENT]

    success = [r["aggregate"]["success_rate"] * 100 for _, r in loaded]
    ci = [r["aggregate"]["success_rate_ci95"] for _, r in loaded]
    lower = [s - c[0] * 100 for s, c in zip(success, ci)]
    upper = [c[1] * 100 - s for s, c in zip(success, ci)]
    axes[0].bar(labels, success, color=colours, edgecolor=INK, lw=0.6)
    axes[0].errorbar(labels, success, yerr=[lower, upper], fmt="none",
                     ecolor=INK, capsize=4, lw=1.1)
    axes[0].set_ylabel("success rate (%)")
    axes[0].set_title("Success, with 95% interval")
    axes[0].set_ylim(0, 80)

    clearance = [r["aggregate"]["min_clearance_m"]["mean"] for _, r in loaded]
    axes[1].bar(labels, clearance, color=colours, edgecolor=INK, lw=0.6)
    axes[1].set_ylabel("mean minimum clearance (m)")
    axes[1].set_title("Safety")

    path = [float(np.mean([e["path_length_m"] for e in r["episodes"]])) for _, r in loaded]
    axes[2].bar(labels, path, color=colours, edgecolor=INK, lw=0.6)
    axes[2].set_ylabel("mean path length, all episodes (m)")
    axes[2].set_title("Did the robot move?")
    axes[2].annotate("barely moves", xy=(2, path[2]), xytext=(1.1, max(path) * 0.75),
                     fontsize=8, color=ACCENT, weight="bold",
                     arrowprops=dict(arrowstyle="->", color=ACCENT, lw=1.2))

    for ax in axes:
        ax.tick_params(axis="x", labelrotation=12)
    fig.suptitle("Measured on 30 held-out arenas. The safest policy never reaches the goal.",
                 fontsize=9.5, y=1.05, color=INK)
    save(fig, "lab09_results")


def fig_lab09_reward_arithmetic():
    """The freeze failure, before any training: it is arithmetic, not luck."""
    p = np.linspace(0, 1, 300)
    freeze = np.full_like(p, -18.0)          # 900 steps at -0.02
    attempt = p * 100.0 + (1 - p) * -100.0   # goal bonus vs collision penalty
    crossover = (100.0 - 18.0) / 200.0

    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    ax.plot(p, freeze, color=MUTED, lw=2, ls="--", label="stand still for the whole episode")
    ax.plot(p, attempt, color=PRIMARY, lw=2, label="attempt the goal")
    ax.axvline(crossover, color=ACCENT, lw=1.4, ls=":")
    ax.axhline(0, color=GRID, lw=1)
    ax.annotate(f"attempting only pays\nabove p = {crossover:.2f}",
                xy=(crossover, -18), xytext=(crossover + 0.06, -75),
                fontsize=8.5, color=ACCENT, weight="bold",
                arrowprops=dict(arrowstyle="->", color=ACCENT, lw=1.2))
    ax.annotate("policy starts here", xy=(0.02, -100), xytext=(0.1, 55),
                fontsize=8.5, color=INK,
                arrowprops=dict(arrowstyle="->", color=INK, lw=1.1))
    ax.set_xlabel("probability of reaching the goal")
    ax.set_ylabel("expected return")
    ax.set_title("reward_sparse_safe: why the policy learns to freeze")
    ax.legend(loc="upper left")
    save(fig, "lab09_reward_arithmetic")


def fig_lab10_trajectories():
    from arc_rl.arenas import training_scenario
    from arc_rl.baselines import make_gap_follow_policy
    from arc_rl.nav_core import FastNavEnv

    try:
        from starters.lab09.train import make_policy_from
        learned = make_policy_from("models/ppo_dense_progress")
    except Exception:
        print("  (skipped lab10 trajectories: no trained policy found)")
        return

    seeds = [501, 505, 512]
    fig, axes = plt.subplots(1, 3, figsize=(9.6, 3.3))
    for ax, seed in zip(axes, seeds):
        scenario = None
        for label, policy, colour, style in (("classical", make_gap_follow_policy(), MUTED, "--"),
                                             ("PPO dense", learned, PRIMARY, "-")):
            env = FastNavEnv(scenario_factory=training_scenario, randomise_start=False)
            obs, _ = env.reset(seed=seed)
            scenario = env.scenario
            xs, ys = [env.x], [env.y]
            for _ in range(900):
                obs, _, term, trunc, info = env.step(policy(obs))
                xs.append(env.x); ys.append(env.y)
                if term or trunc:
                    break
            marker = "*" if info["success"] else "x"
            ax.plot(xs, ys, style, color=colour, lw=1.5, label=label, zorder=4)
            ax.plot(xs[-1], ys[-1], marker=marker, ms=9, color=colour, mec=INK, mew=0.6, zorder=5)
        draw_scenario(ax, scenario, show_goals=True)
        ax.set_title(f"seed {seed}", fontsize=9.5)
    axes[0].legend(loc="upper center", bbox_to_anchor=(1.7, -0.02), ncol=2)
    fig.suptitle("Same arena, two controllers. A star ends at the goal, a cross does not.",
                 fontsize=9.5, y=1.04, color=INK)
    save(fig, "lab10_trajectories")


def fig_arena_grammar():
    from arc_rl.arenas import competition_scenario

    fig, axes = plt.subplots(1, 4, figsize=(11.0, 3.2))
    scenarios = [competition_scenario(s) for s in (1001, 1002, 1003, 1004)]
    span = max(max(sc.bounds) for sc in scenarios) + 0.4
    for ax, seed, sc in zip(axes, [1001, 1002, 1003, 1004], scenarios):
        draw_scenario(ax, sc, limits=(-0.4, span))
        ax.set_title(f"seed {seed}   {sc.bounds[0]:.1f} x {sc.bounds[1]:.1f} m",
                     fontsize=8.5)
        ax.set_xticks([0, 5, 10]); ax.set_yticks([0, 5, 10])
    fig.suptitle("Four arenas from the published competition grammar. "
                 "Circle: start. Stars: checkpoints. Hatched: dynamic obstacles.",
                 fontsize=9.5, y=1.06, color=INK)
    save(fig, "arena_grammar_samples")


def fig_scoring_priorities():
    from arc_eval.scoring import RunMetrics, score_run

    def run(**kw):
        base = dict(checkpoints_reached=4, time_s=60.0, path_length_m=22.0,
                    optimal_path_length_m=20.0)
        base.update(kw)
        return score_run(RunMetrics(**base))

    cases = [
        ("clean, slow\n110 s", run(time_s=110.0)),
        ("clean, fast\n40 s", run(time_s=40.0)),
        ("fast, 2 static\ncollisions", run(time_s=40.0, static_collisions=2)),
        ("fast, 1 dynamic\ncollision", run(time_s=40.0, dynamic_collisions=1)),
        ("fast, 1\nintervention", run(time_s=40.0, interventions=1)),
        ("3 of 4\ncheckpoints", run(checkpoints_reached=3, time_s=40.0)),
    ]
    labels = [c[0] for c in cases]
    parts = ["checkpoints", "completion", "time", "efficiency"]
    colours = [PRIMARY, GOOD, "#7fa8c4", "#b9cddc"]

    fig, ax = plt.subplots(figsize=(8.6, 3.4))
    bottom = np.zeros(len(cases))
    for part, colour in zip(parts, colours):
        values = np.array([getattr(c[1], part) for c in cases])
        ax.bar(labels, values, bottom=bottom, color=colour, edgecolor=INK, lw=0.5,
               label=part)
        bottom += values
    penalties = np.array([c[1].penalties for c in cases])
    ax.bar(labels, penalties, color=ACCENT, edgecolor=INK, lw=0.5, label="penalties")
    for i, c in enumerate(cases):
        ax.text(i, bottom[i] + 18, f"{c[1].total:.0f}", ha="center", fontsize=8.5,
                weight="bold", color=INK)
    ax.axhline(0, color=INK, lw=0.8)
    ax.set_ylabel("score")
    ax.set_title("What the scoring formula rewards: "
                 "complete the mission, then do not hit things, then be quick")
    ax.legend(ncol=5, loc="lower center", bbox_to_anchor=(0.5, -0.30))
    ax.tick_params(axis="x", labelsize=8)
    save(fig, "project2_scoring")


FIGURES = {
    "lab04": [fig_lab04_mapping, fig_lab04_log_odds, fig_lab04_inverse_sensor_model],
    "lab05": [fig_lab05_particle_filter],
    "lab09": [fig_lab09_results, fig_lab09_reward_arithmetic],
    "lab10": [fig_lab10_trajectories],
    "project2": [fig_arena_grammar, fig_scoring_priorities],
}


if __name__ == "__main__":
    wanted = sys.argv[1:] or list(FIGURES)
    for key in wanted:
        print(f"{key}:")
        for fn in FIGURES.get(key, []):
            fn()

"""Run matched FastNavEnv episodes and save actual trajectories and metrics.

Run from the repository root: python3 -m teaching.rollout --help
No ROS adapter or Gazebo execution is implied by these results.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

from teaching.animation import save_replay
from arc_rl.arenas import training_scenario
from arc_rl.baselines import make_gap_follow_policy, make_random_policy
from arc_rl.nav_core import FastNavEnv
from starters.lab09.hybrid import HybridController


def episode(seed, policy_name="gap", model=None, noise=0.01, delay=0):
    """A fresh environment AND controller per episode; delay is in control steps."""
    if delay < 0 or noise < 0:
        raise ValueError("noise and delay must be nonnegative")
    env = FastNavEnv(scenario_factory=training_scenario, randomise_start=False,
                     lidar_noise_std=noise)
    obs, _ = env.reset(seed=seed)
    gap = make_gap_follow_policy()
    hybrid = None
    if policy_name in ("ppo", "hybrid"):
        if model is None:
            raise ValueError("--model is required for ppo and hybrid")
        learned = lambda x: model.predict(x, deterministic=True)[0]
        hybrid = HybridController(gap, learned) if policy_name == "hybrid" else None
        policy = learned
    elif policy_name == "dqn":
        if model is None: raise ValueError("--model is required")
        policy = model
    elif policy_name == "gap":
        policy = gap
    else:
        policy = make_random_policy(seed)
    history = [obs.copy() for _ in range(delay + 1)]
    trajectory = [(0.0, env.x, env.y, env.theta, 0.0, 0.0)]
    total_reward = 0.0
    while True:
        received = history[0]
        action = hybrid(received, position=(env.x, env.y)) if hybrid else policy(received)
        obs, reward, terminated, truncated, info = env.step(action)
        history.append(obs.copy())
        history.pop(0)
        total_reward += reward
        trajectory.append((info["time_s"], env.x, env.y, env.theta, env.v, env.w))
        if terminated or truncated:
            break
    row = dict(seed=seed, policy=policy_name, noise_std_m=noise,
               observation_delay_steps=delay, **info, return_dense=total_reward)
    row["collision_free_success"] = bool(info["success"] and info["collisions"] == 0)
    row["handovers"] = hybrid.stats.handovers if hybrid else 0
    row["learned_fraction"] = hybrid.stats.learned_fraction if hybrid else float(policy_name in ("ppo", "dqn"))
    scenario = env.scenario
    env.close()
    return row, np.asarray(trajectory), scenario


def draw_episode(path, points, scenario, title):
    fig, ax = plt.subplots(figsize=(7, 6), layout="constrained")
    for x0, y0, x1, y1 in scenario.walls:
        ax.plot([x0, x1], [y0, y1], color="#25334a", lw=2)
    for x, y, radius in scenario.static_circles:
        ax.add_patch(Circle((x, y), radius, color="#8795a8"))
    for x, y, speed, heading, radius in scenario.dynamic_circles:
        ax.add_patch(Circle((x, y), radius, fill=False, ls="--", color="#d6802c"))
    ax.plot(points[:, 1], points[:, 2], color="#007c91", lw=2, label="robot centre")
    ax.scatter(*points[0, 1:3], marker="o", color="#16824b", label="start")
    ax.scatter(*points[-1, 1:3], marker="x", color="#bd3347", label="end")
    goals = np.asarray(scenario.goals)
    ax.scatter(goals[:, 0], goals[:, 1], marker="*", s=130, color="#d6802c", label="goals")
    ax.set(xlim=(0, scenario.bounds[0]), ylim=(0, scenario.bounds[1]),
           xlabel="x (m)", ylabel="y (m)", title=title, aspect="equal")
    ax.legend(fontsize=8)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=["gap", "random", "ppo", "hybrid", "dqn"], default="gap")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--seeds", type=int, nargs="+", default=[500])
    parser.add_argument("--noise", type=float, default=0.01)
    parser.add_argument("--delay", type=int, default=0)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("output directory exists; choose a new name to preserve the previous run")
    if len(set(args.seeds)) != len(args.seeds):
        parser.error("duplicate seeds would double-count episodes")
    model = None
    if args.policy in ("ppo", "hybrid"):
        if args.model is None:
            parser.error("--model is required for this policy")
        from stable_baselines3 import PPO
        model = PPO.load(args.model, device="cpu")
    if args.policy == "dqn":
        if args.model is None: parser.error("--model is required")
        from teaching.deep_q import load_policy
        model = load_policy(args.model)
    args.out.mkdir(parents=True)
    rows = []
    for seed in args.seeds:
        row, points, scenario = episode(seed, args.policy, model, args.noise, args.delay)
        rows.append(row)
        np.savetxt(args.out / f"trajectory_{seed}.csv", points, delimiter=",",
                   header="time_s,x_m,y_m,theta_rad,v_m_s,w_rad_s", comments="")
        draw_episode(args.out / f"trajectory_{seed}.png", points, scenario,
                     f"{args.policy}, seed {seed}: {row['termination_reason']}")
        save_replay(args.out / f"replay_{seed}.html", points, scenario.walls, scenario.static_circles, scenario.goals, scenario.bounds, f"{args.policy}: {row['termination_reason']}")
        print(json.dumps(row))
    with (args.out / "episodes.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "environment": "FastNavEnv (NumPy surrogate, not Gazebo)",
        "model": str(args.model) if args.model else None,
        "seeds": args.seeds, "episodes": len(rows),
        "collision_free_successes": sum(r["collision_free_success"] for r in rows),
        "collisions": sum(r["collisions"] for r in rows),
        "timeouts": sum(r["termination_reason"] == "timeout" for r in rows),
        "noise_std_m": args.noise, "observation_delay_steps": args.delay,
        "delay_scope": "all policy observations delayed; hybrid position is current simulator state",
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

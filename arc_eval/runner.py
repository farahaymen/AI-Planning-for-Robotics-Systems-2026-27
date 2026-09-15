"""
Autonomous Robotics Course (ARC) - evaluation harness.

Every benchmarking claim made anywhere in this course, from the Lab 6 controller
comparison to the Project 2 leaderboard, is produced by this harness. Nothing is
graded on a single run.

The harness is deliberately backend agnostic. It takes an environment factory and
a policy callable, so exactly the same code path evaluates:

  * a Nav2 configuration      (env wraps the ROS action client, policy is a stub)
  * a trained RL policy       (env is FastNavEnv or RosNavEnv)
  * a hybrid stack            (env is RosNavEnv, policy is the local controller)

The only requirement is that the environment returns the standard info keys
defined in REQUIRED_INFO_KEYS on the final step of every episode.

Usage
-----
    python -m arc_eval.runner --config arc_eval/configs/lab06_dwb.yaml
    python -m arc_eval.runner --compare results/dwb.json results/mppi.json
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import platform
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional

SCHEMA_VERSION = "1.0"

REQUIRED_INFO_KEYS = (
    "success",
    "collisions",
    "min_clearance_m",
    "path_length_m",
    "time_s",
    "checkpoints_reached",
    "recovery_events",
    "interventions",
    "termination_reason",
)

# Metrics where a larger value is better. Used only for report formatting.
HIGHER_IS_BETTER = {"success_rate", "min_clearance_m", "checkpoints_reached"}


@dataclass
class EpisodeRecord:
    episode_index: int
    seed: int
    success: bool
    termination_reason: str
    time_s: float
    path_length_m: float
    collisions: int
    min_clearance_m: float
    recovery_events: int
    interventions: int
    checkpoints_reached: int
    wall_clock_s: float
    steps: int


def _git_revision() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        return out.stdout.strip() or "unversioned"
    except Exception:
        return "unversioned"


def _mean_sd(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": float("nan"), "sd": float("nan"), "n": 0}
    mean = statistics.fmean(values)
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return {"mean": float(mean), "sd": float(sd), "n": len(values)}


def _wilson_interval(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for the success rate.

    A normal approximation is wrong at the sample sizes used in a lab, where ten
    episodes with ten successes is common. Wilson does not collapse to a zero
    width interval in that case, which is the honest answer to give students.
    """
    if n == 0:
        return (float("nan"), float("nan"))
    p = successes / n
    denom = 1 + z ** 2 / n
    centre = (p + z ** 2 / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z ** 2 / (4 * n ** 2)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def run_evaluation(
    env_factory: Callable[[], Any],
    policy: Callable[[Any], Any],
    seeds: list[int],
    system_label: str,
    scenario_label: str,
    max_steps: int = 2000,
    verbose: bool = True,
) -> dict:
    """Run one episode per seed and return the standard results dictionary."""
    episodes: list[EpisodeRecord] = []

    for index, seed in enumerate(seeds):
        env = env_factory()
        obs, _ = env.reset(seed=seed)
        started = time.time()
        info: dict = {}
        steps = 0

        for steps in range(1, max_steps + 1):
            action = policy(obs)
            obs, _reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                break

        missing = [k for k in REQUIRED_INFO_KEYS if k not in info]
        if missing:
            raise KeyError(
                f"Environment did not report required info keys {missing}. "
                "Every ARC environment must populate REQUIRED_INFO_KEYS on the "
                "terminal step so that results are comparable across teams."
            )

        episodes.append(
            EpisodeRecord(
                episode_index=index,
                seed=seed,
                success=bool(info["success"]),
                termination_reason=str(info["termination_reason"]),
                time_s=float(info["time_s"]),
                path_length_m=float(info["path_length_m"]),
                collisions=int(info["collisions"]),
                min_clearance_m=float(info["min_clearance_m"]),
                recovery_events=int(info["recovery_events"]),
                interventions=int(info["interventions"]),
                checkpoints_reached=int(info["checkpoints_reached"]),
                wall_clock_s=round(time.time() - started, 3),
                steps=steps,
            )
        )
        if hasattr(env, "close"):
            env.close()
        if verbose:
            mark = "ok  " if episodes[-1].success else "fail"
            print(f"  seed {seed:>4}  {mark}  {episodes[-1].termination_reason:<14} "
                  f"t={episodes[-1].time_s:6.2f}s  d={episodes[-1].path_length_m:6.2f}m")

    successes = [e for e in episodes if e.success]
    n = len(episodes)
    lo, hi = _wilson_interval(len(successes), n)

    aggregate = {
        "success_rate": len(successes) / n if n else float("nan"),
        "success_rate_ci95": [lo, hi],
        # Time and path length are aggregated over SUCCESSFUL episodes only.
        # Averaging the duration of failed runs rewards a policy that gives up
        # quickly, which is the classic way navigation benchmarks mislead.
        "time_to_goal_s": _mean_sd([e.time_s for e in successes]),
        "path_length_m": _mean_sd([e.path_length_m for e in successes]),
        "collisions_per_episode": _mean_sd([float(e.collisions) for e in episodes]),
        "min_clearance_m": _mean_sd([e.min_clearance_m for e in episodes]),
        "recovery_events": _mean_sd([float(e.recovery_events) for e in episodes]),
        "interventions_total": sum(e.interventions for e in episodes),
        "checkpoints_reached": _mean_sd([float(e.checkpoints_reached) for e in episodes]),
        "collision_free_rate": sum(1 for e in episodes if e.collisions == 0) / n if n else float("nan"),
    }

    return {
        "schema_version": SCHEMA_VERSION,
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "system_label": system_label,
        "scenario": scenario_label,
        "seeds": seeds,
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "git_revision": _git_revision(),
        },
        "episodes": [asdict(e) for e in episodes],
        "aggregate": aggregate,
    }


def format_report(results: dict) -> str:
    a = results["aggregate"]
    lo, hi = a["success_rate_ci95"]
    lines = [
        f"System    : {results['system_label']}",
        f"Scenario  : {results['scenario']}",
        f"Episodes  : {len(results['episodes'])}  seeds {results['seeds']}",
        "",
        f"Success rate         {a['success_rate']*100:6.1f} %   (95% CI {lo*100:.1f} to {hi*100:.1f})",
        f"Collision free rate  {a['collision_free_rate']*100:6.1f} %",
        f"Time to goal         {a['time_to_goal_s']['mean']:6.2f} +/- {a['time_to_goal_s']['sd']:.2f} s  (successes only)",
        f"Path length          {a['path_length_m']['mean']:6.2f} +/- {a['path_length_m']['sd']:.2f} m  (successes only)",
        f"Collisions/episode   {a['collisions_per_episode']['mean']:6.2f} +/- {a['collisions_per_episode']['sd']:.2f}",
        f"Minimum clearance    {a['min_clearance_m']['mean']:6.3f} +/- {a['min_clearance_m']['sd']:.3f} m",
        f"Recovery events      {a['recovery_events']['mean']:6.2f} +/- {a['recovery_events']['sd']:.2f}",
        f"Interventions        {a['interventions_total']:6d} total",
    ]
    return "\n".join(lines)


def compare(paths: list[str]) -> str:
    """Side by side table for two or more result files."""
    loaded = [json.loads(Path(p).read_text()) for p in paths]
    labels = [r["system_label"] for r in loaded]
    rows = [
        ("Success rate %", lambda a: a["success_rate"] * 100, "{:.1f}"),
        ("Collision free %", lambda a: a["collision_free_rate"] * 100, "{:.1f}"),
        ("Time to goal s", lambda a: a["time_to_goal_s"]["mean"], "{:.2f}"),
        ("Path length m", lambda a: a["path_length_m"]["mean"], "{:.2f}"),
        ("Collisions/ep", lambda a: a["collisions_per_episode"]["mean"], "{:.2f}"),
        ("Min clearance m", lambda a: a["min_clearance_m"]["mean"], "{:.3f}"),
    ]
    width = max(len(l) for l in labels) + 2
    out = ["Metric".ljust(22) + "".join(l.ljust(width) for l in labels)]
    out.append("-" * len(out[0]))
    for name, fn, fmt in rows:
        cells = "".join(fmt.format(fn(r["aggregate"])).ljust(width) for r in loaded)
        out.append(name.ljust(22) + cells)
    out.append("")
    out.append("Report mean and standard deviation. A difference smaller than the")
    out.append("standard deviation is not a result, it is noise.")
    return "\n".join(out)


def _load_callable(spec: str) -> Callable:
    """Resolve 'package.module:attribute' into a callable."""
    module_name, _, attribute = spec.partition(":")
    module = importlib.import_module(module_name)
    return getattr(module, attribute)


def _resolve_specs(args: dict) -> dict:
    """Resolve any 'module:attribute' string in a config argument block.

    This lets a YAML file name a callable, for example pointing env_args at
    arc_rl.arenas:training_scenario, without the runner needing to know which
    arguments happen to be callables.
    """
    out = {}
    for key, value in (args or {}).items():
        if isinstance(value, str) and ":" in value and value.split(":")[0].replace(".", "").isidentifier():
            try:
                out[key] = _load_callable(value)
                continue
            except (ImportError, AttributeError, ValueError):
                pass
        out[key] = value
    return out


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="ARC evaluation harness")
    parser.add_argument("--config", help="YAML run configuration")
    parser.add_argument("--compare", nargs="+", help="Compare two or more result JSON files")
    parser.add_argument("--out", default=None, help="Output JSON path")
    args = parser.parse_args(argv)

    if args.compare:
        print(compare(args.compare))
        return 0

    if not args.config:
        parser.error("either --config or --compare is required")

    import yaml
    cfg = yaml.safe_load(Path(args.config).read_text())

    env_factory = _load_callable(cfg["env_factory"])
    policy_factory = _load_callable(cfg["policy_factory"])
    policy = policy_factory(**_resolve_specs(cfg.get("policy_args", {})))

    env_args = _resolve_specs(cfg.get("env_args", {}))
    results = run_evaluation(
        env_factory=lambda: env_factory(**env_args),
        policy=policy,
        seeds=cfg.get("seeds", list(range(10))),
        system_label=cfg["system_label"],
        scenario_label=cfg.get("scenario", "unspecified"),
        max_steps=cfg.get("max_steps", 2000),
    )

    out_path = Path(args.out or cfg.get("out", "results.json"))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(results, indent=2))
    print()
    print(format_report(results))
    print(f"\nWritten to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

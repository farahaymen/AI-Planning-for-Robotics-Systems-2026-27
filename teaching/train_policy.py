"""PPO with an explicit training geometry bank, kept separate from evaluation."""
from __future__ import annotations

import argparse
import json
import re
from dataclasses import asdict
from pathlib import Path
import time

from arc_rl.arenas import training_scenario
from arc_rl.nav_core import FastNavEnv, OBS, ROBOT


def training_bank(seed):
    return training_scenario(int(seed) % 100)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reward", choices=["dense_progress", "sparse_safe"], default="dense_progress")
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--name", required=True, help="New file stem under models/")
    parser.add_argument("--domain-randomisation", action="store_true")
    args = parser.parse_args()
    if args.steps < 1 or not re.fullmatch(r"[A-Za-z0-9_-]+", args.name):
        parser.error("steps must be positive; name may contain letters, digits, underscores and hyphens")
    path = Path("models") / args.name
    if path.with_suffix(".zip").exists() or path.with_suffix(".json").exists():
        parser.error("model name already exists; choose a new name")
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv

    path.parent.mkdir(parents=True, exist_ok=True)
    log_folder = Path("runs") / args.name
    if log_folder.exists(): parser.error("run folder exists; choose a new name")
    log_folder.mkdir(parents=True)
    counter = iter(range(4))

    def make_env():
        return Monitor(FastNavEnv(scenario_factory=training_bank, reward=args.reward,
                                  randomise_start=False,
                                  domain_randomisation=args.domain_randomisation),
                       filename=str(log_folder / f"env_{next(counter)}"))

    vec = DummyVecEnv([make_env for _ in range(4)])
    model = PPO("MlpPolicy", vec, policy_kwargs=dict(net_arch=[64, 64]),
                n_steps=512, batch_size=256, gae_lambda=0.95, gamma=0.99,
                learning_rate=3e-4, ent_coef=0.005, clip_range=0.2,
                n_epochs=10, seed=args.seed, device="cpu", verbose=1,
                tensorboard_log=None)
    started = time.monotonic()
    try:
        model.learn(total_timesteps=args.steps, progress_bar=False, tb_log_name=args.name)
        path.parent.mkdir(parents=True, exist_ok=True)
        model.save(str(path))
        metadata = dict(training_geometry_seeds="0..99 inclusive, modulo bank",
                        evaluation_geometry_seeds="500..509 reserved for validation",
                        final_test_geometry_seeds="1000..1009 reserved until decisions are frozen",
                        seed=args.seed, requested_steps=args.steps,
                        actual_steps=model.num_timesteps, reward=args.reward,
                        domain_randomisation=args.domain_randomisation,
                        randomisation_scope="lidar noise standard deviation only: 0.005..0.04 m",
                        elapsed_s=time.monotonic() - started,
                        observation=asdict(OBS), robot=asdict(ROBOT))
        path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Saved {path}.zip and {path}.json")
    finally:
        vec.close()


if __name__ == "__main__":
    main()

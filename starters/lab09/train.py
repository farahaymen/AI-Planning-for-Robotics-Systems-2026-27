"""
Lab 9: train a navigation policy on the fast surrogate.

Training happens here, in the NumPy surrogate, NOT in Gazebo. Gazebo runs at a
real time factor below 1.0 inside the course VM, which caps sample collection at
tens of transitions per second. PPO needs hundreds of thousands. The surrogate
delivers thousands per second on one core, which is the difference between a
policy you can train in a laboratory session and one you cannot.

Evaluation and deployment then happen through ROS against Gazebo, using the same
observation contract. The gap between the two is measured in Lab 10.

    python3 starters/lab09/train.py --reward dense_progress --steps 200000
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from arc_rl.arenas import training_scenario
from arc_rl.nav_core import FastNavEnv


def make_env(reward: str, rank: int, domain_randomisation: bool = False):
    def _init():
        env = FastNavEnv(
            scenario_factory=training_scenario,
            reward=reward,
            randomise_start=False,
            domain_randomisation=domain_randomisation,
        )
        return Monitor(env)
    return _init


def train(reward: str, steps: int, n_envs: int = 4, seed: int = 0,
          out: str = "models", domain_randomisation: bool = False):
    # Several environments in parallel, not for speed on one core but because
    # PPO's advantage estimates are far less noisy when each batch contains
    # transitions from several different arenas.
    vec = DummyVecEnv([make_env(reward, i, domain_randomisation) for i in range(n_envs)])

    model = PPO(
        "MlpPolicy",
        vec,
        # 2x64 is a course constraint, not a hyperparameter to tune. It trains on
        # CPU, and at 29 observation dimensions a larger network buys nothing.
        policy_kwargs=dict(net_arch=[64, 64]),
        n_steps=512,
        batch_size=256,
        gae_lambda=0.95,
        gamma=0.99,
        learning_rate=3e-4,
        ent_coef=0.005,
        clip_range=0.2,
        n_epochs=10,
        seed=seed,
        verbose=0,
    )

    started = time.time()
    model.learn(total_timesteps=steps, progress_bar=False)
    elapsed = time.time() - started

    path = Path(out) / f"ppo_{reward}"
    path.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(path))
    return model, elapsed


def make_policy_from(path: str):
    """Wrap a saved policy as a callable for the evaluation harness."""
    model = PPO.load(path, device="cpu")

    def policy(obs):
        action, _ = model.predict(obs, deterministic=True)
        return action
    return policy


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--reward", default="dense_progress")
    p.add_argument("--steps", type=int, default=200_000)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--domain-randomisation", action="store_true")
    a = p.parse_args()

    _, elapsed = train(a.reward, a.steps, seed=a.seed,
                       domain_randomisation=a.domain_randomisation)
    print(f"{a.reward}: {a.steps:,} steps in {elapsed/60:.1f} min "
          f"({a.steps/elapsed:,.0f} steps/s)")

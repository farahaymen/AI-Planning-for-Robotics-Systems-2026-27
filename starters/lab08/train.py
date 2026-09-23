"""
Lab 8: train a navigation policy on the fast surrogate.

This legacy entry point trains in the NumPy surrogate with unrestricted arena
seeds. The student Lab 8 commands use teaching.train_policy instead, which
reserves a fixed training bank and saves metadata for held-out comparisons.
Labs 9-10 evaluate in the same surrogate. A complete learned-policy ROS/Gazebo
adapter is not present; these runs do not establish transfer to Gazebo.

    python3 starters/lab08/train.py --reward dense_progress --steps 200000

The policy is saved to models/ppo_<reward>, or models/ppo_<reward>_dr when
--domain-randomisation is set, so the Lab 10 run does not overwrite the Lab 8
one. Use --name to choose the stem yourself. TensorBoard logs go to runs/.
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


def output_name(reward: str, domain_randomisation: bool = False,
                name: str | None = None) -> str:
    """The file stem this run saves to.

    A randomised run must not overwrite the plain one. Lab 10 trains a
    randomised policy and then compares it against the Lab 8 policy, so if both
    wrote to ppo_dense_progress there would be nothing left to compare.
    """
    if name:
        return name
    return f"ppo_{reward}_dr" if domain_randomisation else f"ppo_{reward}"


def train(reward: str, steps: int, n_envs: int = 4, seed: int = 0,
          out: str = "models", domain_randomisation: bool = False,
          name: str | None = None, tensorboard_dir: str = "runs"):
    # DummyVecEnv steps several environments sequentially in one process.
    # This mixes trajectories from different arenas in each rollout batch.
    vec = DummyVecEnv([make_env(reward, i, domain_randomisation) for i in range(n_envs)])

    # PPO only writes TensorBoard logs if it is told where to put them. Without
    # this, `tensorboard --logdir runs` starts cleanly and shows nothing at all.
    Path(tensorboard_dir).mkdir(parents=True, exist_ok=True)

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
        tensorboard_log=tensorboard_dir,
    )

    stem = output_name(reward, domain_randomisation, name)

    started = time.time()
    model.learn(total_timesteps=steps, progress_bar=False,
                tb_log_name=stem)
    elapsed = time.time() - started

    path = Path(out) / stem
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
    p.add_argument("--name", default=None,
                   help="output file stem under models/. Defaults to "
                        "ppo_<reward>, or ppo_<reward>_dr with randomisation.")
    p.add_argument("--tensorboard-dir", default="runs",
                   help="where PPO writes TensorBoard logs")
    a = p.parse_args()

    _, elapsed = train(a.reward, a.steps, seed=a.seed,
                       domain_randomisation=a.domain_randomisation,
                       name=a.name, tensorboard_dir=a.tensorboard_dir)
    stem = output_name(a.reward, a.domain_randomisation, a.name)
    print(f"{stem}: {a.steps:,} steps in {elapsed/60:.1f} min "
          f"({a.steps/elapsed:,.0f} steps/s)")

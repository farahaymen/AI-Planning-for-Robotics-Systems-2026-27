from pathlib import Path
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from robot_workshop.approach_env import ApproachEnv


def main():
    output = Path('models/student_approach.zip')
    if output.exists():
        raise FileExistsError('Keep the earlier model; choose a new output filename')
    output.parent.mkdir(parents=True, exist_ok=True)
    env = Monitor(ApproachEnv())
    try:
        model = PPO('MlpPolicy', env, n_steps=128, batch_size=64,
                    policy_kwargs={'net_arch': [32, 32]}, seed=0,
                    device='cpu', verbose=1)
        model.learn(total_timesteps=2048)
        model.save(str(output))
    finally:
        env.close()
    model = PPO.load(str(output), device='cpu')
    for seed in (100, 101, 102):
        test = ApproachEnv()
        observation, info = test.reset(seed=seed)
        for step in range(50):
            action, _ = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, info = test.step(action)
            if terminated or truncated:
                print(seed, 'success:', terminated, 'steps:', step + 1)
                break
        test.close()

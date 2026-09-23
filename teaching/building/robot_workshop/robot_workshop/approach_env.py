import gymnasium as gym
import numpy as np


class ApproachEnv(gym.Env):
    """A final straight approach to a goal 1 m along an empty corridor.

    This teaching task has one action and one observation. It does not share
    the warehouse policy's 29-input, 2-action interface.
    """
    metadata = {'render_modes': []}

    def __init__(self):
        self.action_space = gym.spaces.Box(0.0, 1.0, (1,), np.float32)
        self.observation_space = gym.spaces.Box(0.0, 1.0, (1,), np.float32)
        self.x = 0.0
        self.steps = 0

    def observation(self):
        return np.array([max(0.0, 1.0 - self.x)], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.x = float(self.np_random.uniform(0.0, 0.2))
        self.steps = 0
        return self.observation(), {'x_m': self.x}

    def step(self, action):
        speed = 0.5 * float(np.clip(action[0], 0.0, 1.0))
        previous = self.x
        self.x = min(1.0, self.x + speed * 0.1)
        self.steps += 1
        terminated = self.x >= 0.98
        truncated = self.steps >= 50 and not terminated
        reward = (self.x - previous) - 0.01 + float(terminated)
        return self.observation(), reward, terminated, truncated, {'x_m': self.x}


def main():
    env = ApproachEnv()
    observation, info = env.reset(seed=7)
    for step in range(50):
        observation, reward, terminated, truncated, info = env.step(np.array([1.0]))
        print(step + 1, observation, reward, terminated, truncated)
        if terminated or truncated:
            break
    env.close()

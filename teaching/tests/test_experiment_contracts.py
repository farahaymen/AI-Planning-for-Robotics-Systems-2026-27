"""Check reproducibility and evaluation boundaries, not plotting cosmetics."""
import numpy as np

from arc_rl.arenas import training_scenario
from teaching.rollout import episode
from teaching.train_policy import training_bank


def test_training_bank_never_selects_validation_geometry_seeds(monkeypatch):
    seen = []
    monkeypatch.setattr("teaching.train_policy.training_scenario", lambda seed: seen.append(seed))
    for seed in [0, 99, 100, 500, 509, 1000, 999999]:
        training_bank(seed)
    assert seen == [0, 99, 0, 0, 9, 0, 99]


def test_repeated_episode_reproduces_trajectory_and_outcome():
    first, path_a, _ = episode(500, noise=0.01)
    second, path_b, _ = episode(500, noise=0.01)
    assert first == second
    np.testing.assert_array_equal(path_a, path_b)
    assert first["collision_free_success"] == (first["success"] and first["collisions"] == 0)


def test_delay_queue_retains_initial_observation_for_requested_steps(monkeypatch):
    observations = []

    def spy_policy():
        def policy(obs):
            observations.append(obs.copy())
            return np.array([0.2, 0.0], dtype=np.float32)
        return policy

    monkeypatch.setattr("teaching.rollout.make_gap_follow_policy", spy_policy)
    episode(500, noise=0, delay=4)
    assert len(observations) > 5
    for obs in observations[1:5]:
        np.testing.assert_array_equal(obs, observations[0])
    assert not np.array_equal(observations[5], observations[0])


def test_hybrid_state_does_not_leak_between_episodes():
    class DummyModel:
        def predict(self, obs, deterministic=True):
            return np.array([0.2, 0.6], dtype=np.float32), None
    model = DummyModel()
    first, path_a, _ = episode(501, "hybrid", model, noise=0)
    second, path_b, _ = episode(501, "hybrid", model, noise=0)
    assert first == second
    np.testing.assert_array_equal(path_a, path_b)

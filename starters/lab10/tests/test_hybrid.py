import sys; sys.path.insert(0, "starters/lab10"); sys.path.insert(0, ".")
import numpy as np, pytest
from arc_rl.nav_core import OBS
from hybrid import HybridConfig, HybridController, make_hybrid_policy

CLASSICAL = np.array([0.5, 0.0], dtype=np.float32)
LEARNED = np.array([-0.5, 1.0], dtype=np.float32)


def obs_with_front(distance_m):
    o = np.ones(OBS.size, dtype=np.float32)
    norm = (distance_m - OBS.lidar_min_range) / (OBS.lidar_max_range - OBS.lidar_min_range)
    o[:OBS.n_beams] = norm
    return o


def controller(**kw):
    return HybridController(lambda o: CLASSICAL, lambda o: LEARNED, HybridConfig(**kw))


def test_classical_is_the_default():
    c = controller()
    assert np.array_equal(c(obs_with_front(5.0), (0.0, 0.0)), CLASSICAL)


def test_no_handover_while_the_robot_is_moving_even_in_tight_space():
    c = controller(stall_window=10, stall_displacement=0.12)
    for i in range(40):
        action = c(obs_with_front(0.3), (i * 0.1, 0.0))
    assert np.array_equal(action, CLASSICAL) and c.stats.handovers == 0


def test_no_handover_while_stalled_in_open_space():
    """Stalled but with room ahead is not the case the learned policy helps with."""
    c = controller(stall_window=10)
    for _ in range(40):
        action = c(obs_with_front(8.0), (0.0, 0.0))
    assert np.array_equal(action, CLASSICAL) and c.stats.handovers == 0


def test_handover_fires_exactly_when_the_stall_window_fills():
    c = controller(stall_window=10, handover_steps=5)
    actions = [c(obs_with_front(0.3), (0.0, 0.0)) for _ in range(10)]
    assert all(np.array_equal(a, CLASSICAL) for a in actions[:9])
    assert np.array_equal(actions[9], LEARNED)
    assert c.stats.handovers == 1


def test_the_commitment_lasts_exactly_handover_steps():
    """Once handed over, the learned policy holds for the configured number of
    steps even if the space clears immediately. Re-deciding every tick makes the
    controller oscillate at the boundary and behave worse than either policy."""
    c = controller(stall_window=10, handover_steps=5)
    for _ in range(10):
        c(obs_with_front(0.3), (0.0, 0.0))       # triggers on the 10th
    # Space is now wide open. The commitment must still hold.
    after = [c(obs_with_front(9.0), (0.0, 0.0)) for _ in range(5)]
    assert sum(np.array_equal(a, LEARNED) for a in after) == 4
    assert c.stats.learned_steps == 5
    assert np.array_equal(after[-1], CLASSICAL)


def test_handovers_are_capped_per_episode():
    c = controller(stall_window=5, handover_steps=1, max_handovers=3)
    for i in range(400):
        c(obs_with_front(0.3), (0.0, 0.0))
    assert c.stats.handovers == 3


def test_stats_track_the_split():
    c = controller(stall_window=5, handover_steps=10)
    for _ in range(60):
        c(obs_with_front(0.3), (0.0, 0.0))
    assert 0.0 < c.stats.learned_fraction < 1.0


def test_reset_clears_history_between_episodes():
    c = controller(stall_window=5, handover_steps=2)
    for _ in range(20):
        c(obs_with_front(0.3), (0.0, 0.0))
    c.reset()
    assert c.stats.handovers == 0
    assert np.array_equal(c(obs_with_front(0.3), (0.0, 0.0)), CLASSICAL)


def test_harness_adapter_matches_the_policy_signature():
    policy = make_hybrid_policy(lambda o: CLASSICAL, lambda o: LEARNED)
    action = policy(obs_with_front(5.0))
    assert action.shape == (2,) and hasattr(policy, "controller")

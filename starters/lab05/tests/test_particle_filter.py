"""
Tests for Lab 5. Monte Carlo localisation.

By default these test the reference filter in `particle_filter.py`. To test
your own work in `particle_filter_skeleton.py`:

    ARC_PARTICLE_FILTER=particle_filter_skeleton python3 -m pytest starters/lab05 -q
"""

import importlib
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

V = importlib.import_module(os.environ.get("ARC_PARTICLE_FILTER", "particle_filter"))

NO_NOISE = V.MotionNoise(0.0, 0.0, 0.0, 0.0)


def test_seeding_around_a_pose_centres_the_cloud_there():
    pf = V.ParticleFilter(2000, seed=1)
    pf.seed_around((3.0, -1.0, 0.5), xy_std=0.2, theta_std=0.05)
    x, y, th = pf.estimate()
    assert (x, y) == pytest.approx((3.0, -1.0), abs=0.05)
    assert th == pytest.approx(0.5, abs=0.02)


def test_noiseless_prediction_is_exact_dead_reckoning():
    pf = V.ParticleFilter(10, seed=0, noise=NO_NOISE)
    pf.seed_around((0.0, 0.0, 0.0), xy_std=0.0, theta_std=0.0)
    pf.predict(0.0, 1.0, 0.0)
    assert pf.estimate()[:2] == pytest.approx((1.0, 0.0), abs=1e-9)


def test_prediction_noise_spreads_the_cloud():
    pf = V.ParticleFilter(1000, seed=2)
    pf.seed_around((0.0, 0.0, 0.0), xy_std=0.01, theta_std=0.01)
    before = pf.spread()
    for _ in range(10):
        pf.predict(0.0, 0.5, 0.0)
    assert pf.spread() > before * 3, "cloud must diverge without measurements"


def test_effective_sample_size_is_n_when_weights_are_uniform():
    pf = V.ParticleFilter(400, seed=0)
    assert pf.effective_sample_size() == pytest.approx(400.0)


def test_effective_sample_size_collapses_when_one_particle_dominates():
    pf = V.ParticleFilter(400, seed=0)
    errors = np.full(400, 10.0)
    errors[7] = 0.0
    pf.update_weights(errors, sigma=0.3)
    assert pf.effective_sample_size() < 2.0


def test_resampling_preserves_the_particle_count_and_resets_weights():
    pf = V.ParticleFilter(300, seed=3)
    pf.seed_uniform((0, 0, 10, 10))
    errors = np.abs(pf.particles[:, 0] - 5.0)
    pf.update_weights(errors)
    pf.resample()
    assert pf.particles.shape == (300, 3)
    assert np.allclose(pf.weights, 1.0 / 300)


def test_resampling_concentrates_particles_on_the_good_hypothesis():
    pf = V.ParticleFilter(1000, seed=4)
    pf.seed_uniform((0, 0, 10, 10))
    pf.update_weights(np.abs(pf.particles[:, 0] - 5.0), sigma=0.3)
    pf.resample()
    assert pf.particles[:, 0].mean() == pytest.approx(5.0, abs=0.2)


def test_weights_never_collapse_to_all_zero_when_the_robot_is_lost():
    pf = V.ParticleFilter(200, seed=5)
    pf.update_weights(np.full(200, 500.0), sigma=0.1)
    assert np.isfinite(pf.weights).all() and pf.weights.sum() == pytest.approx(1.0)


def test_maybe_resample_holds_off_while_the_cloud_is_healthy():
    pf = V.ParticleFilter(500, seed=6)
    assert pf.maybe_resample() is False
    pf.update_weights(np.linspace(0.0, 5.0, 500), sigma=0.2)
    assert pf.maybe_resample() is True


def test_wrong_error_length_is_rejected_rather_than_broadcast():
    pf = V.ParticleFilter(100, seed=0)
    with pytest.raises(ValueError):
        pf.update_weights(np.zeros(99))


def test_angle_estimate_is_correct_across_the_pi_discontinuity():
    """Arithmetic averaging of +179 and -179 gives 0, which is exactly wrong."""
    pf = V.ParticleFilter(2, seed=0)
    pf.particles[:] = [[0.0, 0.0, math.radians(179)], [0.0, 0.0, math.radians(-179)]]
    pf.weights[:] = 0.5
    assert abs(pf.estimate()[2]) == pytest.approx(math.pi, abs=0.02)


def test_odometry_increment_decomposes_a_straight_move():
    r1, tr, r2 = V.odometry_increment((0, 0, 0), (1.0, 0.0, 0.0))
    assert (r1, tr, r2) == pytest.approx((0.0, 1.0, 0.0))


def test_odometry_increment_ignores_heading_of_a_negligible_translation():
    r1, tr, r2 = V.odometry_increment((0, 0, 0), (1e-9, 1e-9, 0.4))
    assert r1 == 0.0 and r2 == pytest.approx(0.4)

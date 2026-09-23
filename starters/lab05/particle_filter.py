"""
Lab 5: Monte Carlo localisation, reduced to its three moving parts.

AMCL is a production implementation with adaptive particle counts, KLD sampling
and a tuned likelihood field. You will configure AMCL in this lab. You will not
reimplement it, because that is a week of work and the value is in understanding
the mechanism rather than reproducing the engineering.

This module is the mechanism: predict, weight, resample. It is about 120 lines
and it is enough to see why localisation recovers from odometry drift, and why
it fails in a corridor that looks like every other corridor.

Nothing here depends on ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MotionNoise:
    """Odometry noise model, following the alpha parameters AMCL also uses.

    alpha1 is rotation noise from rotating, alpha2 rotation noise from
    translating, alpha3 translation noise from translating, alpha4 translation
    noise from rotating. These four numbers are what you tune in amcl.yaml, and
    tuning them blind is why most people never get AMCL to behave.
    """

    alpha1: float = 0.2
    alpha2: float = 0.2
    alpha3: float = 0.2
    alpha4: float = 0.2


class ParticleFilter:
    """Each particle is one hypothesis about where the robot is."""

    def __init__(self, n_particles: int = 500, seed: int = 0,
                 noise: MotionNoise = MotionNoise()):
        self.rng = np.random.default_rng(seed)
        self.n = n_particles
        self.noise = noise
        # Columns: x, y, theta
        self.particles = np.zeros((n_particles, 3), dtype=np.float64)
        self.weights = np.full(n_particles, 1.0 / n_particles)

    # -- initialisation ------------------------------------------------------

    def seed_around(self, pose: tuple[float, float, float],
                    xy_std: float = 0.3, theta_std: float = 0.2) -> None:
        """Start from a known approximate pose, as AMCL does from an initial pose."""
        x, y, theta = pose
        self.particles[:, 0] = self.rng.normal(x, xy_std, self.n)
        self.particles[:, 1] = self.rng.normal(y, xy_std, self.n)
        self.particles[:, 2] = self.rng.normal(theta, theta_std, self.n)
        self.weights[:] = 1.0 / self.n

    def seed_uniform(self, bounds: tuple[float, float, float, float]) -> None:
        """Global localisation: no idea where the robot is. The hard case."""
        x0, y0, x1, y1 = bounds
        self.particles[:, 0] = self.rng.uniform(x0, x1, self.n)
        self.particles[:, 1] = self.rng.uniform(y0, y1, self.n)
        self.particles[:, 2] = self.rng.uniform(-math.pi, math.pi, self.n)
        self.weights[:] = 1.0 / self.n

    # -- 1. predict ----------------------------------------------------------

    def predict(self, delta_rot1: float, delta_trans: float, delta_rot2: float) -> None:
        """Move every particle by the odometry increment, plus noise.

        The odometry increment is decomposed into rotate, translate, rotate.
        This is the standard sample_motion_model_odometry from Probabilistic
        Robotics and it is what AMCL's DifferentialMotionModel implements.

        Noise is what spreads the particles out. Without it every particle
        follows the same trajectory, the cloud never explores alternatives, and
        the filter cannot recover from being wrong.
        """
        a = self.noise
        n = self.n
        r1 = delta_rot1 + self.rng.normal(
            0.0, math.sqrt(a.alpha1 * delta_rot1 ** 2 + a.alpha2 * delta_trans ** 2), n)
        tr = delta_trans + self.rng.normal(
            0.0, math.sqrt(a.alpha3 * delta_trans ** 2
                           + a.alpha4 * (delta_rot1 ** 2 + delta_rot2 ** 2)), n)
        r2 = delta_rot2 + self.rng.normal(
            0.0, math.sqrt(a.alpha1 * delta_rot2 ** 2 + a.alpha2 * delta_trans ** 2), n)

        theta = self.particles[:, 2]
        self.particles[:, 0] += tr * np.cos(theta + r1)
        self.particles[:, 1] += tr * np.sin(theta + r1)
        self.particles[:, 2] = wrap_angle(theta + r1 + r2)

    # -- 2. weight -----------------------------------------------------------

    def update_weights(self, particle_errors: np.ndarray, sigma: float = 0.3) -> None:
        """Reweight particles from a per-particle measurement error.

        The caller supplies one error per particle. The teaching figure uses
        synthetic landmark ranges; this helper does not implement AMCL's lidar
        likelihood field. It applies a Gaussian to each error and normalises.
        """
        errors = np.asarray(particle_errors, dtype=np.float64)
        if errors.shape[0] != self.n:
            raise ValueError(f"expected {self.n} errors, got {errors.shape[0]}")

        likelihood = np.exp(-0.5 * (errors / sigma) ** 2)
        # A floor prevents every weight collapsing to exactly zero when the robot
        # is genuinely lost. Without it the normalisation divides by zero and the
        # filter dies rather than recovering.
        likelihood = np.maximum(likelihood, 1e-12)
        self.weights = likelihood / likelihood.sum()

    # -- 3. resample ---------------------------------------------------------

    def effective_sample_size(self) -> float:
        """1 / sum(w^2). Equals n when weights are uniform, 1 when one dominates.

        This is the number that tells you whether to resample. Resampling every
        step throws away diversity for no reason and causes particle deprivation,
        where the filter becomes certain and wrong.
        """
        return float(1.0 / np.sum(self.weights ** 2))

    def resample(self) -> None:
        """Low variance (systematic) resampling.

        One random offset places n evenly spaced samples on the cumulative
        weight distribution. This teaching implementation illustrates resampling;
        it does not reproduce AMCL's adaptive particle-count algorithm.
        """
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        indices = np.searchsorted(cumulative, positions)
        self.particles = self.particles[indices].copy()
        self.weights[:] = 1.0 / self.n

    def maybe_resample(self, threshold_ratio: float = 0.5) -> bool:
        if self.effective_sample_size() < threshold_ratio * self.n:
            self.resample()
            return True
        return False

    # -- output --------------------------------------------------------------

    def estimate(self) -> tuple[float, float, float]:
        """Weighted mean pose. Angles are averaged as unit vectors.

        Averaging angles arithmetically gives 0 for the pair (+179, -179), which
        is exactly opposite the truth. This costs two extra lines and removes a
        bug that only appears when the robot happens to face a particular way.
        """
        x = float(np.average(self.particles[:, 0], weights=self.weights))
        y = float(np.average(self.particles[:, 1], weights=self.weights))
        s = float(np.average(np.sin(self.particles[:, 2]), weights=self.weights))
        c = float(np.average(np.cos(self.particles[:, 2]), weights=self.weights))
        return x, y, math.atan2(s, c)

    def spread(self) -> float:
        """Standard deviation of particle position. Your convergence measure."""
        mean = self.particles[:, :2].mean(axis=0)
        return float(np.sqrt(np.mean(np.sum((self.particles[:, :2] - mean) ** 2, axis=1))))


def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


def odometry_increment(previous: tuple[float, float, float],
                       current: tuple[float, float, float]
                       ) -> tuple[float, float, float]:
    """Decompose two consecutive odometry poses into rotate, translate, rotate."""
    dx = current[0] - previous[0]
    dy = current[1] - previous[1]
    trans = math.hypot(dx, dy)
    # Below a threshold the heading of a tiny translation is pure noise, so
    # attributing a rotation to it injects garbage into the filter.
    rot1 = wrap_angle(math.atan2(dy, dx) - previous[2]) if trans > 1e-4 else 0.0
    rot2 = wrap_angle(current[2] - previous[2] - rot1)
    return float(rot1), float(trans), float(rot2)

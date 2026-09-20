"""
Lab 5: Monte Carlo localisation, reduced to its three moving parts.

This is the file you edit. Fill in every block marked TODO and nothing else;
the initialisation, the diagnostics and the geometry helpers are already
correct and the tests depend on them staying that way.

    python3 -m pytest starters/lab05 -q
    ARC_PARTICLE_FILTER=particle_filter_skeleton python3 -m pytest starters/lab05 -q

The second command tests your work. The first tests the reference, so you can
see what the tests look like when they pass.

The three moving parts are predict, weight, resample. Everything else in a
production filter such as AMCL is engineering on top of those.

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

        # TODO 1: draw a noisy copy of each of the three increments, one value
        #         per particle, using self.rng.normal(0.0, std, n):
        #
        #           r1 = delta_rot1  + noise, variance a.alpha1 * delta_rot1**2
        #                                             + a.alpha2 * delta_trans**2
        #           tr = delta_trans + noise, variance a.alpha3 * delta_trans**2
        #                                             + a.alpha4 * (delta_rot1**2
        #                                                           + delta_rot2**2)
        #           r2 = delta_rot2  + noise, variance a.alpha1 * delta_rot2**2
        #                                             + a.alpha2 * delta_trans**2
        #
        #         The alphas scale variances, so pass math.sqrt(variance) as the
        #         standard deviation. Pass `n` as the size: one scalar draw
        #         shared by all particles moves the cloud rigidly and the filter
        #         never spreads.
        #
        # TODO 2: apply the rotate, translate, rotate move to every particle.
        #         With theta = self.particles[:, 2] read BEFORE you overwrite it:
        #
        #           x     += tr * cos(theta + r1)
        #           y     += tr * sin(theta + r1)
        #           theta  = wrap_angle(theta + r1 + r2)
        #
        #         Use the old heading for x and y. Updating theta first and then
        #         translating applies the second rotation before the move, which
        #         curves every trajectory slightly and is hard to see in a plot.
        raise NotImplementedError("TODO: implement predict")

    # -- 2. weight -----------------------------------------------------------

    def update_weights(self, particle_errors: np.ndarray, sigma: float = 0.3) -> None:
        """Reweight particles from a per-particle measurement error.

        Computing the error is the measurement model's job and is supplied in the
        lab starter, because ray casting every beam for every particle is slow in
        pure Python and is not the concept being taught. What matters here is the
        shape: a Gaussian on the error, then normalise.
        """
        errors = np.asarray(particle_errors, dtype=np.float64)
        if errors.shape[0] != self.n:
            raise ValueError(f"expected {self.n} errors, got {errors.shape[0]}")

        # TODO 3: score every particle with a zero mean Gaussian on its error,
        #         likelihood = exp(-0.5 * (errors / sigma) ** 2). A particle
        #         whose predicted scan matches the real one scores near 1, one
        #         that is metres out scores near 0.
        #
        # TODO 4: floor the likelihood at 1e-12 with np.maximum. When the robot
        #         is genuinely lost every term underflows to exactly 0 and the
        #         normalisation below divides by zero, so the filter dies at the
        #         moment it most needs to recover.
        #
        # TODO 5: normalise, so self.weights sums to 1. Every other method here
        #         assumes that, including effective_sample_size and estimate.
        raise NotImplementedError("TODO: implement update_weights")

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

        One random number rather than n. Compared with drawing n independent
        samples this preserves diversity better and runs in linear time, and it
        is what AMCL uses.
        """
        # TODO 6: lay out n equally spaced sample positions along [0, 1) from a
        #         SINGLE random offset:
        #
        #           positions = (self.rng.random() + np.arange(self.n)) / self.n
        #
        #         Drawing n independent numbers instead is multinomial
        #         resampling. It still works, but it adds variance and loses
        #         particles that the systematic version keeps.
        #
        # TODO 7: build the cumulative weight distribution with np.cumsum, then
        #         set the last entry to exactly 1.0. Rounding can leave the sum
        #         a hair under 1, and a position above it makes np.searchsorted
        #         return n, which is out of range.
        #
        # TODO 8: pick the particles with np.searchsorted(cumulative, positions)
        #         and replace self.particles with those rows, copied.
        #
        # TODO 9: reset self.weights to 1.0 / n. The sample itself now carries
        #         the distribution, so keeping the old weights counts the
        #         measurement twice.
        raise NotImplementedError("TODO: implement resample")

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

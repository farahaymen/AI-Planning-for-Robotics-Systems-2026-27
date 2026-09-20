"""
Autonomous Robotics Course (ARC) - shared navigation contract and fast surrogate.

Lab 8. This is the file you edit. The geometry, the raycasting and the
observation assembly are given complete. Four things are marked TODO: reset,
the pose integration in step, the termination logic, and the info dictionary.
Fill those in and change nothing else.

    python3 -m pytest starters/lab08 -q
    ARC_NAV_CORE=nav_core_skeleton python3 -m pytest starters/lab08 -q

The second command tests your work, the first tests the reference environment
in arc_rl/nav_core.py. The tests are the specification; read them first.

This module defines ONE observation and action contract that is shared by two
environments:

  1. FastNavEnv          pure NumPy, no ROS, no Gazebo, used for TRAINING
  2. RosNavEnv           rclpy + Gazebo backed, used for EVALUATION and DEPLOYMENT
                         (see arc_rl/ros_nav_env.py)

Both environments import RobotSpec, ObsSpec and build_observation from here, so a
policy trained in the surrogate can be loaded and executed against the real ROS
stack without any change to the network shape or the meaning of any element of
the observation vector.

Course: Autonomous Robotics with ROS 2
Baseline: ARC VM 2026.1
Python 3.12, NumPy >= 1.26, Gymnasium >= 1.0
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Gymnasium is required. Inside the course VM run: pip install gymnasium"
    ) from exc


# ---------------------------------------------------------------------------
# Frozen course constants. These MUST match the URDF/Xacro of the reference
# robot and the LiDAR plugin configuration in the Gazebo world files.
# Changing anything here invalidates every previously trained policy.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RobotSpec:
    """Physical parameters of the ARC reference differential drive robot."""

    wheel_radius: float = 0.050          # m
    wheel_separation: float = 0.350      # m
    mass: float = 6.0                    # kg
    footprint_radius: float = 0.220      # m, circumscribed
    max_linear_velocity: float = 0.50    # m/s
    min_linear_velocity: float = -0.125  # m/s, limited reverse
    max_angular_velocity: float = 1.80   # rad/s
    control_period: float = 0.05         # s, 20 Hz command loop


@dataclass(frozen=True)
class ObsSpec:
    """Observation contract shared by the surrogate and the ROS environment."""

    n_beams: int = 24                    # downsampled from the 360 beam scan
    lidar_min_range: float = 0.12        # m
    lidar_max_range: float = 12.0        # m
    max_goal_distance: float = 20.0      # m, normalisation clip only
    goal_tolerance: float = 0.25         # m
    clearance_warning: float = 0.10      # m, used by the evaluation harness

    @property
    def size(self) -> int:
        # n_beams + [goal_dist, sin(goal_angle), cos(goal_angle), v, w]
        return self.n_beams + 5


ROBOT = RobotSpec()
OBS = ObsSpec()


def downsample_scan(ranges: np.ndarray, n_beams: int = OBS.n_beams) -> np.ndarray:
    """Reduce a full LiDAR scan to n_beams by taking the minimum of each sector.

    The minimum rather than the mean is deliberate. A thin table leg that appears
    in one raw beam must survive downsampling, otherwise the policy learns to
    drive through furniture. The ROS environment calls this on the incoming
    sensor_msgs/LaserScan so that both environments see identical statistics.
    """
    ranges = np.asarray(ranges, dtype=np.float32)
    ranges = np.nan_to_num(ranges, nan=OBS.lidar_max_range,
                           posinf=OBS.lidar_max_range, neginf=OBS.lidar_max_range)
    sectors = np.array_split(ranges, n_beams)
    return np.array([float(np.min(s)) for s in sectors], dtype=np.float32)


def build_observation(
    beams: np.ndarray,
    goal_distance: float,
    goal_angle: float,
    linear_velocity: float,
    angular_velocity: float,
    obs_spec: ObsSpec = OBS,
    robot: RobotSpec = ROBOT,
) -> np.ndarray:
    """Assemble the normalised observation vector.

    Every element is scaled to roughly [-1, 1] so that a small MLP trains without
    input normalisation layers. The heading is encoded as a sine/cosine pair
    rather than a raw angle to avoid the discontinuity at +/- pi, which otherwise
    produces a policy that behaves strangely when the goal is directly behind the
    robot.
    """
    beams = np.clip(beams, obs_spec.lidar_min_range, obs_spec.lidar_max_range)
    beams_norm = (beams - obs_spec.lidar_min_range) / (
        obs_spec.lidar_max_range - obs_spec.lidar_min_range
    )
    dist_norm = min(goal_distance, obs_spec.max_goal_distance) / obs_spec.max_goal_distance
    obs = np.concatenate(
        [
            beams_norm.astype(np.float32),
            np.array(
                [
                    dist_norm,
                    math.sin(goal_angle),
                    math.cos(goal_angle),
                    linear_velocity / robot.max_linear_velocity,
                    angular_velocity / robot.max_angular_velocity,
                ],
                dtype=np.float32,
            ),
        ]
    )
    return obs.astype(np.float32)


def observation_space(obs_spec: ObsSpec = OBS) -> spaces.Box:
    low = np.concatenate([np.zeros(obs_spec.n_beams), np.array([0.0, -1.0, -1.0, -1.0, -1.0])])
    high = np.ones(obs_spec.size)
    return spaces.Box(low=low.astype(np.float32), high=high.astype(np.float32), dtype=np.float32)


def action_space() -> spaces.Box:
    """Normalised action. a[0] -> linear velocity, a[1] -> angular velocity."""
    return spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)


def scale_action(action: np.ndarray, robot: RobotSpec = ROBOT) -> tuple[float, float]:
    """Map a normalised action onto a geometry_msgs/Twist compatible pair."""
    a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
    # a[0] in [-1, 1] maps asymmetrically so that reverse is available but slow.
    if a[0] >= 0.0:
        v = float(a[0]) * robot.max_linear_velocity
    else:
        v = float(a[0]) * abs(robot.min_linear_velocity)
    w = float(a[1]) * robot.max_angular_velocity
    return v, w


# ---------------------------------------------------------------------------
# Reward functions. Students compare at least two of these in Lab 9.
# Keeping them as separate callables rather than if/else branches inside the
# environment means a reward change never silently alters the observation or
# termination logic, which is the usual source of unreproducible RL results.
# ---------------------------------------------------------------------------

@dataclass
class RewardTerms:
    """Everything a reward function is allowed to see."""

    previous_goal_distance: float
    goal_distance: float
    min_beam: float
    linear_velocity: float
    angular_velocity: float
    reached_goal: bool
    collided: bool
    step_index: int


def reward_dense_progress(t: RewardTerms) -> float:
    """Reward A: dense shaping on distance progress.

    Trains quickly but is prone to reward hacking. Watch for policies that orbit
    the goal collecting progress from oscillation rather than arriving.
    """
    r = 3.0 * (t.previous_goal_distance - t.goal_distance)
    r -= 0.01                                   # time penalty
    if t.min_beam < 0.5:
        r -= 0.15 * (0.5 - t.min_beam)          # proximity discouragement
    r -= 0.005 * abs(t.angular_velocity)        # smoothness
    if t.reached_goal:
        r += 60.0
    if t.collided:
        r -= 60.0
    return float(r)


def reward_sparse_safe(t: RewardTerms) -> float:
    """Reward B: near sparse, heavier safety weighting.

    Slower to train and needs more exploration, but the resulting policies keep
    larger clearances. This is the pair that makes the Lab 9 comparison
    interesting rather than decorative.
    """
    r = -0.02
    if t.min_beam < 0.4:
        r -= 0.6 * (0.4 - t.min_beam)
    if t.reached_goal:
        r += 100.0
    if t.collided:
        r -= 100.0
    return float(r)


REWARDS: dict[str, Callable[[RewardTerms], float]] = {
    "dense_progress": reward_dense_progress,
    "sparse_safe": reward_sparse_safe,
}


# ---------------------------------------------------------------------------
# Geometry for the surrogate
# ---------------------------------------------------------------------------

@dataclass
class Scenario:
    """A training or evaluation arena expressed in the surrogate's own terms."""

    name: str = "default"
    bounds: tuple[float, float] = (10.0, 10.0)     # width, height in metres
    walls: list[tuple[float, float, float, float]] = field(default_factory=list)
    static_circles: list[tuple[float, float, float]] = field(default_factory=list)
    dynamic_circles: list[tuple[float, float, float, float, float]] = field(default_factory=list)
    start_pose: tuple[float, float, float] = (1.0, 1.0, 0.0)
    goals: list[tuple[float, float]] = field(default_factory=lambda: [(8.0, 8.0)])
    max_episode_steps: int = 600

    @classmethod
    def boxed_room(cls, width: float = 10.0, height: float = 10.0) -> "Scenario":
        w, h = width, height
        return cls(
            name="boxed_room",
            bounds=(w, h),
            walls=[(0, 0, w, 0), (w, 0, w, h), (w, h, 0, h), (0, h, 0, 0)],
        )


def _ray_hits_segments(px, py, dx, dy, segs, max_range):
    """Vectorised ray/segment intersection. segs is (M, 4)."""
    if segs.shape[0] == 0:
        return np.full_like(dx, max_range)
    ax, ay, bx, by = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    ex, ey = bx - ax, by - ay                       # (M,)
    fx = ax[None, :] - px                           # (R, M)
    fy = ay[None, :] - py
    dxe = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]      # d x e
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (fx * ey[None, :] - fy * ex[None, :]) / dxe
        u = (fx * dy[:, None] - fy * dx[:, None]) / dxe
    valid = (np.abs(dxe) > 1e-12) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
    t = np.where(valid, t, np.inf)
    return np.minimum(np.min(t, axis=1), max_range)


def _ray_hits_circles(px, py, dx, dy, circles, max_range):
    """Vectorised ray/circle intersection. circles is (N, 3) of (cx, cy, r)."""
    if circles.shape[0] == 0:
        return np.full_like(dx, max_range)
    cx, cy, cr = circles[:, 0], circles[:, 1], circles[:, 2]
    ocx = cx[None, :] - px
    ocy = cy[None, :] - py
    tca = ocx * dx[:, None] + ocy * dy[:, None]
    d2 = (ocx ** 2 + ocy ** 2) - tca ** 2
    r2 = cr[None, :] ** 2
    hit = d2 <= r2
    with np.errstate(invalid="ignore"):
        thc = np.sqrt(np.where(hit, r2 - d2, 0.0))
    t0 = tca - thc
    t1 = tca + thc
    t = np.where(t0 >= 0.0, t0, t1)
    t = np.where(hit & (t >= 0.0), t, np.inf)
    return np.minimum(np.min(t, axis=1), max_range)


def _distance_to_segments(px, py, segs):
    if segs.shape[0] == 0:
        return np.inf
    ax, ay, bx, by = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    ex, ey = bx - ax, by - ay
    L2 = ex ** 2 + ey ** 2
    L2 = np.where(L2 < 1e-12, 1e-12, L2)
    u = np.clip(((px - ax) * ex + (py - ay) * ey) / L2, 0.0, 1.0)
    qx, qy = ax + u * ex, ay + u * ey
    return float(np.min(np.hypot(px - qx, py - qy)))


# ---------------------------------------------------------------------------
# The fast surrogate environment
# ---------------------------------------------------------------------------

class FastNavEnv(gym.Env):
    """Headless 2D kinematic navigation environment.

    This is the TRAINING environment. It contains no physics engine, no ROS and
    no rendering, and it steps in the order of thousands of transitions per
    second on a single CPU core inside the course VM. A PPO policy of the size
    used in Lab 9 reaches competent navigation in roughly ten to fifteen minutes.

    It deliberately shares nothing with Gazebo except the observation contract.
    The difference between the two is not a defect to be minimised, it is the
    simulation gap that Lab 10 asks students to measure.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        scenario: Optional[Scenario] = None,
        scenario_factory: Optional[Callable[[int], "Scenario"]] = None,
        reward: str = "dense_progress",
        obs_spec: ObsSpec = OBS,
        robot: RobotSpec = ROBOT,
        lidar_noise_std: float = 0.01,
        randomise_start: bool = True,
        domain_randomisation: bool = False,
    ):
        super().__init__()
        # When a scenario_factory is given, reset(seed=s) regenerates the arena
        # from s. Without it, every evaluation seed shares one arena and the
        # only thing varying is sensor noise, which is not a generalisation
        # test. Grading configurations must always supply a factory.
        self.scenario_factory = scenario_factory
        self.scenario = scenario or default_training_scenario()
        if reward not in REWARDS:
            raise ValueError(f"Unknown reward '{reward}'. Options: {sorted(REWARDS)}")
        self.reward_name = reward
        self.reward_fn = REWARDS[reward]
        self.obs_spec = obs_spec
        self.robot = robot
        self.lidar_noise_std = lidar_noise_std
        self.randomise_start = randomise_start
        self.domain_randomisation = domain_randomisation

        self.observation_space = observation_space(obs_spec)
        self.action_space = action_space()

        self._angles = np.linspace(-math.pi, math.pi, obs_spec.n_beams, endpoint=False).astype(np.float32)
        self._reset_internal_state()

    # -- internal helpers ---------------------------------------------------

    def _reset_internal_state(self):
        self.x, self.y, self.theta = self.scenario.start_pose
        self.v = 0.0
        self.w = 0.0
        self.goal_index = 0
        self.step_index = 0
        self.path_length = 0.0
        self.collisions = 0
        self.min_clearance = float("inf")
        self._dynamic = np.zeros((0, 5), dtype=np.float32)
        self._prev_goal_distance = 0.0

    @property
    def goal(self) -> tuple[float, float]:
        return self.scenario.goals[min(self.goal_index, len(self.scenario.goals) - 1)]

    def _segments(self) -> np.ndarray:
        return np.asarray(self.scenario.walls, dtype=np.float32).reshape(-1, 4)

    def _circles(self) -> np.ndarray:
        static = np.asarray(self.scenario.static_circles, dtype=np.float32).reshape(-1, 3)
        if self._dynamic.shape[0] == 0:
            return static
        moving = self._dynamic[:, [0, 1, 4]]
        return np.vstack([static, moving]) if static.shape[0] else moving

    def _raycast(self) -> np.ndarray:
        world_angles = self._angles + self.theta
        dx = np.cos(world_angles).astype(np.float32)
        dy = np.sin(world_angles).astype(np.float32)
        r_seg = _ray_hits_segments(self.x, self.y, dx, dy, self._segments(), self.obs_spec.lidar_max_range)
        r_cir = _ray_hits_circles(self.x, self.y, dx, dy, self._circles(), self.obs_spec.lidar_max_range)
        ranges = np.minimum(r_seg, r_cir)
        if self.lidar_noise_std > 0.0:
            ranges = ranges + self.np_random.normal(0.0, self.lidar_noise_std, ranges.shape)
        return np.clip(ranges, self.obs_spec.lidar_min_range, self.obs_spec.lidar_max_range).astype(np.float32)

    def _clearance(self) -> float:
        d_seg = _distance_to_segments(self.x, self.y, self._segments())
        circles = self._circles()
        if circles.shape[0]:
            d_cir = float(np.min(np.hypot(circles[:, 0] - self.x, circles[:, 1] - self.y) - circles[:, 2]))
        else:
            d_cir = float("inf")
        return min(d_seg, d_cir) - self.robot.footprint_radius

    def _goal_relative(self) -> tuple[float, float]:
        gx, gy = self.goal
        dx, dy = gx - self.x, gy - self.y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx) - self.theta
        angle = math.atan2(math.sin(angle), math.cos(angle))
        return distance, angle

    def _observe(self) -> np.ndarray:
        beams = self._raycast()
        distance, angle = self._goal_relative()
        return build_observation(beams, distance, angle, self.v, self.w, self.obs_spec, self.robot)

    def _spawn_dynamic(self):
        specs = self.scenario.dynamic_circles
        if not specs:
            self._dynamic = np.zeros((0, 5), dtype=np.float32)
            return
        arr = []
        for (cx, cy, speed, heading, radius) in specs:
            arr.append([cx, cy, speed, heading, radius])
        self._dynamic = np.asarray(arr, dtype=np.float32)

    def _advance_dynamic(self, dt: float):
        if self._dynamic.shape[0] == 0:
            return
        vx = self._dynamic[:, 2] * np.cos(self._dynamic[:, 3])
        vy = self._dynamic[:, 2] * np.sin(self._dynamic[:, 3])
        self._dynamic[:, 0] += vx * dt
        self._dynamic[:, 1] += vy * dt
        w, h = self.scenario.bounds
        margin = self._dynamic[:, 4] + 0.1
        out = (self._dynamic[:, 0] < margin) | (self._dynamic[:, 0] > w - margin) | \
              (self._dynamic[:, 1] < margin) | (self._dynamic[:, 1] > h - margin)
        self._dynamic[out, 3] += math.pi
        self._dynamic[:, 0] = np.clip(self._dynamic[:, 0], margin, w - margin)
        self._dynamic[:, 1] = np.clip(self._dynamic[:, 1], margin, h - margin)

    # -- Gymnasium API ------------------------------------------------------

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        """Start a new episode and return (observation, info).

        Everything this needs is already available: _reset_internal_state,
        _spawn_dynamic, _clearance, _goal_relative and _observe are given.
        """
        # TODO 1: call super().reset(seed=seed) first. That is what seeds
        #         self.np_random, and every random draw below must come from
        #         self.np_random rather than np.random so an episode is
        #         reproducible from its seed.
        #
        # TODO 2: if self.scenario_factory is not None, rebuild self.scenario
        #         from it. With an explicit seed use self.scenario_factory(seed),
        #         which is how the evaluation harness pins one exact arena. With
        #         seed None draw a fresh arena:
        #         self.scenario_factory(int(self.np_random.integers(0, 1_000_000))).
        #         Omit that second branch and a training run sees one arena per
        #         parallel environment, because SB3 seeds each one once and never
        #         again, so the policy overfits while appearing to learn.
        #
        # TODO 3: clear the per-episode counters with self._reset_internal_state(),
        #         then place the moving obstacles with self._spawn_dynamic().
        #         Do these after TODO 2, since the start pose comes from the
        #         scenario you just chose.
        #
        # TODO 4: if self.randomise_start, try up to 50 candidate poses and keep
        #         the first that is clear. With (w, h) = self.scenario.bounds,
        #         draw x uniform in (0.6, w - 0.6), y uniform in (0.6, h - 0.6),
        #         theta uniform in (-pi, pi), assign them to self.x, self.y and
        #         self.theta, and accept when self._clearance() > 0.25. If all 50
        #         fail, fall back to self.scenario.start_pose. Accepting an
        #         unchecked pose spawns the robot inside a pillar, which reads as
        #         a collision on step 1 and poisons the episode statistics.
        #
        # TODO 5: if self.domain_randomisation, redraw self.lidar_noise_std
        #         uniformly in (0.005, 0.04).
        #
        # TODO 6: set self._prev_goal_distance to self._goal_relative()[0], then
        #         return self._observe() and the info dict
        #         {"scenario": self.scenario.name, "reward_fn": self.reward_name}.
        #         Leaving _prev_goal_distance at 0.0 makes the first dense reward
        #         read as if the robot had just lost the whole distance to the
        #         goal.
        raise NotImplementedError("TODO: implement reset")

    def step(self, action):
        dt = self.robot.control_period
        self.v, self.w = scale_action(action, self.robot)

        # TODO 7: integrate the unicycle model over one control period, using
        #         the heading from BEFORE this step:
        #
        #           self.x += self.v * cos(self.theta) * dt
        #           self.y += self.v * sin(self.theta) * dt
        #           self.theta = wrap(self.theta + self.w * dt)
        #
        #         Wrap with math.atan2(math.sin(a), math.cos(a)) so the heading
        #         stays in [-pi, pi]. Update theta before x and y and every
        #         command curves, which looks like a tuning problem and is not.
        #
        # TODO 8: accumulate self.path_length by abs(self.v) * dt, move the
        #         dynamic obstacles with self._advance_dynamic(dt), and increment
        #         self.step_index. Use the absolute speed, so reversing counts as
        #         distance travelled. Forgetting step_index means the truncation
        #         below never fires and info["time_s"] stays 0.
        raise NotImplementedError("TODO: implement the pose integration in step")

        clearance = self._clearance()
        self.min_clearance = min(self.min_clearance, clearance)
        collided = clearance <= 0.0
        if collided:
            self.collisions += 1

        distance, _ = self._goal_relative()
        reached = distance <= self.obs_spec.goal_tolerance
        advanced = False
        if reached and self.goal_index < len(self.scenario.goals) - 1:
            self.goal_index += 1
            advanced = True
            reached = False

        beams = self._raycast()
        terms = RewardTerms(
            previous_goal_distance=self._prev_goal_distance,
            goal_distance=distance,
            min_beam=float(np.min(beams)),
            linear_velocity=self.v,
            angular_velocity=self.w,
            reached_goal=reached,
            collided=collided,
            step_index=self.step_index,
        )
        reward = self.reward_fn(terms)
        if advanced:
            reward += 30.0
        self._prev_goal_distance = self._goal_relative()[0] if advanced else distance

        # TODO 9: set `terminated` and `truncated`. The episode terminates when
        #         it ends on its own terms, which here is `reached or collided`.
        #         It truncates when self.step_index >= self.scenario.max_episode_steps.
        #         Keep them separate. Gymnasium bootstraps the value function
        #         through a truncated episode and not through a terminated one,
        #         so reporting a timeout as terminated teaches the critic that
        #         running out of time is a real end state.
        #
        # TODO 10: set `reason` to one of "goal_reached", "collision", "timeout"
        #          or "running", checked in that order. A step that both reaches
        #          the goal and hits the step limit is a success. arc_eval writes
        #          these strings into the results file, so they must match.
        raise NotImplementedError("TODO: implement the termination logic in step")

        # TODO 11: build `info` with exactly these keys, which are the ones
        #          arc_eval.runner.REQUIRED_INFO_KEYS checks for on the last step
        #          of every episode:
        #
        #            success              bool, `reached`
        #            collisions           int, self.collisions
        #            min_clearance_m      float, self.min_clearance
        #            path_length_m        float, self.path_length
        #            time_s               float, self.step_index * dt
        #            checkpoints_reached  int, self.goal_index + (1 if reached else 0)
        #            recovery_events      0
        #            interventions        0
        #            termination_reason   reason
        #
        #          time_s is simulated time, not wall clock. goal_index counts
        #          the checkpoints already passed, so the final goal only counts
        #          once `reached` is true. recovery_events and interventions stay
        #          0 in the surrogate; the ROS environment fills them in. Cast to
        #          plain bool, int and float, because the runner serialises info
        #          to JSON and NumPy scalars do not.
        raise NotImplementedError("TODO: implement the info dictionary in step")

        return self._observe(), float(reward), terminated, truncated, info


def default_training_scenario(seed: int = 0) -> Scenario:
    """A modest room with pillars, used as the Lab 8 and Lab 9 default."""
    rng = np.random.default_rng(seed)
    sc = Scenario.boxed_room(10.0, 10.0)
    sc.name = f"training_room_{seed:02d}"
    sc.static_circles = [
        (float(rng.uniform(2.0, 8.0)), float(rng.uniform(2.0, 8.0)), float(rng.uniform(0.25, 0.6)))
        for _ in range(6)
    ]
    sc.dynamic_circles = [(5.0, 2.0, 0.30, 1.2, 0.25), (3.0, 7.0, 0.25, -0.7, 0.25)]
    sc.goals = [(8.5, 8.5)]
    sc.start_pose = (1.0, 1.0, 0.0)
    sc.max_episode_steps = 600
    return sc


if __name__ == "__main__":
    env = FastNavEnv()
    obs, info = env.reset(seed=1)
    print(f"observation size: {obs.shape[0]} (expected {OBS.size})")
    total = 0.0
    for _ in range(200):
        obs, r, term, trunc, info = env.step(env.action_space.sample())
        total += r
        if term or trunc:
            break
    print(f"random rollout reward {total:.2f}, ended as {info['termination_reason']}")

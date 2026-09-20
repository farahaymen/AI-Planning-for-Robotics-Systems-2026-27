---
title: "Lab 8: Building a Reinforcement Learning Environment"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 8: Building a Reinforcement Learning Environment

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1. The training environment is a Gymnasium `Env` with NumPy dynamics. It has no physics engine and no renderer, so it runs in every graphics tier.
**Python packages** `gymnasium`, `numpy`, `stable_baselines3`, `rclpy`
**Course modules** `arc_rl`, in the repository root. It is a plain Python package, not a ROS package: it carries a `COLCON_IGNORE` and no `package.xml`, so `colcon` skips it and there is nothing for `ros2 run` to find. Import it from the repository root, or with `PYTHONPATH` set to it.

**Prerequisites**

Labs 1 to 7. You should be comfortable with the MDP formulation from your
Reinforcement Learning module. Read `arc_rl/nav_core.py` before the session.

---

## Before the session

### Why this lab matters

Everything you have built so far was specified. You told the planner what a good
path was, the controller what a good trajectory was, and the behaviour tree what
to do when things failed.

Today the specification moves. Instead of describing behaviour you describe what
counts as good, and an optimisation process searches for a policy that produces
it. That sounds like less work. It is not; it is the same work relocated into the
environment definition, and the environment is where reinforcement learning
projects usually go wrong.

No training happens today. Today you build the thing that training will happen
in, and you check it carefully, because a subtle defect in an environment does
not produce an error. It produces a policy that learns something other than what
you intended, and you find out three hours into a training run.

### The architecture, and where Gymnasium sits

Two pieces of software are easy to confuse.

**Gazebo is the simulator.** It computes physics and produces sensor data.

**Gymnasium is the interface.** It is a convention for how a learning algorithm
talks to any environment: `reset` returns an observation, `step` takes an action
and returns an observation, a reward, a terminated flag, a truncated flag and an
info dictionary. That is the entire specification, and its value is that every
RL library speaks it.

```
   PPO or SAC  <-->  Gymnasium interface  <-->  the environment
                     reset / step
```

What sits behind the interface is your choice, and that choice is the most
consequential decision in this part of the course.

### Two environments, one contract

Reinforcement learning in Gazebo does not work inside this virtual machine, and
it is worth being precise about why rather than treating it as folklore.

Gazebo runs below real time in a VM. A Gazebo backed environment collects roughly
tens of transitions per second. PPO needs hundreds of thousands. At 50
transitions per second, 200,000 steps is over an hour, and that is before
anything goes wrong.

The measured alternative: the surrogate environment in `arc_rl/nav_core.py` runs
at about **3,800 steps per second on one CPU core**, and a full PPO training run
of 200,000 steps completes in **under two minutes** on a single thread. That is
the difference between a policy you can train in a laboratory session and one you
cannot.

So the course uses two environments that share one frozen interface:

![Two environments, one frozen contract. Throughput figures are measured on the course VM.](docs/figures/diagram_two_environments.png){width=100%}

Both import the same `build_observation`. A policy trained in the surrogate loads
and runs against the ROS stack with no change to the network shape or the meaning
of any observation element.

The two environments differ in physics, and that difference is not a defect to be
minimised. It is the simulation gap, and Lab 10 asks you to measure it. Having
experienced a policy degrade between two simulators is a much better preparation
for the sim-to-real problem than reading about it.

### The MDP, made concrete

**Observation, 29 numbers.** Twenty four LiDAR beams, then goal distance, the
sine and cosine of the goal angle, and the current linear and angular velocity.
All scaled to roughly the interval from minus one to one.

Three decisions in that are worth defending.

*Twenty four beams, not 360.* The policy does not need angular resolution; it
needs to know where the free space is. Twenty four beams keeps the network small
enough to train on CPU.

*Downsampled by taking the minimum of each sector, not the mean.* A thin table
leg that appears in one raw beam must survive downsampling. Averaging erases it,
and the policy learns to drive through furniture.

*Sine and cosine of the goal angle rather than the angle.* A raw angle jumps from
plus pi to minus pi when the goal passes behind the robot. The network sees a
huge input change for an infinitesimal physical change, and the resulting policy
behaves strangely in exactly that configuration.

**Action, 2 numbers.** Normalised linear and angular velocity, scaled by
`scale_action` to the robot's real limits. Keeping the network's output range
fixed and doing the scaling outside means changing the robot's speed limit does
not invalidate a trained policy.

**Termination and truncation are different things.** Terminated means the episode
ended for a reason inside the MDP: the goal was reached, or the robot collided.
Truncated means the episode was cut off from outside: the step limit expired.

The distinction is not bookkeeping. Value estimation bootstraps through a
truncation and does not bootstrap through a termination, because a terminated
state genuinely has no future and a truncated one does. Conflating them
systematically biases the value function, and it is one of the most common
silent bugs in custom environments.

### Reward design

The reward function is where your intentions enter, and it is where they get
misread.

Two are supplied in `nav_core.py`.

`reward_dense_progress` rewards reducing the distance to the goal on every step,
with a small time penalty, a proximity penalty and terminal bonuses. Dense
rewards train quickly because there is a gradient everywhere.

`reward_sparse_safe` gives almost nothing until the goal, with a heavier safety
weighting. Sparse rewards are harder to optimise and specify the task more
honestly.

Next week you will train both and compare them, and one of them fails in an
instructive way. Do not read ahead; predict which and why, and write the
prediction in your exit task.

---

> ### Engineering Practice: an environment is software, and it needs tests
>
> A reinforcement learning environment is the least tested code in most projects
> and the code where a bug is hardest to notice, because the symptom is a policy
> that learns the wrong thing rather than a stack trace.
>
> The failure modes are specific and recurring. An observation that leaks
> information the robot could not have, so the policy performs impossibly well in
> training and fails on the robot. A reward computed from state after the step
> but compared against a distance from before it, producing a free reward for
> doing nothing. Termination and truncation conflated. A reset that does not
> fully reset, so episode 400 begins in a state episode 399 left behind.
>
> The defences are ordinary software engineering. `check_env` from
> Stable-Baselines3 catches interface violations. A random policy rollout catches
> crashes and shape errors. A determinism test, running the same seed twice and
> comparing, catches hidden state. A rollout with a hand written policy that
> should obviously succeed catches reward sign errors.
>
> The determinism test in particular is worth adopting as a habit. It takes four
> lines, it catches an entire class of bug, and without it "my training run is
> not reproducible" is a mystery rather than a failing test.

---

### Pre-lab quiz

Five questions covering the difference between the simulator and the interface,
why training does not happen in Gazebo, why the goal angle is encoded as a sine
and cosine pair, the difference between terminated and truncated, and what the
minimum-per-sector downsampling protects against.

---

## In the session

### Stage 0: health check (5 minutes)

```
course-check
python3 -c "import gymnasium, stable_baselines3; print('ok')"
```

### Stage 1: demonstration (10 minutes)

Your demonstrator will run a random policy in the surrogate and then a trained
policy, both rendered as trajectory plots, and will show the throughput figure
that makes the two-environment design necessary.

### Exercise 8.1: complete the environment (35 minutes)

`starters/lab08/nav_core_skeleton.py` is the file you edit. It is
`arc_rl/nav_core.py` with the geometry, the raycasting and the observation
assembly left complete and twelve TODOs opened up across `reset`, the pose
integration in `step`, the termination logic and the info dictionary.

Work against the tests. There are 14, and all 14 fail on the untouched skeleton.
Eleven of the twelve TODOs are covered by at least one test. The exception is
TODO 5, the domain randomisation branch in `reset`, which no test constructs;
Lab 10 is where that one gets exercised, so write it carefully rather than
waiting for a red test to tell you it is wrong:

```
cd ~/arc_ws/src/arc-course
ARC_NAV_CORE=nav_core_skeleton python3 -m pytest starters/lab08 -q
```

Without the variable the same tests run against the reference implementation in
`arc_rl/nav_core.py`, which is a quick way to confirm the suite itself is
healthy before you start:

```
python3 -m pytest starters/lab08 -q
```

**Code 8.1: The step method you are completing**

```python
def step(self, action):
    dt = self.robot.control_period
    self.v, self.w = scale_action(action, self.robot)

    # Unicycle integration. The surrogate has no physics engine on purpose:
    # no wheel slip, no motor dynamics, no controller lag. That is what makes
    # it fast, and it is also precisely the gap Lab 10 asks you to measure.
    self.x += self.v * math.cos(self.theta) * dt
    self.y += self.v * math.sin(self.theta) * dt
    self.theta = math.atan2(math.sin(self.theta + self.w * dt),
                            math.cos(self.theta + self.w * dt))
    self.path_length += abs(self.v) * dt
    self._advance_dynamic(dt)
    self.step_index += 1

    clearance = self._clearance()
    self.min_clearance = min(self.min_clearance, clearance)
    collided = clearance <= 0.0

    distance, _ = self._goal_relative()
    reached = distance <= self.obs_spec.goal_tolerance

    terms = RewardTerms(
        previous_goal_distance=self._prev_goal_distance,
        goal_distance=distance,
        # ... remaining fields
    )
    reward = self.reward_fn(terms)
    self._prev_goal_distance = distance

    # Terminated: the episode ended inside the MDP.
    # Truncated: the episode was cut off from outside.
    # Value estimation bootstraps through truncation and not through
    # termination, so returning the wrong one biases the value function
    # silently for the whole training run.
    terminated = bool(reached or collided)
    truncated = bool(self.step_index >= self.scenario.max_episode_steps)

    return self._observe(), float(reward), terminated, truncated, self._info()
```

The `_prev_goal_distance` bookkeeping is the one to be careful about. The dense
reward compares the distance before the step against the distance after it. If
you update `_prev_goal_distance` before computing the reward, every step scores
zero progress and the policy learns nothing while training runs happily to
completion.

### Exercise 8.2: check it four ways (20 minutes)

Passing your own tests is not enough. Run the four checks from the engineering
practice box.

**Code 8.2: The four checks, in the order that finds bugs fastest**

```python
import numpy as np
from gymnasium.utils.env_checker import check_env as gym_check
from stable_baselines3.common.env_checker import check_env as sb3_check

from arc_rl.nav_core import FastNavEnv, OBS
from arc_rl.arenas import training_scenario

env = FastNavEnv(scenario_factory=training_scenario)

# 1. Interface conformance. Catches spaces, dtypes and signatures.
gym_check(env, skip_render_check=True)
sb3_check(env)
print("interface: PASS")

# 2. Shape. The contract says 29; assert it rather than trusting it.
obs, info = env.reset(seed=0)
assert obs.shape == (OBS.size,), f"expected {OBS.size}, got {obs.shape}"
assert env.observation_space.contains(obs), "observation outside its own space"
print("shape: PASS")

# 3. Determinism. Four lines, and it catches every hidden-state bug.
def rollout(seed):
    e = FastNavEnv(scenario_factory=training_scenario)
    e.reset(seed=seed)
    rng, total = np.random.default_rng(7), 0.0
    for _ in range(300):
        _, r, term, trunc, _ = e.step(rng.uniform(-1, 1, 2).astype(np.float32))
        total += r
        if term or trunc:
            break
    return total

assert rollout(11) == rollout(11), "same seed, different result: hidden state"
print("determinism: PASS")

# 4. Sanity. A policy that drives straight at the goal must do better than one
#    that acts at random. If it does not, the reward sign is wrong.
def straight_at_goal(obs):
    angle = np.arctan2(obs[OBS.n_beams + 1], obs[OBS.n_beams + 2])
    return np.array([0.6, np.clip(angle * 1.5, -1, 1)], dtype=np.float32)
```

The observation space containment check on line 3 is worth keeping permanently.
An observation outside its declared bounds is accepted silently by most
algorithms and quietly breaks normalisation.

Record your results:

| Check | Result | If it failed, what was wrong |
|-------|--------|------------------------------|
| `gymnasium` check_env | | |
| `stable_baselines3` check_env | | |
| Observation shape and containment | | |
| Determinism under a repeated seed | | |
| Goal-seeking beats random | | |

### Exercise 8.3: break it deliberately (15 minutes)

Introduce each of these, observe the symptom, then revert. This is a fifteen
minute investment that will save you an afternoon next week.

| Injected bug | Predict the symptom | Observed symptom |
|--------------|--------------------|------------------|
| Update `_prev_goal_distance` before computing the reward | | |
| Return `terminated=True` on the step limit instead of `truncated` | | |
| Downsample the scan with `mean` instead of `min` | | |
| Feed the raw goal angle instead of sine and cosine | | |
| Omit the reset of `path_length` in `reset` | | |

Notice that none of these raises an exception. Every one produces a working
environment that trains a policy which is not the policy you wanted. That is the
point of the exercise, and it is why the checks in 8.2 exist.

**[SCREENSHOT PLACEHOLDER]**
Trajectory plots from a random policy and from the hand written goal-seeking
policy on the same arena.
*Instructor note: plot the arena walls, obstacles, goal and trajectory in one
figure per policy, side by side at the same scale. The difference should be
obvious at a glance, since this is the plot students compare their own against.*

### Exercise 8.4: read the ROS evaluation adapter (10 minutes)

You do not build this one; it is supplied. Read it and answer two questions.

```
less arc_eval/ros_nav2_env.py
```

`Nav2EvalEnv` is how `arc_eval.runner` benchmarks the Nav2 stack through the
same code path it uses for a trained policy. Note three things as you read. It
subscribes to `/scan` with best effort QoS, the same reliability decision Lab 1
spends time on and the same one your surrogate does not have to make. It imports
`ROBOT` from `arc_rl.nav_core`, so the robot constants are shared with the
surrogate rather than duplicated. And its `step` is not a control step at all:
the first call sends one `NavigateToPose` goal and every later call spins the
executor and polls for completion while accumulating metrics, which is why the
policy the runner passes it is a stub that ignores its observation and returns
`None`.

That last point is the exercise. `FastNavEnv` steps a policy on an observation
vector at 20 Hz; `Nav2EvalEnv` hands one goal to a behaviour tree and waits. The
two satisfy the same runner and report the same metrics, and they do not share
the observation contract.

In your exit task, answer both of these. First: name three ways a Gazebo backed
environment differs physically from the surrogate, and predict for each whether
a policy trained in the surrogate would do better or worse because of it.
Second: `Nav2EvalEnv` cannot be used to run your trained policy against Gazebo,
because it never gives the policy an observation to act on. Say what a
`RosNavEnv` would have to do differently, in terms of the `ObsSpec` and
`build_observation` you have just been working with, to close that gap.

No such environment exists in this repository yet. The `arc_rl/ros_nav_env.py`
named in the `arc_rl/nav_core.py` docstring is planned work, not a file you can
open, and Lab 10 says where that leaves Exercise 10.1. Building it against the
contract in `nav_core.py` is a well-scoped Project 2 topic.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your completed environment with all tests passing.
2. Your four-check table.
3. Your bug injection table with predicted and observed symptoms.
4. Your three physical differences and predictions from Exercise 8.4.
5. Your prediction about which supplied reward function will perform worse next
   week, and why. This is marked on the reasoning, not on being right.

---

## Troubleshooting

**`check_env` complains about the observation dtype.** The space is `float32` and
NumPy promotes to `float64` on almost any arithmetic. Cast explicitly on return.

**`check_env` complains the observation is outside the space.** Some element
exceeds its declared bounds, usually a velocity when the action was not clipped
or a beam when the range clip was skipped.

**Rewards are all zero.** The `_prev_goal_distance` ordering bug from Exercise
8.1.

**Episodes never end.** Termination is being computed from a stale distance, or
the goal tolerance is smaller than the distance the robot moves in one step.
At 0.5 m/s and a 0.05 s period that is 2.5 cm, so a tolerance below that can be
stepped over entirely.

**The same seed gives different results.** State that survives `reset`. The usual
culprits are accumulators not being cleared and the dynamic obstacles not being
respawned.

**Training in the surrogate is slower than 1,000 steps per second.** Check that
you have not added a print, a plot, or a `time.sleep` to the step function. This
sounds obvious and it is the answer about half the time.

---

## Connection to Lab 9

You have an environment that is fast, deterministic, checked and correct, and you
have not learned anything with it yet.

Next week you train two policies with the two reward functions and benchmark both
against the classical controller through the same evaluation harness you used in
Lab 6. The classical floor on held-out arenas is 40 percent success, so there is
a concrete number to beat.

One of the two reward functions produces a policy that fails completely, in a way
that is entirely explained by arithmetic you can do on the reward constants before
training starts. Bring your prediction from the exit task.

---

## References

**Textbooks**

Sutton, R. S. and Barto, A. G. (2018). *Reinforcement Learning: An Introduction*,
2nd edition. MIT Press, freely available online. Chapter 3 for the MDP
formulation and chapter 17.4 on the difference between episodic termination and
time limits, which is the terminated versus truncated distinction.

**Official documentation**

Gymnasium: https://gymnasium.farama.org
The custom environment tutorial and the API reference. The migration notes from
Gym are worth reading, because a great deal of material you will find online uses
the old four-value step signature.

Stable-Baselines3 custom environments:
https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html
The `check_env` documentation lists exactly what it verifies.

**Papers**

Amodei, D. et al. (2016). Concrete Problems in AI Safety. arXiv:1606.06565.
Sections 2 and 3 on reward hacking and side effects are the clearest treatment of
why specifying what you want is harder than it looks, and they are directly
relevant to the reward you write here.

**Repositories**

`DLR-RM/rl-baselines3-zoo` on GitHub. Working hyperparameters for a large number
of environments, and a useful reference for how a training script is normally
structured.

## Further reading

`docs/references.md` has a fuller list under **Lab 8. Building a reinforcement
learning environment**. Start with the Gymnasium paper by Towers et al.: it
explains why `reset`, `step`, `observation_space` and `action_space` are shaped
the way they are, which is exactly the contract you spent this lab
implementing. If you want to see the observation contract argument made at full
scale, the Arena 4.0 paper in the same section describes a ROS 2 platform where
the training environment and the real navigation stack share one interface.

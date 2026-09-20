---
title: "Lab 9: Deep Reinforcement Learning for Navigation"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 9: Deep Reinforcement Learning for Navigation

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1. Training is CPU only and runs in every graphics tier.
**Python packages** `stable_baselines3`, `torch`, `gymnasium`, `tensorboard`
**Course modules** `arc_rl`, `arc_eval`, in the repository root. Both are plain Python packages, not ROS packages: each carries a `COLCON_IGNORE` and no `package.xml`, so `colcon` skips them and `ros2 run` will not find them. Run them as modules from the repository root, for example `python3 -m arc_eval.runner`.

**Prerequisites**

Lab 8, with your environment passing all four checks. Bring your prediction about
which reward function will perform worse.

---

## Before the session

### Why this lab matters

You are going to train a policy, and it is going to be worse than you expect.
That is the lab.

Reinforcement learning is presented, in most courses and nearly all
demonstrations, at the point where it works. What is left out is the two weeks
before that, which are spent discovering that the agent found a way to satisfy
your reward function without doing the task. Today you get a compressed version:
two reward functions, one of which produces a competent navigator and one of
which produces a policy that has learned, correctly and rationally, to do
nothing.

Both were written by the same person, in the same file, with the same intent.
The difference is arithmetic you can do before training starts.

### The two algorithms

**PPO** is the default choice and the one you will use. It is an on-policy
method: it collects a batch of experience with the current policy, improves the
policy, and discards the batch. The clipped objective prevents any single update
from moving the policy far, which is what makes it stable enough to use without
extensive tuning. It is not sample efficient, and it does not need to be here,
because samples are nearly free in the surrogate.

**SAC** is off-policy and keeps a replay buffer, so it reuses experience many
times and needs perhaps a tenth as many environment steps. It also maximises
entropy alongside reward, which produces deliberately stochastic policies that
explore well. The costs are more hyperparameters, more memory, and more ways to
diverge quietly.

The rule of thumb worth taking away: **when samples are cheap, use PPO; when
samples are expensive, the extra complexity of SAC starts paying for itself.** In
the surrogate samples are cheap. On a physical robot they are not, which is why
sample efficiency dominates the sim-to-real literature.

**DQN** is worth knowing for context and is not appropriate here, because it
handles discrete actions only and this is a continuous control problem. It would
require discretising velocity into bins, which throws away resolution for no
benefit.

### The compute budget, measured

These are measured figures from the course virtual machine, on a single PyTorch
thread, not estimates.

| Quantity | Measured |
|----------|----------|
| Surrogate throughput | about 3,800 steps/s, one core |
| PPO training throughput | about 1,750 steps/s including all overhead |
| 200,000 step training run | **1.9 minutes** |
| Policy network | MLP, two hidden layers of 64 units |
| Observation | 29 values |

Two training runs therefore cost under four minutes, which is what makes a real
comparison possible inside a two hour session. The gap between 3,800 and 1,750
is PPO's own work: forward passes, advantage estimation and gradient steps.

### The floor you have to beat

The classical `gap_follow` controller from `arc_rl/baselines.py` scores, over 30
held-out arenas:

| Metric | Classical baseline |
|--------|--------------------|
| Success rate | 40.0 % (95% CI 24.6 to 57.7) |
| Collision free rate | 40.0 % |
| Time to goal | 13.61 ± 5.04 s |
| Path length | 5.60 ± 2.20 m |
| Collisions per episode | 0.60 ± 0.50 |
| Minimum clearance | 0.066 ± 0.126 m |

Note the confidence interval. Over 30 episodes, a 40 percent success rate is
consistent with anything between 25 and 58 percent. A learned policy scoring 45
percent has not beaten this, and reporting that it has is the single most common
error in student RL reports.

### Reward hacking is not exotic

The phrase suggests something adversarial. In practice it is mundane: the agent
maximises what you wrote rather than what you meant, and what you wrote had a
cheaper solution than the one you had in mind.

Before the session, do this arithmetic for `reward_sparse_safe`, whose constants
are a per-step penalty of 0.02, a collision penalty of 100, a goal bonus of 100,
and a proximity penalty near obstacles.

Consider a policy that stops moving and stays still for the full 900 step episode.
Its return is about **minus 18**.

Consider a policy that attempts the goal. If it succeeds it gets about **plus
100**. If it collides it gets about **minus 100**.

Attempting is better than freezing only when the probability of success exceeds
roughly 0.41. At the start of training, with a randomly initialised network, the
probability of success is close to zero.

So the gradient points towards freezing, and once the policy freezes it collects
no data about what succeeding would look like. Write down what you expect to
observe before you run anything.

![The freezing failure is predictable before training starts.](docs/figures/lab09_reward_arithmetic.png){width=78%}


---

> ### Research Frontier: what the literature does not report
>
> Learned navigation policies have been published for a decade and almost none
> are deployed. Understanding why is more useful than reading another paper
> reporting a higher success rate.
>
> **Sample efficiency.** Policies commonly need millions of steps. In simulation
> that is hours. On a robot it is impossible, so essentially all of this work
> trains in simulation, which makes the transfer problem unavoidable rather than
> optional.
>
> **Generalisation.** A policy that scores well on the arenas it trained on often
> collapses on a layout drawn from a different distribution. You will observe
> exactly this in Lab 10. Papers that report performance only on the training
> distribution are reporting something close to nothing, and once you notice how
> many do, you read the field differently.
>
> **Verification.** You can inspect a costmap and a global plan and say why the
> robot is going where it is going. You cannot do that with a policy network, and
> a safety case that says "it worked in ten thousand simulated episodes" is not
> the same kind of argument as one bounded by a geometric constraint. This, more
> than performance, is why deployed systems keep a classical stack and add
> learning at the edges.
>
> **Reward specification.** The failure you will see today is not a toy. It is
> the same problem that appears throughout applied RL, and the usual industrial
> response is not a cleverer reward but a constrained formulation or a learned
> component wrapped in classical guarantees.
>
> When you read a DRL navigation paper, three questions separate the useful ones:
> was evaluation on held-out layouts, how many seeds, and what happens when the
> policy is wrong.

---

### Pre-lab quiz

Five questions covering on-policy versus off-policy, when SAC's complexity is
worth paying for, why DQN is inappropriate here, what the reward arithmetic above
predicts, and why a 45 percent result does not beat a 40 percent baseline at
n=30.

---

## In the session

### Stage 0: health check (5 minutes)

```
course-check
python3 -c "import torch; print(torch.__version__, torch.get_num_threads())"
```

### Stage 1: demonstration (10 minutes)

Your demonstrator will start a training run and show the TensorBoard curves as
they appear, so you know what healthy and unhealthy learning look like before
your own run starts.

### Exercise 9.1: train both policies (25 minutes)

Start both immediately. They run while you do Exercise 9.2.

```
export PYTHONPATH=.
python3 starters/lab09/train.py --reward dense_progress --steps 200000 &
python3 starters/lab09/train.py --reward sparse_safe    --steps 200000 &
tensorboard --logdir runs --port 6006
```

**Code 9.1: The parts of the training script that are decisions, not defaults**

```python
model = PPO(
    "MlpPolicy",
    vec,
    # A course constraint rather than a hyperparameter. It trains on CPU, and at
    # 29 observation dimensions a larger network buys nothing measurable.
    policy_kwargs=dict(net_arch=[64, 64]),
    n_steps=512,          # transitions per environment before each update
    batch_size=256,
    gae_lambda=0.95,
    gamma=0.99,           # 0.05 s per step, so a 1/(1-gamma) = 5 s horizon
    learning_rate=3e-4,
    ent_coef=0.005,       # entropy bonus: keeps the policy exploring
    clip_range=0.2,
    n_epochs=10,
    seed=seed,
    # Without this PPO writes no logs, and `tensorboard --logdir runs` starts
    # cleanly and shows an empty page.
    tensorboard_log="runs",
)
```

`gamma` sets the planning horizon. At 0.99 with a 0.05 second control period,
a reward three seconds away is worth 0.99^60 = 0.55 of its face value, a little
over half. The conventional horizon is 1/(1 - gamma) = 100 steps, which at this
control period is 5 seconds. Beyond that the discounting has flattened the
signal and the policy is effectively blind. That is why a purely learned local
controller cannot reason about a route across a building. Lab 10 builds on that
observation.

`ent_coef` is the entropy bonus, and it is what stands between you and a policy
that commits early to a mediocre strategy. Setting it to zero is a good way to
see premature convergence.

Watch three TensorBoard curves:

- `rollout/ep_rew_mean` should rise. Flat from the start means no learning
  signal, which is a reward problem, not a training problem.
- `rollout/ep_len_mean` is the diagnostic one. Rising towards the step limit
  while reward stays flat means the policy is surviving rather than succeeding.
- `train/explained_variance` should climb above zero. Persistently near zero
  means the value function is not learning and advantage estimates are noise.

**[SCREENSHOT PLACEHOLDER]**
TensorBoard showing `ep_rew_mean` and `ep_len_mean` for both reward functions on
the same axes.
*Instructor note: capture at the end of both runs, with the two runs in
contrasting colours and the legend visible. The divergence in `ep_len_mean`, with
the sparse run climbing to the step limit, is the shot that makes the lesson
visible before any evaluation is run.*

### Exercise 9.2: predict, then evaluate (25 minutes)

Fill in the prediction column before either run finishes.

```
python3 -m arc_eval.runner --config arc_eval/configs/gap_follow.yaml
python3 -m arc_eval.runner --config arc_eval/configs/lab09_dense.yaml
python3 -m arc_eval.runner --config arc_eval/configs/lab09_sparse.yaml
python3 -m arc_eval.runner --compare results/gap_follow.json \
    results/ppo_dense_progress.json results/ppo_sparse_safe.json
```

`gap_follow.yaml` is the classical comparator: the follow-the-gap controller you
wrote in Lab 3, on the same environment and the same arena generator. Read it
before you use it. It is configured for seeds 0 to 9, not the held-out 500 to
529, so treat it as a reference point for what the classical approach achieves
on this generator rather than as a like-for-like held-out score. A classical
controller has no training set, so the distinction costs it nothing; it matters
only when you write up the comparison.

Evaluation uses **seeds 500 to 529**, which the arena generator has never
produced during training. This is a held-out set, not a test on the training
distribution.

| Metric | Classical | Predicted dense | Actual dense | Predicted sparse | Actual sparse |
|--------|-----------|-----------------|--------------|------------------|---------------|
| Success rate % | 40.0 | | | | |
| Collision free % | 40.0 | | | | |
| Time to goal (s) | 13.61 | | | | |
| Collisions per episode | 0.60 | | | | |
| Minimum clearance (m) | 0.066 | | | | |

For reference, the results obtained on the reference machine were:

| Metric | Classical | PPO dense | PPO sparse |
|--------|-----------|-----------|------------|
| Success rate % | 40.0 | 40.0 | **0.0** |
| Collision free % | 40.0 | 53.3 | 63.3 |
| Time to goal (s) | 13.61 | 14.46 | n/a |
| Collisions per episode | 0.60 | 0.47 | 0.37 |
| Minimum clearance (m) | 0.066 | 0.144 | **0.360** |

Your numbers will differ, because your seed differs. The pattern should not.

![Measured over 30 held-out arenas. The policy with the best safety metrics never reaches the goal.](docs/figures/lab09_results.png){width=100%}


### Exercise 9.3: explain both results (20 minutes)

Two things happened and neither is what a naive reading expects.

**The dense policy matched the classical baseline on success and beat it on
safety.** Identical success rates, 53 percent collision free against 40, and more
than double the minimum clearance. Both differences in success are far inside the
confidence interval, so the correct statement is that the dense policy did not
improve success and did improve safety.

Write that sentence in your report in that form. Claiming the learned policy
"achieved comparable performance with improved safety" is accurate. Claiming it
outperformed the baseline is not.

**The sparse policy scored zero and had the best safety metrics of all three.**
This is not a contradiction. Confirm the mechanism yourself:

```python
import json, numpy as np
eps = json.load(open("results/ppo_sparse_safe.json"))["episodes"]
print("mean path length over ALL episodes:",
      np.mean([e["path_length_m"] for e in eps]))
print("mean episode duration:", np.mean([e["time_s"] for e in eps]))
print("termination reasons:",
      {r: sum(1 for e in eps if e["termination_reason"] == r) for r in
       {e["termination_reason"] for e in eps}})
```

On the reference run this gave a mean path length of **1.07 m** over episodes
averaging **37.8 seconds**, with **19 of 30 episodes ending in timeout**. The
robot barely moves.

That is exactly what the arithmetic in the pre-lab reading predicted. Freezing
costs about 18 points. Colliding costs 100. Attempting is only worth it once the
success probability exceeds about 0.41, and at initialisation it is near zero, so
the policy learns to stand still and then collects no evidence that anything
better exists.

The safety metrics are excellent because a stationary robot is very safe. This is
worth sitting with, because it generalises: **a metric can be optimised by
abandoning the task, and any metric you report without the success rate beside it
can be gamed this way.**

Now fix it. Pick one change, justify it, retrain for 200,000 steps and re-evaluate:

| Change | Rationale | Success rate after |
|--------|-----------|--------------------|
| Reduce the collision penalty to 30 | Attempting becomes rational sooner | |
| Add a small progress term | Restores a gradient towards the goal | |
| Raise `ent_coef` to 0.02 | Forces continued exploration past the frozen policy | |
| Curriculum: start goals 2 m away | Raises early success probability above the threshold | |

Any of these works. What is assessed is whether your rationale matches what you
observed.

### Exit task (10 minutes)

Commit and push, then submit:

1. Both trained policies and both evaluation JSON files.
2. Your prediction and actuals table, with predictions unedited.
3. Your diagnosis of the sparse failure, with the numbers you measured.
4. Your chosen fix, its rationale, and the result.
5. One sentence stating the correct comparison between the dense policy and the
   classical baseline, phrased so that it is defensible at n=30.

---

## Troubleshooting

**Training crashes on start.** Almost always the environment rather than SB3. Run
the Lab 8 checks again.

**`ep_rew_mean` is flat from the first step.** No learning signal reaching the
policy. Check the reward is non-zero and non-constant by stepping manually.

**`ep_len_mean` rises to the step limit while reward stays flat.** The freezing
failure. Diagnose it with the path length check above rather than by tuning
hyperparameters.

**Training is much slower than 1,700 steps per second.** Check `torch.get_num_threads()`
and that nothing is printing or plotting inside `step`.

**The policy is excellent in training and poor in evaluation.** Check that
evaluation uses held-out seeds, and that `deterministic=True` is set in
`predict`. A policy evaluated stochastically looks worse than it is.

**Results change between runs of the same script.** The seed is not set, or the
arena factory is being reseeded differently. Both training and evaluation must be
reproducible before any comparison means anything.

---

## Connection to Lab 10

You have a learned policy that matches a hand written controller on success and
beats it on clearance, on arenas drawn from the distribution it trained on.

Two questions remain, and they are the ones that decide whether any of this is
useful. What happens on arenas drawn from a harder distribution, which is the
generalisation question. And what happens when the same policy is run against
Gazebo instead of the surrogate, which is the simulation gap.

Next week you measure both. You will also build the hybrid architecture that
keeps the classical controller as the default and hands over to the learned
policy only where the classical one struggles. The supplied hybrid does not work
as intended, and finding out why is the exercise.

---

## References

**Textbooks**

Sutton, R. S. and Barto, A. G. (2018). *Reinforcement Learning: An Introduction*,
2nd edition. MIT Press. Chapter 13 for policy gradient methods.

**Papers**

Schulman, J., Wolski, F., Dhariwal, P., Radford, A. and Klimov, O. (2017).
Proximal Policy Optimization Algorithms. arXiv:1707.06347. Short, and the clipped
objective is explained clearly in section 3.

Haarnoja, T., Zhou, A., Abbeel, P. and Levine, S. (2018). Soft Actor-Critic.
*ICML*. arXiv:1801.01290. The maximum entropy formulation and why it helps
exploration.

Andrychowicz, M. et al. (2021). What Matters In On-Policy Reinforcement Learning?
A Large-Scale Empirical Study. *ICLR*. arXiv:2006.05990. An unusually honest paper
about how much of reported RL performance comes from implementation details
rather than algorithms. Read it before you believe any single reported number,
including your own.

**Official documentation**

Stable-Baselines3: https://stable-baselines3.readthedocs.io
The PPO and SAC pages document every hyperparameter used in Code 9.1.

TensorBoard integration:
https://stable-baselines3.readthedocs.io/en/master/guide/tensorboard.html

**Video**

Pieter Abbeel, Deep Reinforcement Learning lectures, UC Berkeley, on YouTube. The
policy gradient and PPO lectures cover the derivations this lab treats
operationally.

## Further reading

`docs/references.md` has a fuller list under **Lab 9. Training a navigation
policy**. The one to read is Laidlaw, Russell and Dragan on the effective
horizon, which turns the discount factor arithmetic at the top of this lab into
a usable prediction about whether learning will succeed at all. Pair it with
Vasan et al. on sparse rewards, which is the honest counterweight to reward
shaping and directly relevant to the dense against sparse comparison you just
measured.

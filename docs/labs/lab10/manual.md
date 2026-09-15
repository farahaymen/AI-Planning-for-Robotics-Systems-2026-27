---
title: "Lab 10: Hybrid Autonomy, Generalisation and Sim-to-Real"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 10: Hybrid Autonomy, Generalisation and Sim-to-Real

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1 (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2 Jazzy)
**Packages** `arc_rl`, `arc_eval`, `arc_nav`, `stable_baselines3`, `nav2_bringup`

**Prerequisites**

Lab 9, with both trained policies and their evaluation results. This is the last
laboratory before the Grand Challenge, and the decision you reach today is the
one you will act on in your competition entry.

---

## Before the session

### Why this lab matters

You now have two ways to make a robot navigate. One was designed and one was
trained. Today you find out what each is actually good for, and the answer is not
the one either camp advertises.

The lab is built around three measurements: how each system performs on arenas
harder than those it was tuned or trained on, how a policy degrades when moved
from the surrogate to Gazebo, and whether combining the two does better than
either. All three have been run on the reference machine and the numbers are
below. Two of the three are failures, and the failures are the useful part.

Everything here feeds directly into a decision you have to defend in your Project
2 report: which system would you actually deploy, and why.

### Three architectures

**Classical.** SLAM builds a map, AMCL localises on it, a global planner searches
the whole map for a route, and a local controller follows it while avoiding what
the map does not know about. Verifiable, inspectable, and it fails in ways you can
diagnose.

**Learned.** LiDAR in, velocity out. No map, no plan, no explicit state. Reactive,
fast, and with a planning horizon set by the discount factor. At gamma 0.99 and a
0.05 second control period, that horizon is roughly three seconds. A policy like
this cannot represent a route across a building, because rewards from the far side
of a building are discounted to nothing.

That last sentence is the single most important thing in this lab and you should
be able to derive it. A purely learned local policy is not a navigation system. It
is a local controller, and comparing it against a full classical stack is
comparing a component against a system.

**Hybrid.** Keep the classical stack, and substitute the learned policy for the
local controller in the situations where the classical controller does badly. This
is where the applied literature has converged, and the reason is that it keeps the
part of the classical system that is genuinely hard to learn, which is long
horizon planning, while allowing learning where reactive behaviour matters.

Chandra and colleagues (2024) implement exactly this, detecting when the global
plan has been obstructed and switching to an RL planner for that stretch, and
report a 26 percent improvement over either planner alone on a physical robot.
Their switching criterion is a geometric test rather than a learned one, argued
explicitly on the grounds that a learned switch inherits the generalisation
problems of the thing it is supposed to guard.

The supplied `starters/lab10/hybrid.py` implements the same shape. It does not
work, and Exercise 10.3 is finding out why.

### Generalisation: the measurement

The evaluation arenas in Lab 9 came from the training grammar: about 10 metres
square, one or two partitions, five to nine obstacles, one goal. The competition
grammar is harder: 10 to 14 metres, two to four partitions, 8 to 16 obstacles,
and **four checkpoints** that must be visited in sequence.

Measured on the reference machine, 30 unseen arenas from the competition grammar:

| System | Success | Collision free | Checkpoints reached (of 4) | Min clearance |
|--------|---------|----------------|----------------------------|---------------|
| Classical `gap_follow` | 6.7 % | 6.7 % | 0.43 | 0.001 m |
| Learned PPO | 0.0 % | 20.0 % | 0.23 | 0.040 m |
| Hybrid | 6.7 % | 6.7 % | 0.43 | −0.000 m |

Everything collapses. The learned policy that matched the classical baseline at
40 percent on the training grammar reaches 0 percent here, and the classical
reactive controller reaches 6.7.

This is not a defect in the exercise; it is the result. Neither a reactive
controller nor a purely learned local policy is a navigation system, and a
four-checkpoint mission through a partitioned building is where that stops being
a philosophical point. Both need a global planner, and the classical stack you
built in Lab 6 has one.

Notice also that the hybrid is identical to the classical system, to three
decimal places. That is a clue, and Exercise 10.3 follows it.

![Classical and learned controllers on identical arenas. A star ends at the goal, a cross does not.](docs/figures/lab10_trajectories.png){width=100%}


---

> ### Research Frontier: sim-to-real, and why you are not doing it
>
> Everything in this course happens in simulation, and the honest reason is that
> the university does not have thirty physical robots. The dishonest version of
> this lab would pretend otherwise. Instead you measure a real gap between two
> simulators, which is the same problem with a smaller constant.
>
> The techniques that address it are worth knowing.
>
> **Domain randomisation** varies simulation parameters during training, so the
> policy sees a distribution of physics rather than one setting and treats the
> real world as another sample. `FastNavEnv` exposes `domain_randomisation` and
> you use it today.
>
> **System identification** goes the other way, measuring the real robot and
> tuning the simulator to match. Cheaper when you have the robot and it is
> well-behaved.
>
> **Fine-tuning on the real system** transfers a simulation-trained policy and
> continues training on hardware. This is where sample efficiency stops being an
> academic concern, which is why SAC and offline RL dominate this literature.
>
> **Keeping a classical safety layer**, so the learned policy can be wrong
> without the robot being damaged. This is what actually gets deployed, and it is
> the Collision Monitor from Lab 7 doing the job it was built for.
>
> **Isaac Sim and Isaac Lab** are NVIDIA's GPU-accelerated simulation and RL
> stack, running thousands of environments in parallel with photorealistic
> rendering, and they are what a well-funded robot learning laboratory uses. They
> are not required for any graded work in this course, because the GPU stack
> cannot be relied upon in a university virtual machine. If you continue into
> robot learning research you will meet them immediately, and the concepts you
> have built here transfer directly.
>
> **Robot foundation models and vision-language-action policies** are the current
> frontier: single large models trained across many robots and tasks, taking
> language instructions and producing actions. They are genuinely impressive in
> demonstrations, they are far from the reliability and latency a warehouse AMR
> needs, and you should follow them without assuming they make any of this
> obsolete.

---

### Pre-lab quiz

Five questions covering why the discount factor bounds the planning horizon, what
distinguishes the training grammar from the competition grammar, why the hybrid
keeps a hand written switching criterion, what domain randomisation does, and why
a classical safety layer is retained in deployed learned systems.

---

## In the session

### Stage 0: health check (5 minutes)

```
course-check
export PYTHONPATH=.
```

### Stage 1: demonstration (10 minutes)

Your demonstrator will run the same policy in the surrogate and in Gazebo, side
by side, on the same arena layout. Watch where the behaviour diverges before you
measure it.

### Exercise 10.1: measure the simulation gap (25 minutes)

Take your dense-reward policy from Lab 9 and evaluate it in both environments on
matched arenas.

```
python3 -m arc_eval.runner --config arc_eval/configs/lab10_surrogate.yaml
python3 -m arc_eval.runner --config arc_eval/configs/lab10_gazebo.yaml
python3 -m arc_eval.runner --compare results/lab10_surrogate.json results/lab10_gazebo.json
```

| Metric | Surrogate | Gazebo | Change |
|--------|-----------|--------|--------|
| Success rate % | | | |
| Collision free % | | | |
| Time to goal (s) | | | |
| Path length (m) | | | |
| Minimum clearance (m) | | | |

Then account for the difference. The surrogate integrates a unicycle model
exactly and has no physics engine. Gazebo has wheel slip, motor and controller
dynamics, sensor latency, a discrete control loop that does not align with the
physics step, and a robot with mass that does not stop instantly.

| Physical difference | Predicted effect on the policy | Consistent with what you measured? |
|---------------------|--------------------------------|-------------------------------------|
| Wheel slip | | |
| Actuation lag | | |
| Sensor latency | | |
| Momentum on stopping | | |

**[GIF PLACEHOLDER]**
The same policy running in the surrogate and in Gazebo on identical arena
geometry, side by side.
*Instructor note: render the surrogate as a matplotlib animation of the
trajectory and record Gazebo with RViz alongside, at matched scale and matched
start pose. Choose a seed where the Gazebo run clips an obstacle the surrogate
run clears, since that is the frame that makes the gap concrete.*

### Exercise 10.2: close some of the gap (20 minutes)

Retrain with domain randomisation and measure whether it helped.

```
python3 starters/lab09/train.py --reward dense_progress --steps 200000 \
    --domain-randomisation
```

With randomisation on, the sensor noise standard deviation is resampled each
episode. That is a narrow form of randomisation and deliberately so; the exercise
is to see the mechanism, not to solve transfer.

| | Surrogate success | Gazebo success | Gap |
|--|-------------------|----------------|-----|
| Without randomisation | | | |
| With randomisation | | | |

Expect the randomised policy to be **slightly worse in the surrogate and closer
to its surrogate score in Gazebo**. Robustness is bought with specialisation, and
if you see no cost you should be suspicious of the measurement rather than
pleased.

Apply the rule from Lab 6 honestly. At 30 episodes, a change of a few percentage
points is inside the noise, and the correct report is inconclusive.

### Exercise 10.3: why the hybrid does nothing (25 minutes)

The supplied hybrid produces results identical to the classical controller. Find
out why.

```python
import sys
sys.path.insert(0, "starters/lab10")
from hybrid import make_hybrid_policy
from arc_rl.baselines import make_gap_follow_policy
from arc_rl.nav_core import FastNavEnv
from arc_rl.arenas import competition_scenario
from starters.lab09.train import make_policy_from

hyb = make_hybrid_policy(make_gap_follow_policy(),
                         make_policy_from("models/ppo_dense_progress"))
env = FastNavEnv(scenario_factory=competition_scenario, randomise_start=False)

for seed in range(500, 510):
    hyb.controller.reset()
    obs, _ = env.reset(seed=seed)
    for _ in range(2400):
        obs, _, term, trunc, info = env.step(hyb(obs))
        if term or trunc:
            break
    s = hyb.controller.stats
    print(f"seed {seed}: handovers={s.handovers} "
          f"learned_fraction={s.learned_fraction:.2f} end={info['termination_reason']}")
```

On the reference machine this reports **zero handovers in nine of ten episodes**,
with almost every episode ending in a collision.

The switching criterion requires the robot to be **stalled** and in **tight
space**. It was written on the assumption that the classical controller fails by
getting stuck. Measure what actually happens and you find it fails by driving
into things, and a collision terminates the episode before any stall can be
detected. The handover is guarding against a failure mode that does not occur.

This is a real design error, and it is the kind that measurement catches and
reasoning does not. The switching criterion encoded an assumption about the
failure distribution that nobody checked.

Now fix it. Choose one and implement it:

| Approach | What it assumes | Your result |
|----------|-----------------|-------------|
| Switch on low front clearance alone, no stall requirement | The learned policy is better in tight space | |
| Switch when the classical controller's chosen heading has no valid gap | The failure is a planning failure, not a stalling one | |
| Switch on predicted time-to-collision below a threshold | The failure is imminent contact | |
| Invert it: learned by default, classical when clearance is large | The classical controller is the specialist | |

Re-evaluate over the same 30 seeds and record whether your version beats both
components. It may not, and an honest inconclusive result with a correct
diagnosis scores full marks.

### Exercise 10.4: the deployment decision (15 minutes)

This is the question your Project 2 report has to answer, so answer it here in
draft.

Given everything measured across Labs 6, 9 and 10, which system would you deploy
in a hospital corridor at night, and which in a warehouse aisle, and why are the
answers different?

Your answer must engage with:

- The measured collapse of both reactive systems on the competition grammar, and
  what that implies about needing a global planner
- The dense policy's better clearance against its identical success rate, and how
  to weight those against each other
- Verifiability: what you can say about why the classical stack did something,
  and what you can say about why the policy did
- What happens when each system is wrong

There is no single correct answer. There are answers that engage with the
measurements and answers that do not, and only the first kind earns marks.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your simulation gap table and your physical-differences analysis.
2. Your domain randomisation comparison, reported with the correct statistical
   caution.
3. Your hybrid diagnosis with the handover counts you measured, and your fixed
   switching criterion with its evaluation.
4. Your deployment decision, half a page, which becomes a section of your Project
   2 report.

---

## Troubleshooting

**The Gazebo evaluation is far slower than expected.** It should be. Twenty runs
at 120 seconds is 40 minutes at a real time factor of 1.0 and longer below it.
Reduce the seed count for the in-session exercise and note that you did.

**The policy behaves completely differently in Gazebo.** Check the observation
first, not the policy. Print the observation vector from both environments in the
same pose and compare element by element. A mismatch in beam count, ordering or
normalisation is far more likely than a genuine physics effect.

**The hybrid never hands over even after your fix.** Log the criterion's inputs
every step rather than only the outcome. Being unable to see why a switch did not
fire is the same debugging failure the exercise is about.

**The hybrid oscillates and performs worse than either component.** Your
commitment window is too short. Re-deciding every tick at the boundary condition
produces exactly this, which is why the supplied controller commits for a fixed
number of steps.

**Domain randomisation made everything worse.** Possible and legitimate. If the
randomisation range is wide relative to the real variation, the policy hedges
against conditions that never occur. Report it.

---

## Connection to the Grand Challenge

That is the last laboratory. What remains is the competition, and you now have
everything you need to choose an approach and defend it.

Three things from today should shape that choice. The reactive systems collapse
on multi-checkpoint missions in partitioned arenas, which is what the competition
arena is, so a global planner is not optional. A learned component can improve
clearance without improving success, which may or may not be what the scoring
rewards, so read the scoring formula carefully. And a hybrid built on an
unexamined assumption about how the base system fails will do nothing at all,
quietly.

The competition specification is a separate document. The arena grammar in it is
the same one you have been evaluating against all term, and the seeds are drawn
publicly on the day.

---

## References

**Papers**

Chandra, R. et al. (2024). Hybrid Classical/RL Local Planner for Ground Robot
Navigation. arXiv:2410.03066. The architecture this lab implements. Short, and
the argument for a geometric rather than learned switching criterion is the part
to read closely.

Kolomeytsev, Y. et al. (2025). Hybrid Motion Planning with Deep Reinforcement
Learning for Mobile Robot Navigation. arXiv:2512.24651. The complementary
approach: a graph-based global planner feeding checkpoints into a DRL local
policy through both the observation and the reward.

Tobin, J. et al. (2017). Domain Randomization for Transferring Deep Neural
Networks from Simulation to the Real World. *IROS*. arXiv:1703.06907. The paper
that established the technique.

Zhao, W., Queralta, J. P. and Westerlund, T. (2020). Sim-to-Real Transfer in Deep
Reinforcement Learning for Robotics: a Survey. *IEEE SSCI*. arXiv:2009.13303. A
map of the field, useful for locating any specific method you encounter later.

**Official documentation**

NVIDIA Isaac Sim and Isaac Lab: https://developer.nvidia.com/isaac/sim
Not required for this course. Worth knowing what it does before an interview.

**Video**

ROSCon talks on deploying learned components in production navigation stacks.
Prefer talks by people shipping robots over talks by people publishing papers;
the failure modes they describe are the ones this lab measured.

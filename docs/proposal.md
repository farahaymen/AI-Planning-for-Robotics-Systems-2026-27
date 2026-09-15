---
title: "Autonomous Robotics with ROS 2: A Proposed Laboratory Redesign"
subtitle: "Mapping, Navigation and Reinforcement Learning"
author: "Farah Aymen | British University in Egypt"
date: "Proposal for review"
---

# Autonomous Robotics with ROS 2: a proposed laboratory redesign

## Summary

This proposes a redesign of the autonomous robotics laboratory as a ten week
sequence in which students implement the core algorithms themselves, configure
the production tools used in industry, compare classical and learned approaches
with proper experimental method, and finish with a competitive autonomous
robotics challenge.

The proposal is not a plan. The laboratory manuals, the starter packages, the
evaluation infrastructure and the assessment scheme are written, and the parts
that could be tested without ROS have been tested: 87 unit tests pass, and the
central technical claim, that a navigation policy can be trained on a CPU inside
a laboratory session, has been measured rather than assumed.

Six defects were found during that validation and corrected. They are listed in
section 8, because they are the strongest available evidence that the material
has been built rather than described.

---

## 1. Why change

The current laboratory teaches robotics tooling. Students finish able to follow a
ROS tutorial and unable to say why any of it is shaped the way it is.

Three specific gaps motivate this redesign.

**Students implement nothing.** Configuring a package and understanding an
algorithm are different skills, and only one of them transfers. A student who has
never written an occupancy grid update has no basis for tuning one.

**There is no experimental method.** Robotics results are reported from single
runs. This is the opposite of what the discipline requires, and it is a habit
that damages students who continue to research.

**The learning-based half of the field is absent.** Our students take Machine
Learning, Deep Learning, NLP and Reinforcement Learning. They arrive fluent in
PyTorch and leave the robotics laboratory without having connected any of it to a
robot.

The redesign addresses all three, and it does so without asking for extra
timetabled hours.

---

## 2. Design principles

**Python continuity.** Roughly 85 percent Python, using NumPy, PyTorch,
Gymnasium and rclpy, so that the robotics laboratory feels like a continuation of
the AI modules rather than a different subject. C++ appears as one build task and
three guided reading exercises, which is enough for graduates to read a Nav2
plugin without turning the module into a C++ course.

**Implement the concept, configure the system.** Students write breadth first
search, Dijkstra, A*, occupancy grid mapping with an inverse sensor model,
Gymnasium environments and reward functions. They configure SLAM Toolbox, AMCL,
robot_localization, Nav2 and ros2_control. Nobody reimplements an industrial SLAM
system, and nobody tunes a costmap they do not understand.

**Every claim is measured.** All benchmarking in the course runs through one
evaluation harness that executes a seeded set of episodes and reports means,
standard deviations and confidence intervals. Students are told explicitly that a
difference smaller than the standard deviation is not a result, and are marked on
reporting inconclusive findings honestly.

**Fairness by construction.** A standardised virtual machine, a frozen reference
robot, three defined graphics fallback tiers with a commitment that every graded
task is completable in the middle tier, and a health check that produces an
incident ID so that infrastructure failures are recorded rather than argued over.

---

## 3. The ten week architecture

![The semester arc.](docs/figures/diagram_course_arc.png){width=100%}


The course follows one arc: fundamentals, Python implementation, ROS 2
integration, industry practice, current research, autonomy, reinforcement
learning, hybrid systems, competition.

| Week | Laboratory | Students implement | Students configure |
|------|-----------|--------------------|--------------------|
| 1 | ROS 2 as a software system | Publisher, subscriber, QoS diagnosis | Lifecycle nodes, DDS discovery |
| 2 | Kinematics, TF2, URDF, ros2_control | Differential drive kinematics | URDF, controllers, one C++ build |
| 3 | Gazebo, sensors, rosbag2 | Sensor rate and clock diagnostics | Simulation, bridge, recording |
| 4 | Occupancy grid mapping | Log odds mapping, inverse sensor model | Map server, map export |
| 5 | Localisation and SLAM | Particle filter, motion model | SLAM Toolbox, AMCL, robot_localization |
| **6** | **Project 1 demonstration** | | |
| 7 | Autonomous navigation with Nav2 | A* on the real costmap | Nav2, costmap layers, controllers |
| 8 | Robust autonomy and safety | Stuck detection, recovery escalation | Behaviour trees, Collision Monitor |
| 9 | Building an RL environment | Gymnasium environment, rewards | Environment validation |
| 10 | Deep RL for navigation | Training and evaluation | PPO, TensorBoard |
| 11 | Hybrid autonomy and sim-to-real | Hybrid switching controller | Domain randomisation |
| **12** | **Grand Challenge** | | |

A take-home assessed notebook covering breadth first search, depth first search,
Dijkstra and A* is due in week 4. Relocating it out of contact hours is what
frees the two project slots without extending the timetable, and it is defensible
because that content needs no ROS, no simulator and no virtual machine.

Each laboratory carries four layers: the fundamental concept, a student
implementation, an industry perspective, and either a research frontier or an
engineering practice discussion. The concept delivery moves into required pre-lab
reading with a short gated quiz, which raises hands-on time in the session from
about 60 minutes to about 75 without adding an hour to the timetable.

---

## 4. Industry alignment

Across the semester students use Linux, Git, ROS 2 Jazzy, rclpy, TF2, URDF,
Xacro, QoS, actions, parameters, launch files, rosbag2, Gazebo Harmonic, RViz2,
ros2_control, SLAM Toolbox, robot_localization, AMCL, Nav2, behaviour trees, the
Collision Monitor, Gymnasium, PyTorch, Stable-Baselines3, pytest, launch_testing
and Docker.

Three choices deserve particular note.

**ROS 2 Jazzy rather than the 2026 LTS.** Lyrical Luth released in May 2026 and
is the correct long term target. Nav2, SLAM Toolbox and ros2_control tooling are
mature on Jazzy and still settling on Lyrical, and a course whose central promise
is grading fairness cannot also be where a distribution transition is debugged. A
migration review is scheduled for December 2026 and the decision will be recorded
either way.

**Safety standards are named.** Laboratory 8 distinguishes the Nav2 Collision
Monitor, a software safeguard, from a safety rated protective device in the sense
of ISO 3691-4. Graduates who conflate the two lose credibility quickly in
industry.

**Declared use of language models.** Students may use them and must declare what
was generated in an `AI_USE.md` file. Enforcement is the individual viva, in
which each student diagnoses a live fault introduced in front of them. A module
that pretends these tools do not exist reads as dated.

---

## 5. Research alignment

Six laboratories carry a research frontier discussion; the remainder carry
engineering practice instead. That split is deliberate. Forcing a recent paper
into a laboratory on breadth first search produces decorative citations, which
students recognise.

The frontier content covers modern map representations including semantic mapping
and Gaussian splatting, learned and hybrid planning, verification of behaviour
trees, the reproducibility problems in reported RL results, and sim-to-real
transfer. Papers are cited with a stated reason to read them.

All frontier text lives in a single annex included into the manuals at build
time, so the annual refresh is one file rather than ten documents.

---

## 6. Assessment

| Component | Weight |
|-----------|--------|
| Pre-lab quizzes, best 9 of 10 | 5 % |
| Weekly exit tasks, best 8 of 10 | 20 % |
| Path planning notebook | 10 % |
| Project 1, autonomous mapping and navigation | 25 % |
| Project 2, academic component | 30 % |
| Project 2, competition performance | 10 % |

Competition performance is capped at 10 percent deliberately. The event should
motivate without making a degree classification depend on whether a dynamic
obstacle crossed a corridor at the wrong moment. Named awards carry the
motivational load instead, including a Best AI Solution award available to a team
whose learned approach honestly did not beat the classical baseline.

Both project marks are multiplied by an individual peer contribution factor
between 0.8 and 1.1, determined from an anonymous peer form, the Git history and
the individual viva. The mechanism is published in week one, and its main effect
is preventive.

The module handbook will state that no mark is lost to a failure of the official
environment. The health check produces an incident ID which makes that promise
enforceable in two minutes rather than unfalsifiable in both directions.

---

## 7. The Grand Challenge

Teams place a robot at an unknown pose in an unseen arena and must visit four
checkpoints while handling static obstacles, moving obstacles and a blocked
route, without human intervention.

The arena is unseen but drawn from a grammar published in week 9, so teams know
the distribution they must generalise over. Every generated arena is validated
for reachability by breadth first search before use, so an unwinnable arena
cannot be drawn. Seeds are drawn publicly on the day.

Each team receives ten unscored minutes in the arena beforehand to teleoperate
and build a map. This mirrors how a commercial autonomous mobile robot is
commissioned at a new site, keeps everything taught in week 5 directly relevant,
and removes an untaught capability, autonomous frontier exploration, from the
critical path.

Scoring is computed by a tested function from the recorded metrics, so a disputed
score is resolved by re-running it rather than by argument. The weights encode a
priority order: complete the mission, then do not hit things, then take an
efficient route, then be quick.

---

## 8. Validation status

The laboratory manuals were not written from assumption. Everything that could be
tested without ROS has been.

**87 unit tests pass** across the seven starter modules. **The evaluation harness
runs end to end.** **PPO training was measured**, not estimated: 200,000 steps in
1.9 minutes on a single CPU thread, which is what makes reinforcement learning
possible inside a two hour session on a virtual machine with no GPU.

Six defects were found and corrected during that process. They are the strongest
evidence available that the material is real.

| Defect | Consequence had it shipped |
|--------|----------------------------|
| Wheel command interface capped at 10 rad/s while the velocity limits demand 16.3 | Robot saturates silently and turns slowly; presents as a controller tuning fault |
| Training environment regenerated its arena only when given a seed | Every training run sees four layouts; students taught to overfit while the curves look healthy |
| Competition scoring rewarded speed over safety | A 40 s run with two collisions outscored a clean 110 s run, contradicting the stated intent |
| LibreOffice silently drops equations when exporting Word to PDF | Students on lab machines see blank space where the kinematics should be |
| `pytest.importorskip` in a decorator skipped an entire test file | Six self-check tests silently never ran |
| Both benchmark configurations wrote to the same results file | The Lab 7 controller comparison compares a file with itself |

Two further findings changed the teaching rather than the code. A sparse reward
function produces a policy that freezes, scoring zero percent success while
achieving the best safety metrics of the three systems, and the arithmetic
predicts it exactly. And the hybrid controller hands over in one episode in ten,
because it was built assuming the classical controller fails by stalling when it
actually fails by colliding. Both are now exercises rather than assertions.

What has **not** been validated is anything requiring ROS 2 or Gazebo, which
cannot run in the environment used to author this. Every laboratory therefore
carries a validation checklist, 15 to 28 items, to be signed off on the weakest
laboratory PC before release. The checklists identify the highest risk item in
each session; the binding constraint across the course is Gazebo-based evaluation
in week 11, which does not fit a two hour session at 30 seeds and must be set as
coursework.

---

## 9. What a graduate can do

A student completing this laboratory has implemented four search algorithms,
occupancy grid mapping, a particle filter, differential drive kinematics, a
Gymnasium environment and a reward function. They have configured a full Nav2
stack including costmap layers, behaviour trees and a collision monitor. They
have trained a navigation policy, found it did not beat the classical baseline on
success, and reported that honestly.

They can diagnose a robot that is not working using a systematic workflow rather
than by guessing. They have used Git throughout, written tests, and produced a
repository someone else can clone and run.

Most importantly they can compare two systems properly, over seeds, with
uncertainty, and say which they would deploy and why. That is the skill that
distinguishes an engineer from someone who can run a tutorial, and it is the one
this redesign is built around.

---

## 10. What is needed

**Confirmation of seven assumptions**, listed in the design baseline. The two
that matter most are whether a parallel lecture module delivers robotics theory,
which changes how much the laboratory must carry, and the actual RAM in the
laboratory PCs, since a 12 GB virtual machine on a 16 GB host produces stuttering
that looks like a graphics fault.

**A decision on the timetable.** The design assumes twelve teaching weeks with
ten laboratory sessions and two project weeks. If the module is strictly ten
weeks, the least damaging cut is folding week 11 into Project 2 preparation.

**Technician time** to build and image the virtual machine, and access to the
weakest laboratory PC for validation.

**Approval to proceed** with one laboratory as a vertical slice for review before
the remainder is finalised. Laboratory 7, on Nav2, is the densest integration
point and is already written; validating it end to end proves the virtual
machine, the reference robot, the evaluation harness and the manual format in one
pass.

---

## Appendix: intended learning outcome mapping

To be completed against the module specification.

| Week | Laboratory | Assessment evidence | Module ILO | Framework outcome |
|------|-----------|---------------------|------------|-------------------|
| 1 | ROS 2 as a software system | Exit task, live diagnosis | | |
| 2 | Kinematics, TF2, URDF | Exit task, C++ build task | | |
| 3 | Gazebo and sensors | Exit task, recorded bag | | |
| 4 | Occupancy mapping | Exit task, planning notebook | | |
| 5 | Localisation and SLAM | Exit task, saved map | | |
| 6 | Project 1 | Report, 30 seed evaluation, viva | | |
| 7 | Nav2 | Exit task, seeded benchmark | | |
| 8 | Robust autonomy and safety | Exit task, recovery tree | | |
| 9 | RL environment | Exit task, validated environment | | |
| 10 | Deep RL | Exit task, reward comparison | | |
| 11 | Hybrid autonomy | Exit task, deployment analysis | | |
| 12 | Grand Challenge | Report, evaluation, viva, competition | | |

The framework column should be completed from the programme's accreditation
documentation rather than from a general source, since the codes and wording
differ by scheme and version.

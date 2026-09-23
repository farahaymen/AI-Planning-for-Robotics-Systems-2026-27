---
title: "Autonomous Robotics Lab Redesign: Design Baseline"
subtitle: "Decisions taken before any lab manual is written"
author: "Farah Aymen, British University in Egypt"
date: "Version 0.1"
---

# 0. Purpose and status

This document is not a lab manual and it is not the proposal. It is the layer
underneath both: the set of decisions that every lab manual, every starter
package and the professor facing proposal will depend on. Writing ten manuals
before these decisions are frozen guarantees rework, because a change to the
observation contract or the grading scheme propagates into every document.

Everything here is a decision with a stated reason. Where a decision rests on
something I do not yet know about the module or the hardware, it is marked as an
assumption in section 1 and should be confirmed before the document is
circulated.

The decision register in section 2 is the summary. The remaining sections give
the reasoning and the concrete artifacts.

---

# 1. Assumptions requiring confirmation

These materially change the answers below. Each is written as the assumption I
have made so that correcting it is a single edit rather than a redesign.

| ID | Assumption | Why it matters | Confirm with |
|----|------------|----------------|--------------|
| A1 | The module runs 12 timetabled weeks, of which 10 are laboratory sessions | Determines whether the projects get their own slots | Module specification |
| A2 | There is no parallel lecture module delivering robotics theory | Determines how much concept delivery the lab must carry | Programme handbook |
| A3 | Cohort is 40 to 80 students, working in teams of 3 to 4 | Drives TA load, autograding need and competition format | Registry |
| A4 | Lab PCs have at least 16 GB RAM and 4 physical cores | Caps the VM allocation | Estimates office |
| A5 | Lab PCs restore to a clean image between sessions | Forces the work persistence policy in section 4.3 | Lab technician |
| A6 | Assessment weights are set by the module leader, not fixed centrally | Determines whether section 8 is a proposal or a constraint | Module leader |
| A7 | Students have GitHub accounts and outbound HTTPS from the lab network | The whole version control thread depends on this | IT services |

Assumption A2 is the one I would confirm first. If theory is delivered
elsewhere, the flipped structure in section 3.2 becomes easier and the labs get
more hands on time than the table shows.

---

# 2. Decision register

| ID | Decision | Section |
|----|----------|---------|
| D1 | Ten labs, but Lab 4 moves out of contact hours as an assessed take home notebook | 3.3 |
| D2 | Concept, industry and research delivery move to pre lab reading with a gated quiz | 3.2 |
| D3 | Project 1 and the competition each get a dedicated timetabled slot | 3.3 |
| D4 | Stay on ROS 2 Jazzy for 2026.1 with a documented migration review for 2027.1 | 4.1 |
| D5 | Three graphics tiers; every graded task must be completable in Tier B | 4.2 |
| D6 | Nothing inside the VM is durable; Git push is the last step of every session | 4.3 |
| D7 | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` plus per seat `ROS_DOMAIN_ID` | 4.4 |
| D8 | `course-check` exits non zero and prints an incident ID used to protect marks | 4.5, 8.6 |
| D9 | One frozen reference robot; parameters are course constants, not team choices | 5 |
| D10 | All benchmarking runs through `arc_eval`; no single run result is ever graded | 6 |
| D11 | Ten seeds minimum for lab claims, thirty for project claims | 6.4 |
| D12 | RL trains in a fast NumPy surrogate and is evaluated in ROS and Gazebo | 7 |
| D13 | One observation and action contract shared by both environments | 7.2 |
| D14 | Policy networks capped at 2x64 MLP with 24 LiDAR beams for CPU feasibility | 7.5 |
| D15 | Competition performance capped at 10 percent of the module mark | 8.1 |
| D16 | Individual viva during both project demonstrations | 8.3 |
| D17 | Peer contribution factor of 0.8 to 1.1 applied to project marks | 8.4 |
| D18 | LLM use permitted and declared in `AI_USE.md`; the viva is the check | 8.5 |
| D19 | Competition teams get a ten minute unscored mapping run before the scored run | 10.1 |
| D20 | The arena grammar is published; arenas are drawn from it and seed identified | 10.2 |
| D21 | Two interventions maximum, each carrying a penalty and the clock keeps running | 10.3 |
| D22 | Manuals authored in Markdown, generated to DOCX with pandoc | 11.2 |
| D23 | Research frontier content lives in a separate annex refreshed annually | 11.3 |
| D24 | Six labs carry a research frontier box, four carry an engineering practice box | 11.3 |
| D25 | Starter packages ship with pytest and launch_testing checks students run themselves | 11.4 |
| D26 | C++ gets one build task in Lab 2 and three read only walkthroughs | 12.1 |
| D27 | Lifecycle nodes introduced in Lab 1, not deferred to Lab 7 | 12.3 |
| D28 | ISO 3691-4 safety context added to Lab 7 | 12.4 |
| D29 | Lab 5 splits mapping from localisation across two sessions | 12.5 |
| D30 | One lab built end to end as a vertical slice before the rest are written | 14 |

---

# 3. Time budget and semester structure

## 3.1 The arithmetic that does not close

Ten sessions at two hours is twenty contact hours. The original structure spends
45 of every 120 minutes on recap, concept, industry and research, leaving 75
minutes, of which the last 15 are experiment and submission. Actual hands on
build time is therefore about 60 minutes per session, or ten hours for the whole
semester.

Ten hours cannot absorb ROS 2 fundamentals, TF and URDF and ros2_control, Gazebo
and sensor debugging, four search algorithms, occupancy mapping, SLAM, AMCL,
Nav2, behaviour trees, collision monitoring, a custom Gymnasium environment,
policy training and a hybrid comparison. Two labs are individually impossible as
scoped. Lab 5 contains log odds mapping, particle filter intuition, EKF
intuition, scan matching, loop closure, SLAM Toolbox, map saving, AMCL and an
odometry comparison. Lab 2 contains differential drive kinematics, TF2, URDF,
Xacro and ros2_control.

Three changes recover the time without adding a single timetabled hour.

## 3.2 Decision D2: flip the delivery

The Word manual becomes required pre lab reading. It carries the fundamental
concept, the mathematics, the industry perspective and the research frontier. A
five question quiz on the VLE closes one hour before the session and is worth a
small part of the mark, which is what makes the reading actually happen.

The revised in session template:

| Minutes | Activity | Change |
|---------|----------|--------|
| 0 to 10 | Health check, recap, questions on the pre lab reading | Was 0 to 10 recap plus 10 to 45 delivery |
| 10 to 25 | Live demonstration of the day's target behaviour, then the fault to be diagnosed | New |
| 25 to 100 | Main build and integration exercise | Was 45 to 105 |
| 100 to 112 | Seeded benchmark run through `arc_eval` | Was 105 to 115 |
| 112 to 120 | Commit, push, exit task submission | Was 115 to 120 |

Hands on time rises from 60 to 75 minutes and the benchmarking slot becomes
protected rather than the first thing sacrificed when the build overruns.

The 10 to 25 minute demonstration is worth defending. Watching the working system
before building it gives students a target behaviour to compare against, which is
what makes debugging tractable in a two hour window.

## 3.3 Decisions D1 and D3: where the content and the projects sit

Lab 4, search and path planning, requires no ROS, no Gazebo, no VM and no
simulator. It is pure NumPy and `heapq`. It is also the content closest to what
students already do in their Machine Learning and Deep Learning modules, which
means it is the lowest risk thing to set as independent work.

Moving it out of contact hours as an assessed take home notebook frees an entire
session and loses nothing pedagogically. The ROS facing half of the original Lab
4, converting a `nav_msgs/OccupancyGrid` into a NumPy array and planning on a
real saved map, moves into the first fifteen minutes of the Nav2 lab where it
belongs, because that is where the connection actually pays off.

Recommended twelve week map under assumption A1:

| Week | Session | Assessment |
|------|---------|------------|
| 1 | Lab 1: ROS 2 as a software system | Exit task |
| 2 | Lab 2: Kinematics, TF2, URDF, ros2_control | Exit task, C++ build task |
| 3 | Lab 3: Gazebo, sensors, rosbag2 | Exit task, bag submitted |
| 4 | Lab 4: Occupancy mapping | Exit task; planning notebook due |
| 5 | Lab 5: Localisation, AMCL, SLAM Toolbox | Exit task |
| 6 | **Project 1 build and demonstration** | Project 1, individual viva |
| 7 | Lab 6: Nav2 | Exit task, seeded benchmark |
| 8 | Lab 7: Behaviour trees, recovery, safety | Exit task |
| 9 | Lab 8: Gymnasium environment engineering | Exit task, `check_env` passing |
| 10 | Lab 9: PPO and SAC for navigation | Exit task, reward comparison |
| 11 | Lab 10: Hybrid autonomy and generalisation | Exit task, baseline comparison |
| 12 | **Autonomous Robotics Grand Challenge** | Project 2, individual viva |

If the module is strictly ten weeks, the fallback is to fold Lab 10 into the
Project 2 preparation week and deliver its content as a guided comparison
exercise inside Project 2 rather than as a separate session. That is the least
damaging cut available, because Lab 10 is analysis rather than new capability.
What must not happen is running the projects entirely outside contact hours,
which pushes all integration failure onto students without TA support and
produces the worst version of this course.

---

# 4. Course software baseline

## 4.1 Decision D4: stay on Jazzy, document why

ROS 2 Lyrical Luth was released in May 2026 as the new LTS on Ubuntu 26.04, with
support to May 2031. It is the correct long term target and the wrong choice for
this semester. Nav2, SLAM Toolbox, ros2_control and the entire tutorial and Stack
Overflow corpus are mature on Jazzy and still settling on Lyrical. A course whose
central promise is grading fairness through a reproducible environment cannot
also be the place where a distribution transition is debugged.

State this explicitly in the professor facing proposal. A stated decision with a
migration review date reads as engineering judgement. An unexplained older
version reads as being out of date.

VM release naming: `ARC VM 2026.1`. The migration review to Lyrical is scheduled
for December 2026, with the decision recorded in `docs/adr/` regardless of
outcome.

## 4.2 Decision D5: three graphics tiers

Gazebo Harmonic under virtualised 3D acceleration is the largest technical risk
in the entire plan, and it is a risk that expresses itself as a student unable to
complete a graded task through no fault of their own.

| Tier | Condition | Configuration |
|------|-----------|---------------|
| A | Hardware OpenGL 4.x available in the guest | Gazebo GUI, RViz2, full visual experience |
| B | OpenGL unreliable or software rendered | `gz sim -s` headless server, RViz2 only, `LIBGL_ALWAYS_SOFTWARE=1` |
| C | Gazebo will not run at all | Bag replay plus the NumPy surrogate simulator |

The commitment that makes this a fairness guarantee rather than a contingency:
**every graded task must be completable in Tier B.** Tier A adds visual
comprehension and nothing else. Tier C is a documented fallback that keeps a
student productive for one session while their machine is fixed, and it exists
because `arc_rl.nav_core` and `arc_eval` have no ROS dependency at all.

Test on the weakest lab PC, in the weakest tier, before release.

Hypervisor: test both VirtualBox and VMware Workstation Player and pick on
measured Gazebo frame rate rather than on licensing preference. VMware has
historically been more reliable for guest OpenGL.

## 4.3 Decision D6: nothing inside the VM is durable

If lab machines restore between sessions, work stored inside the VM is lost, and
this will happen to somebody in week two.

Policy: the last item of every weekly exit task is `git push`. The exit task is
not considered submitted until the commit appears on the remote. This removes an
entire category of complaint, gives you a timestamped record of individual
contribution for the peer factor in section 8.4, and builds the version control
habit the curriculum wants anyway without needing a dedicated software
engineering lab.

The course workspace `~/arc_ws/src/` is a Git repository from first boot, with
the remote already configured to a template the student forks in Lab 1.

## 4.4 Decision D7: DDS discovery on a shared subnet

Thirty students on one subnet with default multicast discovery will see each
other's nodes. A student will drive another student's robot, and diagnosing it
during a lab is expensive.

`ROS_LOCALHOST_ONLY` was deprecated in favour of a finer grained mechanism. On
Jazzy the correct configuration is:

```bash
# /etc/profile.d/arc-ros.sh, baked into the golden image
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=${ARC_SEAT_ID:-42}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ARC_VM_RELEASE=2026.1
```

`ARC_SEAT_ID` is set per machine from the lab seat number, so that if a lab ever
needs cross machine discovery for a multi robot demonstration, changing one
variable enables it. Verify both variables in `course-check`; a wrong
`ROS_DOMAIN_ID` is otherwise invisible until two nodes mysteriously fail to see
each other.

Lab 1 should include a short, deliberate exercise where two terminals are given
different domain IDs and students diagnose the resulting silence. Discovery
failure is one of the most common real ROS problems and one of the least taught.

## 4.5 Decision D8: `course-check` as a contract

The health check is not a convenience script. It is the evidence that decides
whether a failure is the student's problem or the infrastructure's problem, so it
needs to behave like a test suite: deterministic, exit coded, and producing an
artifact.

Requirements:

- Every check reports PASS, FAIL or SKIP with the observed value, not just a tick
- Exit code 0 only if all required checks pass
- Writes `~/arc_ws/.arc/health-<timestamp>.json` on every run
- On failure prints an incident ID of the form `ARC-<seat>-<date>-<hash>`
- Detects and reports the graphics tier from section 4.2

The incident ID is what a student pastes into the mark protection form described
in section 8.6. Without an artifact, "the VM was broken" is unfalsifiable in both
directions, which is bad for students and bad for you.

The smoke test `course-smoke-test` goes further and asserts behaviour rather than
presence: Gazebo starts, the robot spawns, `/scan` and `/odom` publish at the
expected rate, the TF tree resolves `map` to `base_link`, a `/cmd_vel` command
produces measurable odometry displacement, and the process tears down cleanly.
That last one matters more than it sounds; orphaned `gz` processes are a frequent
cause of the second lab of the day behaving strangely.

## 4.6 VM resources against assumption A4

The original figures need checking against the actual hardware. Twelve GB of RAM
allocated to a guest on a 16 GB host leaves the host swapping, and the symptom is
Gazebo stuttering that looks like a graphics problem.

| Resource | Minimum | Preferred | Constraint |
|----------|---------|-----------|------------|
| vCPU | 4 | 6 | Never exceed physical cores minus one |
| RAM | 8 GB | 12 GB | Host must retain at least 4 GB |
| Disk | 60 GB dynamic | 60 GB dynamic | Check aggregate free space on shared machines |
| Video memory | 128 MB | 256 MB | With 3D acceleration enabled |

Distribution of a 20 to 30 GB compressed appliance to a full cohort over the
campus network is its own logistics problem. Plan for the lab machines to be
pre imaged by the technicians and treat student owned copies as a secondary
channel served from a local mirror or USB, not from a cloud link.

---

# 5. Decision D9: the frozen reference robot

These values are course constants. They appear in the URDF, in the Gazebo world
files, in the Nav2 costmap configuration and in `arc_rl.nav_core.RobotSpec`, and
they must agree in all four places. A mismatch between the URDF footprint and the
costmap footprint is a genuinely difficult bug to find and there is no reason for
students to meet it accidentally.

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel radius | 0.050 m | |
| Wheel separation | 0.350 m | |
| Mass | 6.0 kg | |
| Footprint radius | 0.220 m | Circumscribed; used for costmap inflation |
| Maximum linear velocity | 0.50 m/s | |
| Minimum linear velocity | -0.125 m/s | Limited reverse |
| Maximum angular velocity | 1.80 rad/s | |
| Control period | 0.05 s | 20 Hz |
| Wheel command limit | 20.0 rad/s | Must exceed 16.3, the peak demanded by the velocity limits |
| LiDAR beams | 360 | Downsampled to 24 for RL |
| LiDAR range | 0.12 to 12.0 m | |
| LiDAR rate | 10 Hz | |
| IMU rate | 100 Hz | |
| Odometry rate | 50 Hz | |
| Goal tolerance | 0.25 m | |

These are plausible starting values and must be validated in simulation before
freezing. In particular, confirm that the maximum angular velocity is achievable
given the mass and wheel separation without the physics solver producing wheel
slip, and that the LiDAR rate does not starve the Nav2 local costmap update.

Deliberately excluded from student control: all of the above. Deliberately under
student control: everything in software. This is what makes team results
comparable, which is what makes the competition fair.

---

# 6. Decisions D10 and D11: the evaluation harness

## 6.1 Why this is the highest leverage change

The Golden VM solves dependency reproducibility. It does not solve result
reproducibility, and result reproducibility is the one that actually determines
marks. Two teams running identical Nav2 configurations get different outcomes,
and a single run comparison of two controllers is an anecdote.

For a course that positions itself as research aware, teaching students to draw
conclusions from one run would be the single largest intellectual failure
available. The fix is a course wide harness that every benchmarking claim passes
through.

## 6.2 What it does

`arc_eval.runner` takes an environment factory and a policy callable, runs one
episode per seed, and emits a standard JSON document. It is backend agnostic by
construction, so exactly the same code path evaluates a Nav2 configuration, a
trained policy, and a hybrid stack. The only requirement on an environment is
that it reports the standard info keys on its terminal step.

Standard metrics, all of which the harness computes:

| Metric | Aggregation | Note |
|--------|-------------|------|
| Success rate | Proportion with Wilson 95% interval | Normal approximation is wrong at n=10 |
| Collision free rate | Proportion | Separate from success on purpose |
| Time to goal | Mean and SD over **successes only** | |
| Path length | Mean and SD over **successes only** | |
| Collisions per episode | Mean and SD over all episodes | |
| Minimum clearance | Mean and SD over all episodes | |
| Recovery events | Mean and SD | Nav2 recovery behaviour count |
| Interventions | Total | Competition relevant |

Two of these choices are worth explaining to students rather than hiding in the
code. Averaging time to goal over failed episodes rewards a policy that gives up
quickly, which is how navigation benchmarks most commonly mislead. And the Wilson
interval does not collapse to zero width when ten episodes out of ten succeed,
which is the honest answer at lab sample sizes.

## 6.3 Decision D11: sample sizes

Measured on the validated implementation, a classical reactive baseline scoring
40 percent success over ten seeds carries a 95 percent interval of 16.8 to 68.7
percent. That is a wide enough band that a team claiming to have beaten it at 55
percent has demonstrated nothing.

- Weekly lab benchmarking: 10 seeds, sufficient for large effects
- Project 1 and Project 2 claims: 30 seeds minimum
- Any claim of an improvement smaller than one standard deviation must be
  reported as inconclusive, and doing so honestly earns marks rather than losing
  them

That last rule is the one that changes behaviour. Students currently learn that
negative results are failures. Stating in the rubric that a well executed
inconclusive comparison scores full marks for methodology is cheap and it teaches
something real.

## 6.4 Where the harness appears

| Session | Use |
|---------|-----|
| Lab 4 | Planner comparison on identical maps |
| Lab 6 | Two Nav2 controller configurations |
| Lab 7 | Recovery success under injected faults |
| Lab 9 | Two reward formulations |
| Lab 10 | Classical against learned against hybrid |
| Project 1 | Multi goal mission over 30 seeds |
| Project 2 | Official scored runs and the leaderboard |

By the time students reach the competition, the harness is familiar
infrastructure rather than an unfamiliar constraint imposed on the last week.

---

# 7. Decisions D12 to D14: reinforcement learning that actually trains

## 7.1 The problem with the original plan

Gazebo in the loop reinforcement learning is limited by real time factor. Inside
a VM with no GPU passthrough, a Gazebo backed environment collects roughly tens
to low hundreds of transitions per second. PPO navigation policies need hundreds
of thousands. Labs 8, 9 and 10 as originally scoped cannot produce a working
policy inside two hours, and supplying a pre trained checkpoint to hide this
means students never see training work.

## 7.2 Decision D13: one contract, two environments

The design that does work is two environments that share a single, frozen
interface:

```
                    arc_rl/nav_core.py
        RobotSpec  |  ObsSpec  |  build_observation()  |  scale_action()
                            |                |
              +-------------+                +-------------+
              |                                            |
        FastNavEnv                                    RosNavEnv
        pure NumPy                                    rclpy + Gazebo
        ~3,800 steps/s                                ~20 steps/s
        TRAINING                                      EVALUATION, DEPLOYMENT
```

Both environments import the same `build_observation` function. A policy trained
in the surrogate loads and runs against the ROS stack with no change to the
network shape or the meaning of any observation element. This is verified rather
than asserted: `arc_eval` refuses to run an environment that does not report the
required info keys, and a shape assertion in the loader catches a contract
mismatch immediately rather than as mysteriously poor performance.

## 7.3 Why the gap between them is a feature

The surrogate has no physics engine, no wheel slip, no sensor latency and no
controller dynamics. A policy that works in it will degrade in Gazebo.

This is not a defect to be minimised. It is the simulation gap, and Lab 10's sim
to real discussion becomes something students have measured rather than read
about. The Lab 10 question changes from "what is the sim to real gap" to "you
have a policy that scores 82 percent in the surrogate and 61 percent in Gazebo,
account for the difference and propose two mitigations." Domain randomisation
stops being a term and becomes a lever they can pull, with `domain_randomisation`
already exposed as a constructor argument.

## 7.4 Decision D14: CPU feasibility budget

| Constraint | Value | Reason |
|------------|-------|--------|
| Observation size | 29 | 24 LiDAR beams plus 5 state elements |
| LiDAR downsampling | Minimum per sector, not mean | A table leg in one raw beam must survive |
| Policy network | MLP 2 x 64 | Trains on CPU; larger buys nothing at this observation size |
| Training budget in lab | 200k to 300k steps | Roughly 4 to 6 minutes measured |
| Algorithms | PPO primary, SAC as a contrast | DQN mentioned for discrete framing only |

The minimum rather than mean downsampling deserves a sentence in the manual. It
is a small decision with a visible consequence, and explaining it teaches
students that observation design is engineering rather than plumbing.

## 7.5 Revised Labs 8 to 10

**Lab 8** builds `FastNavEnv` from a skeleton. Students implement `reset`,
`step`, the reward function and the termination logic; the raycasting geometry is
supplied because ray-segment intersection is not the learning objective. The exit
criterion is `check_env` passing and a random policy rollout completing. No
training occurs. The ROS backed environment is demonstrated but not built.

**Lab 9** trains two policies with different reward functions and benchmarks both
against the supplied classical baseline through `arc_eval`. The measured floor is
40 percent success on the training grammar, so students have something concrete
to beat. The reward hacking discussion is grounded in an actual observed failure:
`reward_dense_progress` produces policies that orbit the goal collecting progress
from oscillation, which is visible in the trajectory plot.

**Lab 10** loads the Lab 9 policy into the ROS backed environment, measures the
drop, and compares three architectures on unseen arenas drawn from the
competition grammar.

---

# 8. Assessment

## 8.1 Decision D15: weights

Proposed, subject to assumption A6.

| Component | Weight | Basis |
|-----------|--------|-------|
| Pre lab quizzes, best 9 of 10 | 5 % | Makes the reading happen |
| Weekly exit tasks, best 8 of 10 | 20 % | Two dropped scores absorb illness without paperwork |
| Path planning notebook | 10 % | The relocated Lab 4 |
| Project 1 | 25 % | Report, code, demonstration, individual viva |
| Project 2 academic component | 30 % | Report, code, methodology, individual viva |
| Project 2 competition performance | 10 % | Capped deliberately |

Capping competition performance at 10 percent is the decision that keeps the
event motivating without making the degree classification depend on whether a
dynamic obstacle happened to cross a corridor at the wrong moment. Ranking
prestige does the motivational work; the awards in section 10.5 cost nothing and
carry it.

## 8.2 Exit task rubric

One rubric for all ten weeks, four points each, scaled into the 20 percent:

| Level | Descriptor |
|-------|------------|
| 0 | Not submitted, or not pushed to the remote |
| 1 | Runs but does not produce the specified artifact |
| 2 | Produces the artifact; the diagnostic reasoning is absent or wrong |
| 3 | Correct artifact with a correct account of why the fault occurred |
| 4 | As 3, plus the student identified a limitation or edge case not asked for |

Level 4 is deliberately reachable by curiosity rather than by extra volume.

## 8.3 Decision D16: the viva is the real assessment

Students will use language models to generate ROS code, and no plagiarism
detector meaningfully addresses this. The workable response is to move assessment
to something generation does not help with.

Five minutes per student at each project demonstration:

1. One question from a published bank of about forty, so preparation is possible
   and useful
2. One parameter justification drawn from their own submitted configuration:
   "your inflation radius is 0.30, the robot footprint is 0.22, explain the
   choice"
3. One live fault, introduced by the TA in front of them, which they diagnose
   aloud using the standard workflow

The third item is the one that decides. A student who cannot begin with
`ros2 topic list` did not build the system, and it takes ninety seconds to find
out. It also assesses precisely the skill the course claims to teach, which makes
it defensible if challenged.

## 8.4 Decision D17: peer contribution factor

Uneven effort within groups is the most common and most legitimate student
complaint about project based modules.

A multiplier between 0.8 and 1.1 applied to the individual's project marks,
determined by the TA from three inputs: an anonymous peer form, the Git commit
history over the project window, and the individual viva. The commit history is
evidence rather than a metric, since commit counts are trivially gamed, but a
student with no commits and a weak viva is a clear case.

Publish the mechanism in week one. Its main effect is preventive.

## 8.5 Decision D18: declared AI use

Permitted, with a required `AI_USE.md` in every submitted repository stating what
was generated, which tool, and what was changed afterwards. Undeclared use is an
academic integrity matter; declared use is not.

The viva is what makes this enforceable without surveillance. A student who
declares heavy assistance and can explain and debug the result has demonstrated
competence. A student who declares nothing and cannot explain their own code has
a problem that the declaration policy did not create.

This is worth writing into the proposal explicitly. A module that pretends the
tools do not exist reads as dated to a professor in 2026.

## 8.6 Decision D8 applied: infrastructure protection

Stated in the module handbook: **no mark is lost due to a failure of the official
environment.**

The mechanism: `course-check` fails, prints an incident ID, the student shows the
TA, the TA records the ID and either reallocates a machine or grants a 48 hour
extension on that week's exit task. The incident log is also your evidence base
when arguing for better lab hardware.

Without an artifact this promise is unenforceable and becomes a source of
disputes. With one it is a two minute process.

---

# 9. Project 1: Autonomous Mapping and Navigation System

Scope unchanged from the original specification, with three additions that follow
from the decisions above.

Deliverables: a mapped environment with the saved map committed, a localisation
configuration, a multi goal mission node using the Nav2 action interface, handling
of one unannounced obstacle, a clean package structure, a Git history showing
distributed contribution, a demonstration and a four page report.

Additions:

1. The multi goal mission must be evaluated over 30 seeds through `arc_eval`, and
   the report must present the aggregate table rather than a description of one
   successful run.
2. The report must include one honest failure analysis. A team that reports no
   failures across 30 seeds has either not run 30 seeds or has not looked.
3. Individual viva per section 8.3.

Marking split: system function 30, ROS engineering and package structure 20,
experimental method and the seeded evaluation 20, report and technical
communication 20, viva 10. Multiplied by the peer factor.

---

# 10. Project 2: the Grand Challenge

## 10.1 Decision D19: resolving the map versus explore ambiguity

The original specification says teams start from an unknown position in an unseen
arena. That admits two very different competitions. If teams must explore
autonomously, they need frontier exploration, which the curriculum never teaches.
If they are handed a map, "unknown position" means only global localisation.

Resolution: each team gets a **ten minute unscored commissioning run** in the
competition arena immediately before their scored attempt. During it they may
drive the robot, run SLAM Toolbox and save a map. The scored run then starts from
an unknown pose with that map available.

This is the right answer for three reasons. It mirrors how commercial AMR
deployments actually work, since no warehouse robot is dropped into an unmapped
building and expected to perform. It keeps AMCL and everything in Lab 5 directly
relevant to the final assessment. And it removes an untaught capability from the
critical path without making the challenge trivial, because the dynamic
obstacles, the blocked route and the unseen layout all remain.

Teams that prefer to skip the commissioning run and navigate reactively may do
so, and that becomes an interesting strategic choice rather than a rule.

## 10.2 Decision D20: the published arena grammar

The arena is unseen but not arbitrary. The grammar is published in week nine and
implemented in `arc_rl/arenas.py`, so a single seed integer fully identifies an
arena for grading and for dispute resolution.

| Parameter | Range |
|-----------|-------|
| Arena footprint | 10 to 14 m square |
| Internal partitions | 2 to 4, each with one doorway |
| Doorway width | 0.9 to 1.6 m |
| Static obstacles | 8 to 16, radius 0.15 to 0.45 m |
| Dynamic obstacles | 2 to 3, speed 0.20 to 0.40 m/s, radius 0.25 m |
| Checkpoints | 4, minimum separation 3 m |
| Time limit | 120 s of simulated time per attempt |

Every generated arena is validated for reachability by breadth first search on an
occupancy grid inflated by the robot footprint radius **plus a 0.15 m clearance
margin**. Validating at the bare footprint only proves a nearly point-sized robot
could pass, and two partitions falling close together then produce a route that
is technically reachable and unusable in practice. This matters practically, since an
unwinnable competition arena would be discovered during the event, and it matters
pedagogically, since it is the same BFS students implement in the planning
notebook, reused as a production validation step.

Training arenas are drawn from a deliberately easier grammar. Students should see
a generalisation gap rather than failing uniformly everywhere.

## 10.3 Decision D21: run protocol

Ambiguity in competition rules produces arguments, and arguments during a public
event are worse than slightly wrong rules stated clearly in advance.

- Two scored attempts per team on different seeds; the better attempt counts
- An intervention is any physical or software action by a team member after the
  start signal, including restarting a node
- Maximum two interventions; the third ends the attempt at its current score
- The clock does not stop during an intervention
- The robot is considered stuck after 20 s without 0.1 m of displacement, and a
  stuck robot may be recovered only through an intervention
- Seeds are drawn publicly at the start of the session

## 10.4 Leaderboard scoring

Separate from the academic mark, per the original specification.

```
score = max(0,
            100 x checkpoints_reached
          + 100 if all checkpoints reached
          + 100 x max(0, (T_limit - T_used) / T_limit)      if completed
          + 100 x min(1, optimal_path_length / actual_path) if completed
          -  75 x static_collisions
          - 150 x dynamic_collisions
          -  20 x clearance_violations
          - 150 x interventions )
```

These weights were corrected during validation. The first version used a time
bonus of 200 and a static collision penalty of 40, and under it a run finishing
in 40 s with two collisions scored 653 against 617 for a clean run finishing in
110 s, which is the opposite of the stated intent. `arc_eval/scoring.py`
implements the corrected version and
`test_safe_and_slow_beats_fast_and_reckless` guards the ordering:

    completing the mission  >  not hitting things  >  efficiency  >  speed

The time and efficiency bonuses are earned only by a completed mission. Without
that condition a robot that gives up immediately collects the full time bonus for
having used no time, which is the first thing a good team would try.

The optimal path length is computed by the same BFS used for arena validation, so
the efficiency term is grounded rather than relative to whichever team happened
to do well.

Note the deliberate structure: completing the mission slowly and safely beats
completing it quickly with two collisions. That ordering should be stated to
students, because it tells them what to optimise and it reflects what the
industry actually values.

## 10.5 Awards and the dynamic obstacle risk

Grand Champion, Best Navigation, Best AI Solution, Most Robust Robot, Best
Engineering Design. These cost nothing and carry the motivational load that
section 8.1 deliberately removed from the marks.

One technical risk to retire early: actor and scripted trajectory support in
Gazebo Harmonic differs from Gazebo Classic, and most available tutorials target
Classic. Prototype the dynamic obstacles in week one of development, not week
eleven. The fallback if actors prove unreliable is a second instance of the
reference robot driven by a scripted node, which is less elegant and entirely
adequate.

---

# 11. Repository and authoring

## 11.1 Layout

```
arc-course/
  docs/
    00_design_baseline.md          this document
    adr/                           architecture decision records
    labs/lab01/ ... lab10/         manual sources in Markdown
    annex/research_frontier.md     refreshed annually, see D23
    templates/arc-reference.docx   pandoc style reference
  arc_description/                 URDF, Xacro, meshes
  arc_bringup/                     launch files, parameter YAML
  arc_gazebo/                      worlds, models, arena generator output
  arc_nav/                         Nav2 configurations, behaviour trees
  arc_rl/
    nav_core.py                    the shared contract
    arenas.py                      the published grammar
    baselines.py                   classical floor
    ros_nav_env.py                 rclpy backed environment
  arc_eval/
    runner.py                      the harness
    configs/                       one YAML per benchmarked system
  scripts/
    course-check
    course-smoke-test
  starters/lab01/ ... lab10/       student starting points with TODO markers
  tests/                           pytest and launch_testing
```

## 11.2 Decision D22: Markdown to DOCX

Authoring ten code heavy manuals directly in Word is a mistake that becomes
visible at revision three. Word documents cannot be diffed, code blocks lose
their formatting under editing, and there is no way to verify that the code in
the manual matches the code in the starter package.

Author in Markdown under version control and generate DOCX with pandoc against a
reference document that carries the styling:

```bash
pandoc docs/labs/lab03/manual.md \
  --reference-doc=docs/templates/arc-reference.docx \
  --toc --toc-depth=2 \
  --highlight-style=tango \
  -o build/Lab03_Gazebo_Sensors_and_Robot_Data.docx
```

The reference document is built once by exporting a DOCX, editing the `Source
Code` character style to a monospace font with a light background, and adjusting
the heading styles to the university template. Every generated manual then
inherits it.

The decisive advantage is that code in the manual can be included from the
starter package by the build, rather than copied. A code cell that has drifted
from the file students actually run is the most damaging error a lab manual can
contain, and this eliminates the possibility structurally.

The DOCX generated from this document, included alongside it, is the pipeline
working.

## 11.3 Decisions D23 and D24: research content in an annex

Two problems with a research frontier box in all ten manuals. The content dates
within eighteen months, and forcing recent papers into introductory topics
produces exactly the decorative citations the original specification says to
avoid. Breadth first search does not have a 2025 frontier worth a student's
attention.

Resolution: research frontier boxes in Labs 5, 6, 7, 9 and 10, plus the planning
notebook, where the frontier is genuine. Labs 1, 2, 3 and 8 carry an engineering
practice box instead, covering things like DDS and middleware selection, hardware
abstraction across CAN and EtherCAT, telemetry and offline reproduction, and
environment determinism. That content is equally industry relevant and it does
not expire.

All frontier text lives in `docs/annex/research_frontier.md` and is included into
the manuals at build time, so the annual refresh is one file rather than six.

## 11.4 Decision D25: tests students run themselves

With the cohort size in assumption A3 and a small TA team, weekly marking is only
sustainable if the mechanical checks are automated.

Each starter package ships with a `tests/` directory the student runs before
submitting. `pytest` covers the pure Python components, and `launch_testing`
covers the ROS integration: node comes up, topic publishes at the expected rate,
TF resolves, action succeeds.

This is not only a marking efficiency measure. Giving students a failing test to
make pass is a better exercise specification than a paragraph of prose, and
`launch_testing` is a genuine industry skill that most graduates have never seen.

The TA then assesses judgement, the viva and the report, which is where human
attention is actually worth something.

---

# 12. Curriculum content fixes

## 12.1 Decision D26: giving C++ a concrete home

The specification promises 10 to 15 percent C++ exposure, but no lab in the
original outline contains a C++ task. As written the promise is aspirational,
and a professor reading the proposal will notice.

| Session | C++ activity | Type |
|---------|--------------|------|
| Lab 2 | Modify a supplied `rclcpp` node, rebuild with `colcon`, run it | Build, graded |
| Lab 2 | Read a `ros2_control` hardware interface implementation | Read only |
| Lab 6 | Read a Nav2 controller plugin header and its `pluginlib` export | Read only |
| Lab 7 | Read a behaviour tree action node | Read only |

The single build task in Lab 2 is the important one. It forces students through
`CMakeLists.txt`, `package.xml` dependencies and a compile error exactly once,
which is enough for them to recognise the workflow later without turning the
module into a C++ course. The modification should be small and meaningful:
changing a subscribed topic name and a declared parameter, so that both a build
step and a runtime configuration step are exercised.

## 12.2 Launch files, parameter YAML and costmap layers

These appear in the industry readiness list but were never allocated to a
session, and they are among the most used skills in the entire stack.

Launch files and parameter YAML become a running thread: Lab 1 runs a supplied
launch file, Lab 2 modifies one, Lab 3 writes one composing two nodes, Lab 6
onward configures large parameter files. No dedicated session needed.

Costmap layers, static, obstacle, inflation and voxel, need explicit time in Lab
6. Layer configuration is the single most tuned thing in deployed Nav2 systems
and it is where the Lab 4 occupancy grid work connects to production reality.
Budget twenty minutes of the Lab 6 build for an inflation radius experiment,
benchmarked through `arc_eval`, because the effect on path shape and clearance is
large, immediate and visually obvious.

## 12.3 Decision D27: lifecycle nodes earlier

Managed lifecycle is listed in Lab 7 but it underpins every Nav2 server from Lab
6 onward. A student who first meets `configure` and `activate` in week eight has
spent two weeks not understanding why `ros2 lifecycle list` matters or why a node
that is running is not necessarily working.

Introduce the concept in Lab 1 alongside the node graph, with one short exercise
transitioning a supplied managed node and observing what does and does not
publish in each state. Ten minutes in Lab 1 saves confusion in three later
sessions.

## 12.4 Decision D28: safety standards in Lab 7

Add a short section to the Lab 7 industry perspective covering ISO 3691-4, the
safety standard for driverless industrial trucks, and the role of safety rated
sensing and protective stops in deployed AMRs. Connect it directly to the Nav2
Collision Monitor the students are configuring, and be precise about the
distinction: the Collision Monitor is a software safeguard within the autonomy
stack, not a safety rated protective device in the sense the standard means.

This costs ten minutes and it is the kind of detail that distinguishes a
curriculum written by someone who has looked at industrial robotics from one
assembled from documentation.

## 12.5 Decision D29: splitting Lab 5

Lab 5 as originally scoped is three sessions of content. The split follows the
new twelve week map:

**Lab 4, Occupancy Mapping.** Students implement log odds occupancy grid updates
with an inverse sensor model, applied to a rosbag recorded in Lab 3. Using a
replayed bag rather than a live simulation makes the exercise deterministic, which
means it is fairly gradeable and reproducible on any machine including Tier C.
It also gives the Lab 3 recording an evident purpose, which strengthens the
narrative continuity the specification asks for.

**Lab 5, Localisation and SLAM.** Particle filter intuition delivered in the pre
lab reading, then AMCL configuration, SLAM Toolbox mapping, map saving, restart,
relocalisation, and a comparison of raw odometry drift against the localised
pose. The EKF discussion attaches to `robot_localization` where it is concrete
rather than being taught abstractly.

Lab 2 has a similar problem. Kinematics and TF2 and URDF and Xacro and
ros2_control is a great deal for one session. The mitigation without spending a
slot: kinematics moves entirely into the pre lab reading with worked examples,
since it is arithmetic that students can do at home, and the session focuses on
the robot description, the TF tree, and diagnosing a deliberately broken
transform.

---

# 13. ILO mapping

For a professor facing proposal at a UK model institution, the section that
converts a good idea into an approvable one is the mapping from lab content to
module intended learning outcomes and, where the programme is accredited, to the
relevant engineering learning outcome framework.

Template to complete from the module specification:

| Lab | Module ILO | Assessment evidence | Framework outcome |
|-----|-----------|---------------------|-------------------|
| 1 | ILO n | Exit task, live diagnosis | |
| 2 | | Exit task, C++ build task | |
| ... | | | |
| Project 2 | | Report, seeded evaluation, viva | |

Fill the framework column from your programme's accreditation documentation
rather than from a general source, since the codes and wording differ by scheme
and by version. If the programme is not accredited, the module ILO column alone
is sufficient and the table still demonstrates deliberate coverage.

I would also add a short table showing which weeks assess which of the twelve
capability claims in section 20 of the original specification. It closes the loop
between what the proposal promises and what the assessment actually measures,
which is the first thing a sceptical reader checks.

---

# 14. Build plan

## 14.1 Decision D30: one vertical slice first

The honest estimate for the full deliverable set is 250 to 400 hours. Building it
in the order of the deliverable list means writing ten manuals against an
unvalidated environment.

Build Lab 6 end to end first. It is the densest integration point in the course,
exercising Gazebo, the reference robot, RViz, Nav2, costmap configuration,
`arc_eval` and the pandoc pipeline simultaneously. A working Lab 6 proves the VM,
the robot, the harness, the manual style and the build chain in one pass, and it
is the artifact to show your professor before committing the remaining effort.

## 14.2 Sequence

| Stage | Output | Blocking risk retired |
|-------|--------|-----------------------|
| 1 | VM built, `course-check` passing on the weakest lab PC in all three tiers | Graphics and hardware |
| 2 | Reference robot in Gazebo, TF valid, `/cmd_vel` moves it | Robot model |
| 3 | Nav2 reaching goals, `arc_eval` producing a report | Integration |
| 4 | Lab 6 manual complete, DOCX generated, reviewed by professor | Style and pipeline |
| 5 | Labs 1, 2, 3 written and validated | Front half |
| 6 | Labs 4, 5, Project 1 spec | Mapping and localisation |
| 7 | ROS backed RL environment, policy transfer verified | The largest technical risk |
| 8 | Labs 7 to 10 | Back half |
| 9 | Competition arena generation, dynamic obstacles, rehearsal run | Event risk |
| 10 | Professor facing proposal assembled from the above | |

Stage 7 should be attempted early in parallel rather than in sequence. If policy
transfer from the surrogate to Gazebo proves worse than expected, the mitigation
is more surrogate fidelity or heavier domain randomisation, and finding that out
in month one is very different from finding it out in month four.

## 14.3 Minimum viable first offering

If the timeline compresses, the first offering that is still worth running:
ten labs delivered with six fully polished manuals and four in a lighter format,
the VM and harness complete, both projects run, and the competition treated
openly as a pilot. Collect the incident log, the seeded results and student
feedback, then invest the polish in year two where the data says it is needed.

Running a slightly rough version of this curriculum is considerably better than
running the current one, and the data from a real cohort will improve the second
version more than another hundred hours of writing would.

---

# 15. Open risks

| Risk | Impact | Mitigation | Owner |
|------|--------|-----------|-------|
| Gazebo Harmonic unusable under virtualised OpenGL | High | Tier B and C fallbacks, hypervisor comparison, early test on weakest PC | Stage 1 |
| Surrogate to Gazebo policy transfer poor | Medium | Domain randomisation, added surrogate fidelity, hybrid fallback for Project 2 | Stage 7 |
| Dynamic obstacles unreliable in Harmonic | Medium | Scripted second robot as fallback | Stage 9 |
| Lab PC RAM insufficient for the VM allocation | High | Confirm A4 before freezing the VM specification | Stage 1 |
| Appliance distribution saturates the campus network | Medium | Technician pre imaging, local mirror, USB channel | Stage 1 |
| Authoring effort exceeds available time | High | Vertical slice first, minimum viable first offering | Stage 4 |
| TA capacity insufficient for vivas at cohort size | Medium | Confirm A3; vivas can be batched two students at a time if needed | Before week 6 |

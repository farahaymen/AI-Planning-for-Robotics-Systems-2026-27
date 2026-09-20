---
title: "Lab 5: Localisation and SLAM"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 5: Localisation and SLAM

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1 (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic)
**Packages** `slam_toolbox`, `nav2_amcl`, `robot_localization`, `nav2_map_server`, `arc_lab5`

**Prerequisites**

Lab 4, including your working occupancy map and the bag it was built from.
`starters/lab05/particle_filter_skeleton.py` should be read before the session.

---

## Before the session

### Why this lab matters

Your map from last week has doubled walls. Not because your sensor model was
wrong, but because every scan was placed using a pose that had drifted.

That single observation contains the whole of this lab. Odometry integrates wheel
rotation, wheel rotation is measured imperfectly, and integrating an imperfect
measurement accumulates error without bound. After thirty metres of driving, a
robot with excellent encoders can easily be half a metre and several degrees from
where it believes it is, and nothing about that error announces itself.

There are two responses. If you already have a map, compare what you see against
what the map predicts and correct the estimate. That is localisation, and AMCL
does it. If you do not have a map, estimate the map and the trajectory
simultaneously, treating the poses as unknowns to be optimised rather than as
given. That is SLAM, and SLAM Toolbox does it. You will use both today.

### Why odometry drift is unfixable rather than merely large

It is tempting to think a better encoder solves this. It does not, and
understanding why is the point.

Odometry is an integral. Each measurement has a small error, and integrating adds
those errors together. There is no averaging effect, because the errors
accumulate in the state rather than in a repeated observation of the same
quantity. Position error grows roughly with distance travelled, and heading error
is worse, because a small heading error rotates all subsequent motion.

The wheel radius exercise from Lab 2 is the deterministic version of the same
thing. There the error was systematic and you could measure and remove it. What
remains after that is random, and it cannot be removed by calibration.

The consequence for architecture is the `map` to `odom` transform you met in Lab
2. Odometry gives a smooth, continuous, drifting estimate. Localisation gives a
correct but discontinuous one. Rather than choosing, ROS keeps both and publishes
the correction between them.

### The Bayes filter

Localisation is a recursive estimation problem with two alternating steps.

**Predict.** The robot moved. Apply the motion model to the current belief, which
makes the belief less certain, because motion adds noise.

**Update.** The robot observed something. Compare the observation against what
each hypothesis predicts, and weight hypotheses by how well they agree, which
makes the belief more certain.

Everything in localisation is a choice of how to represent the belief and how to
implement those two steps.

An **extended Kalman filter** represents the belief as a Gaussian, so the whole
belief is a mean and a covariance. This is compact and fast, and it fails when
the true belief is not shaped like a Gaussian. A robot that could be in either of
two identical corridors has a two-peaked belief, and a Gaussian forced onto it
puts its mean in the wall between them.

A **particle filter** represents the belief as a set of weighted samples. It can
represent any shape, including two peaks, at the cost of needing many samples.
This is what AMCL uses, and it is why AMCL can perform global localisation while
an EKF cannot.

In this course you use both, for different jobs. `robot_localization` runs an EKF
to fuse wheel odometry with the IMU, producing a better `odom` estimate. AMCL
runs a particle filter on top of that to produce the `map` correction. They are
not competitors; they sit at different levels.

### Monte Carlo localisation in three steps

Read `starters/lab05/particle_filter_skeleton.py` before the session. It is the
file you edit. `predict`, `update_weights` and `resample` are removed and what
belongs in them is marked with nine TODOs, which implement exactly the three
steps below. `starters/lab05/particle_filter.py` is the reference
implementation, 187 lines. Leave it closed until your own tests pass, then read
it and compare.

**Predict.** Each particle is moved by the odometry increment plus noise sampled
from the motion model. The noise is what matters. Without it, every particle
follows an identical trajectory, the cloud never explores alternatives, and the
filter cannot recover from being wrong.

The motion model is parameterised by four alphas, which are the same
`alpha1` to `alpha4` you will set in `amcl.yaml`. They describe how much rotation
noise comes from rotating, how much rotation noise comes from translating, and
the two translation equivalents. Tuning them without knowing what they mean is
the usual reason AMCL misbehaves.

**Update.** Each particle predicts what the LiDAR should see from its pose,
compares against the actual scan, and is weighted by the agreement. Doing this by
ray casting for every beam and every particle is expensive, so AMCL uses a
likelihood field: precompute the distance from every map cell to the nearest
obstacle, then score a beam endpoint by looking up that distance. It turns a ray
cast into an array lookup.

**Resample.** Particles are redrawn in proportion to their weights, so poor
hypotheses die and good ones multiply.

Two details in resampling are worth knowing because they are the difference
between a filter that works and one that quietly fails.

Resampling should not happen every step. Each resample discards diversity, and
doing it when the weights are nearly uniform discards diversity for no
information gained. The standard test is the effective sample size,
$1 / \sum w_i^2$, which equals the particle count when weights are uniform and
falls towards one when a single particle dominates. Resample when it drops below
about half the particle count.

Low variance resampling draws one random number rather than $n$, stepping through
the cumulative weights at even intervals. It preserves diversity better than
independent draws and runs in linear time. AMCL uses it, and so does the supplied

![Global localisation with 600 particles and no initial pose estimate, produced by `particle_filter.py`.](docs/figures/lab05_particle_convergence.png){width=100%}


implementation.

### From localisation to SLAM

Localisation assumes a map. SLAM does not, and instead estimates the map and the
trajectory together.

Modern 2D SLAM, including SLAM Toolbox, is pose graph based. Each robot pose is a
node. Edges are constraints: consecutive poses are constrained by odometry, and
poses that observed the same place are constrained by **scan matching**, which
finds the relative transform that best aligns two scans.

The important edge is the **loop closure**. When the robot returns somewhere it
has been, scan matching produces a constraint between two poses that are far
apart in time. The optimiser then distributes the accumulated drift around the
whole loop rather than leaving it at the end. Watching a map snap into alignment
at the moment a loop closes is the most satisfying thing in this course, and you
should try to make it happen today.

---

> ### Industry Perspective: the three packages and why all three exist
>
> A deployed AMR runs all three of the packages in this lab, and people new to
> the stack often think two of them are redundant.
>
> `robot_localization` fuses wheel odometry with an IMU using an EKF, publishing
> a better `odom` to `base_link` transform. The IMU measures rotation directly
> rather than inferring it from a wheel speed difference, so it corrects exactly
> the heading error that hurts most. This runs continuously and is invisible when
> working.
>
> `slam_toolbox` builds the map. In production this typically happens once, at
> commissioning, when an engineer drives the robot around a new site. It also has
> a localisation mode and a lifelong mapping mode, and choosing between mapping
> once and mapping continuously is a real deployment decision. Continuous mapping
> adapts to a warehouse that gets rearranged, and it also lets a bad day slowly
> corrupt a good map.
>
> `nav2_amcl` localises against the saved map during normal operation. It is
> cheaper than running SLAM continuously and it cannot corrupt the map, which is
> why most deployments prefer it.
>
> The commissioning workflow is worth knowing because it is exactly what Project
> 2 asks you to do: drive the site, build a map, check it, save it, then run
> localisation against it thereafter.

> ### Research Frontier: what a map is for
>
> The occupancy grid you built in Lab 4 is a statement about which cells are
> free. That is enough for a robot that only needs to avoid walls, and it throws
> away almost everything a camera or a modern LiDAR could tell you.
>
> Several active lines of work replace it. **Semantic mapping** labels regions
> with what they are rather than only whether they are occupied, so a robot can
> be told to go to the kitchen rather than to coordinates. **Neural implicit
> representations** and, more recently, **3D Gaussian splatting** store a scene
> as parameters of a continuous function rather than as a discretised grid,
> giving photorealistic reconstruction and continuous surfaces at the cost of
> interpretability and predictable memory use. **Lifelong and collaborative
> mapping** address maps that must survive months of change or be built by
> several robots at once.
>
> None of these has replaced the occupancy grid in production navigation, and the
> reasons are instructive. A grid has bounded memory, is trivially inspectable by
> an engineer standing in a warehouse, degrades predictably, and can be checked
> against a floor plan. Those properties matter more to a company shipping robots
> than reconstruction quality does.
>
> The question to bring to any paper in this area, and the one to ask about your
> own Project 2 choices, is what the new representation gives up. It is usually
> interpretability or worst case behaviour, and both are expensive to lose.

---

### Pre-lab quiz

Five questions covering why odometry drift is unbounded, what the `map` to `odom`
transform expresses, why a particle filter can globally localise and an EKF
cannot, what the effective sample size measures, and what a loop closure
constrains.

---

## In the session

### Stage 0: health check (5 minutes)

```
course-check
```

### Exercise 5.1: measure the drift (15 minutes)

Before fixing something, measure it. Gazebo can publish ground truth, which the
robot does not have access to but you do.

```
ros2 launch arc_gazebo simulation.launch.py world:=arc_warehouse
ros2 run arc_lab5 drift_meter
```

Drive a loop that returns the robot to its starting point, then read the report.

| Distance driven (m) | Final odometry error (m) | Final heading error (deg) | Error as % of distance |
|---------------------|--------------------------|---------------------------|------------------------|
| 5 | | | |
| 15 | | | |
| 30 | | | |

The percentage column is the interesting one. If it stays roughly constant, the
error is proportional to distance, which is what an accumulating random error
looks like. If it grows, something systematic is present as well, and Lab 2 tells
you where to look.

**[GIF PLACEHOLDER]**
RViz with the fixed frame set to `odom`, showing the robot's odometry path
diverging from the ground truth path over a full loop.
*Instructor note: display both paths in contrasting colours and record about 30
seconds covering one complete loop. The moment of return to the start, with a
visible gap between the two paths, is the shot that matters.*

### Exercise 5.2: watch a particle filter converge (20 minutes)

Complete `particle_filter_skeleton.py` and run it against your Lab 4 map,
offline, so you can watch the mechanism rather than fight the simulator. Fill in
the nine TODOs in `predict`, `update_weights` and `resample`, and run the tests
as you go. The `ARC_PARTICLE_FILTER` variable is what points the tests at your
file rather than at the reference:

```
cd ~/arc_ws/src/arc-course
ARC_PARTICLE_FILTER=particle_filter_skeleton python3 -m pytest starters/lab05 -q
```

Without that variable the tests import `particle_filter.py`, the reference
implementation, and they all pass without you having written anything. Once your
own version passes, read `particle_filter.py` and compare it against what you
wrote.

**Code 5.1: Global localisation from a uniform prior**

```python
import numpy as np
from particle_filter_skeleton import ParticleFilter, odometry_increment
from likelihood_field import LikelihoodField        # supplied

field = LikelihoodField.from_map("my_map.yaml", sigma=0.2)
pf = ParticleFilter(n_particles=2000, seed=0)

# Global localisation: the robot has no idea where it is. This is the hard case
# and the one an EKF cannot represent, because the belief is spread over the
# whole building rather than concentrated anywhere.
pf.seed_uniform(field.bounds)

previous = None
for step, (stamp, scan, odom_pose) in enumerate(run):
    if previous is not None:
        pf.predict(*odometry_increment(previous, odom_pose))
    previous = odom_pose

    # One number per particle: how badly its predicted scan disagrees with the
    # real one. Computing it is the measurement model's job and is supplied.
    errors = field.scan_errors(pf.particles, scan)
    pf.update_weights(errors, sigma=0.3)

    resampled = pf.maybe_resample(threshold_ratio=0.5)

    if step % 10 == 0:
        x, y, theta = pf.estimate()
        print(f"step {step:4d}  estimate ({x:6.2f}, {y:6.2f}, {np.degrees(theta):6.1f})  "
              f"spread {pf.spread():5.2f} m  ESS {pf.effective_sample_size():7.1f}  "
              f"{'resampled' if resampled else ''}")
```

Watch three numbers. The spread should fall from several metres to a few
centimetres. The effective sample size should drop sharply, trigger a resample,
and recover. The estimate should jump around early and then settle.

Record the step at which the spread first drops below 0.5 m, and note what the
robot was doing at that moment. Convergence usually happens at a distinctive
feature such as a corner or doorway, and almost never in a plain corridor. That
observation is the entire practical content of the perceptual aliasing problem.

Then try the harder cases:

| Case | Steps to converge | Converged correctly? |
|------|-------------------|----------------------|
| Global, 2000 particles | | |
| Global, 200 particles | | |
| Global, 200 particles, `alpha` all 0.0 | | |
| Seeded near the true pose, 200 particles | | |

The third row should fail. With no motion noise the particles never explore, so
if none of the initial samples was near the truth the filter can never find it.
This is particle deprivation, and it is why the noise parameters are not
optional.

### Exercise 5.3: SLAM Toolbox (25 minutes)

Now the production tool. Run it against the same bag, so the comparison with your
Lab 4 map is fair.

```
ros2 launch arc_lab5 slam_from_bag.launch.py bag:=~/arc_ws/bags/lab03_mapping_run
rviz2 -d $(ros2 pkg prefix arc_nav)/share/arc_nav/rviz/slam.rviz
```

Watch the pose graph build in RViz. When the robot returns to a previously
visited area, look for the moment the map shifts as a loop closure is applied.

Save the map:

```
ros2 run nav2_map_server map_saver_cli -f ~/arc_ws/maps/slam_map
```

**[GIF PLACEHOLDER]**
The map immediately before and after a loop closure, showing the doubled corridor
walls snapping into alignment.
*Instructor note: this is the single most valuable capture in the course. Record
the pose graph display alongside the map. If loop closure does not trigger
reliably on the reference bag, record a purpose-made run in which the robot
traverses a loop twice, and note the bag name here.*

Compare the two maps:

| | Your Lab 4 map | SLAM Toolbox map |
|--|----------------|------------------|
| Wall thickness at the far end of the run (cells) | | |
| Doubled walls present? | | |
| Corridor width at the doorway (m) | | |
| Time to build | | |

The difference between these two columns is what optimising over poses buys you,
and it is worth being able to state in one sentence for your Project 1 report.

### Exercise 5.4: localise on the saved map (20 minutes)

Restart the simulator with the robot in a position it does not know, load the map
and run AMCL.

```
ros2 launch arc_lab5 localization.launch.py map:=~/arc_ws/maps/slam_map.yaml
```

In RViz, publish an initial pose estimate that is deliberately about a metre
away from the truth, then drive. Watch the particle cloud contract.

Then compare the two estimates directly:

```
ros2 run tf2_ros tf2_echo map base_footprint     # localised
ros2 topic echo /odom --field pose.pose.position  # raw odometry
```

Drive a long loop and record both at the end.

| | Raw odometry | AMCL estimate | Ground truth |
|--|--------------|---------------|--------------|
| Final x (m) | | | |
| Final y (m) | | | |
| Final heading (deg) | | | |
| Error from truth (m) | | | |

Finally, the kidnapped robot. Ask your demonstrator to teleport the robot to a
different part of the map. AMCL will not recover with default settings, because
all its particles are concentrated somewhere else. Note what the particle cloud
does, and read the `recovery_alpha_slow` and `recovery_alpha_fast` parameters in
`amcl.yaml` to see what would be needed. You do not need to make it work; you
need to be able to explain why it does not.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your drift measurement table.
2. Your completed `particle_filter_skeleton.py` with all tests passing under
   `ARC_PARTICLE_FILTER=particle_filter_skeleton`.
3. Your particle filter convergence table, including the failed no-noise case,
   and the step and location at which the 2000 particle run converged.
4. Your saved SLAM map, and the comparison table against your Lab 4 map.
5. Your AMCL versus odometry table.
6. Two sentences on the kidnapped robot: what you observed and why.

---

## Troubleshooting

**SLAM Toolbox produces a map that rotates or spirals.**

The scan matcher is failing and the pose graph is being driven by odometry alone.
Check that `/scan` and `/odom` timestamps are consistent and that
`use_sim_time` is true everywhere, which is the Lab 3 lesson returning.

**AMCL particles never converge.**

Either the initial pose was too far from the truth for the cloud to cover it, or
the map does not match the environment, or `laser_max_range` in `amcl.yaml`
disagrees with the actual sensor. Check the last one first; it is quick and it is
often the answer.

**AMCL converges on the wrong place and stays there.**

Perceptual aliasing. The robot is somewhere that looks like somewhere else.
Drive to a distinctive feature and watch whether it corrects. If it does not, the
filter has already discarded the correct hypothesis and only global relocalisation
will recover it.

**The robot's position jumps in RViz.**

Expected. The `map` to `odom` transform is discontinuous by design, because it
carries the correction. If the jumps are large and frequent the filter is
struggling; if they are small and occasional it is working.

**No `map` to `odom` transform is published.**

Neither AMCL nor SLAM Toolbox is active. Both are lifecycle nodes.

```
ros2 lifecycle get /amcl
ros2 run tf2_tools view_frames
```

**`robot_localization` makes things worse.**

Check the IMU sign conventions and that you have not configured it to fuse the
same measurement twice, which makes the filter overconfident. Fusing wheel yaw
and IMU yaw as independent measurements is a common and damaging mistake.

---

## Connection to Project 1

You now have every component of a conventional autonomous robot except the
decision to go somewhere.

The robot can build a map of an unknown space, save it, restart, work out where
it is on that map without being told, and keep that estimate correct while it
drives. What it cannot yet do is choose a route and follow it, which is Lab 6.

Project 1 comes first, and it is deliberately placed before Nav2 rather than
after. You will assemble what you have into a working system: map an environment,
save it, localise on it, and drive to a sequence of goals. You may use the Nav2
action interface for the driving, but the mapping, localisation and mission logic
are yours, and the report has to explain the choices rather than describe the
steps.

The specification is a separate document. Read it this week rather than next,
because the environment you choose to map affects how much time the rest takes.

---

## References

**Textbooks**

Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
Chapter 4 covers particle filters, chapter 7 Monte Carlo localisation, and
chapter 11 the graph based SLAM formulation SLAM Toolbox implements. This lab is
essentially a tour of that book.

Grisetti, G., Kümmerle, R., Stachniss, C. and Burgard, W. (2010). A tutorial on
graph based SLAM. *IEEE Intelligent Transportation Systems Magazine*, 2(4).
The clearest short explanation of pose graph optimisation available.

**Papers**

Macenski, S. and Jambrecic, I. (2021). SLAM Toolbox: SLAM for the dynamic world.
*Journal of Open Source Software*, 6(61).
The paper describing the package you used today, including its lifelong mapping
and localisation modes.

**Official documentation**

SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
The README documents the operating modes and the parameters that matter, and the
distinction between online, offline and lifelong mapping is explained there
better than anywhere else.

`nav2_amcl` configuration:
https://docs.nav2.org/configuration/packages/configuring-amcl.html
Read the `alpha1` to `alpha4` descriptions alongside the motion model in
`particle_filter.py`; they are the same four numbers.

`robot_localization`: https://docs.ros.org/en/melodic/p/robot_localization/
Dated documentation but still the authoritative description of the sensor fusion
configuration matrices.

**Video**

Cyrill Stachniss, Mobile Sensing and Robotics lecture series, University of Bonn,
on YouTube. The lectures on particle filters and on graph based SLAM are the best
freely available treatment of this material and follow the same notation as
*Probabilistic Robotics*.

---

## Further reading

`docs/references.md` has a fuller list under **Lab 5. SLAM and localisation**,
with papers, industry write-ups and the documentation worth keeping open. Every
entry says what you get from it and which part of the lab it connects to.

If you read one thing, read *Autonomous Navigation with LeKiwi and Nav2*
(Kamath, Foxglove, 2026). It maps a real robot with slam_toolbox and then
localises it two ways, slam_toolbox localization against AMCL, which is exactly
the comparison Exercise 5.4 asks you to make.

---
title: "Lab 3: Gazebo Simulation, Sensors and Robot Data"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 3: Gazebo Simulation, Sensors and Robot Data

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1 (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic)
**Packages** `arc_gazebo`, `arc_description`, `ros_gz_sim`, `ros_gz_bridge`, `rosbag2`, `rqt`, `rviz2`

**Prerequisites**

Labs 1 and 2. You need a working robot description and the diagnostic habits from
Lab 1, because today is largely about applying them to data that is genuinely
imperfect.

---

## Before the session

### Why this lab matters

Until now your robot has been a drawing. Today it acquires mass, friction and
sensors that lie to it.

Simulation is not a lesser version of a real robot. It is a different instrument
with its own failure modes, and the ones that catch people out are almost never
physics. They are timing, frames and data rates. A LiDAR that publishes at 3 Hz
instead of 10 will still produce a plausible looking map and a costmap that is
always slightly out of date. A sensor stamped with wall clock time while the
simulator runs on its own clock will produce transform lookups that fail with an
error message about extrapolation, and no amount of staring at the geometry will
help.

There is also a practical reason this lab is here rather than later. The recording
you make today is the input to the occupancy mapping exercise in Lab 4. Replaying
a bag makes that exercise deterministic, which means it is reproducible on any
machine, gradeable fairly, and completable even if Gazebo will not run at all on
your PC that week. A careless recording today makes next week harder.

### How the simulator connects to ROS

Gazebo Harmonic is a separate program with its own message system. It does not
speak ROS. The `ros_gz_bridge` translates between the two, and every topic that
crosses the boundary has to be declared.

```
Gazebo Harmonic                 ros_gz_bridge                ROS 2
  physics stepping    <---->    topic translation   <---->   your nodes
  sensor plugins                and type mapping             Nav2, RViz
  /world/.../clock                                           /clock
```

Two consequences follow. First, a topic that exists in Gazebo and is not in the
bridge configuration is invisible to ROS, and the symptom is a topic that simply
is not in `ros2 topic list`. Second, the bridge is a process, so it can be a
bottleneck, and a bridge struggling under load shows up as a sensor rate that is
lower than the sensor is actually producing.

### Simulation time

The simulator does not run at the speed of the wall clock. It runs as fast or as
slow as the physics allows, and inside a virtual machine that is usually slower.
The ratio is the real time factor, and you should expect something below 1.0 in
this course.

If some of your nodes measure time with the wall clock while the simulator runs
on its own clock, their notion of "now" diverges from the sensor timestamps, and
TF lookups start failing. The fix is a single parameter, `use_sim_time`, which
tells a node to take its time from the `/clock` topic that the simulator
publishes rather than from the operating system.

Every node in a simulated system must have `use_sim_time` set to true. Every one,
including RViz, including the ones you write, including anything you launch from
a terminal to check something. A single node that misses it produces errors that
appear to come from somewhere else entirely.

The characteristic symptom is worth memorising now, because you will see it:

```
Lookup would require extrapolation into the future.
Requested time 1731... but the latest data is at time 42.3
```

A requested time in the region of 1.7 billion is a Unix wall clock timestamp. A
latest data time of 42.3 is seconds since the simulation started. Seeing those
two numbers side by side tells you immediately that one node is on the wrong
clock, and which side is which.

### Sensor models

The LiDAR in your description is a `gpu_lidar` publishing 360 samples over a full
circle at 10 Hz, with a range of 0.12 to 12.0 metres and Gaussian noise with a
standard deviation of 0.01 m. Those numbers are course constants and match the
values in `arc_rl/nav_core.py`.

Three properties of the model matter downstream.

The **minimum range** exists because a real LiDAR cannot measure closer than its
optics allow. Returns below it are reported as invalid, and code that treats
invalid returns as zero distance will conclude the robot is permanently in
collision. Your Lab 6 code filters them explicitly for this reason.

The **noise** is why a map built from a single scan looks fuzzy and a map built
from many scans looks sharp. Occupancy mapping in Lab 4 is essentially a
principled way of averaging that noise away.

The **rate** determines how stale the costmap can be. At 10 Hz, a robot moving at
0.5 m/s travels five centimetres between scans, which is fine. At 2 Hz it travels
twenty five centimetres between scans, which is not.

![Three sensor streams, all averaging about 10 Hz. Statistics computed by `rate_stats.analyse`.](docs/figures/lab03_rate_diagnostics.png){width=95%}


### rosbag2

A bag records messages with their timestamps and replays them later with the same
relative timing. It is the closest thing robotics has to a reproducible
experiment.

This matters more in this field than in most. A robot failure is often not
reproducible on demand, because it depended on where a person was standing and
what the lighting was. If the run was recorded, the failure can be replayed as
many times as needed while you change the code, which turns an anecdote into a
debuggable event. Every serious robotics company logs continuously for exactly
this reason.

Jazzy uses MCAP as the default storage format. It is self describing, so a bag
recorded today can be read years later without the message definitions being
installed, and it handles large files better than the older SQLite format.

---

> ### Engineering Practice: what gets logged, and what it costs
>
> Deployed robots record continuously, and deciding what to record is a real
> engineering decision rather than a default.
>
> Recording everything is tempting and expensive. Camera topics dominate: a
> single 640x480 RGB stream at 30 Hz is roughly 26 MB per second uncompressed,
> which fills a disk in hours. Fleets therefore record a reduced set
> continuously and switch to full rate recording when something anomalous
> happens, keeping a rolling buffer of the preceding thirty seconds so the cause
> is captured along with the effect.
>
> The second decision is what makes a log useful later. A bag containing sensor
> data but not the transforms, parameters or software version is often
> unusable, because you cannot reconstruct what the robot believed at the time.
> A good practice, and one you should adopt for your projects, is to record
> `/tf`, `/tf_static`, the parameters and the Git commit hash alongside the
> sensor topics. It costs almost nothing and it is the difference between a
> reproducible investigation and a guess.
>
> This is also how the evaluation harness you meet in Lab 6 works. It records
> what happened rather than asking you to describe it, which is why its output
> can be graded and your description of a run cannot.

---

### Pre-lab quiz

Five questions on the VLE covering the role of the bridge, what `use_sim_time`
does and which nodes need it, why a LiDAR minimum range matters, what the real
time factor tells you, and what should be recorded alongside sensor data for a
log to be useful.

---

## In the session

### Stage 0: health check and launch (10 minutes)

```
course-check
```

Note your graphics tier from the output. The course VM ships with VirtualBox 3D
acceleration off, so `course-check` reports Tier B and rendering goes through
llvmpipe. Launch headless and use RViz for visualisation. Every graded task
today works this way.

```
ros2 launch arc_gazebo simulation.launch.py world:=arc_warehouse headless:=true
```

Do not turn 3D acceleration on to get a Gazebo window. With acceleration on, the
sensor renderer runs against VirtualBox's SVGA3D driver, which advertises an
OpenGL version it does not fully implement. The LiDAR then renders nothing and
reports every beam at `range_min`, at the right rate, in the right frame, with
no error in any log. `docs/vm_graphics_and_gpu_sensors.md` has the measurements.

**[SCREENSHOT PLACEHOLDER]**
The Gazebo Harmonic window with the robot spawned in the warehouse world, and the
real time factor indicator visible in the status bar.
*Instructor note: capture on the weakest lab PC rather than a development
machine, so the real time factor students see in the manual matches what they
will actually observe. Annotate the RTF reading.*

### Stage 1: demonstration and the fault (15 minutes)

Your demonstrator will show a healthy system, then break one sensor in a way that
produces plausible but wrong data rather than no data. Diagnosing it is part of
the exit task. Plausible but wrong is harder than absent, and it is also more
common.

### Exercise 3.1: verify the data before trusting it (15 minutes)

Never build on a sensor you have not checked. Check all three.

```
ros2 topic list
ros2 topic hz /scan --window 50     # expect 10 Hz
ros2 topic hz /odom --window 50     # expect 50 Hz
ros2 topic hz /imu --window 50      # expect 100 Hz
ros2 topic echo /scan --once --full-length
ros2 topic info /scan --verbose
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

Do not read the first number `ros2 topic hz` prints. It is a running average, and
the first report covers the shortest window. It also arrives while nodes are
still starting, so a busy moment is measured as a slow topic. On a machine where
a settled measurement gives `/odom` at 49.7 Hz, the first report reads about
33 Hz. Let each command run for several reports and record a later one.
`--window 50` widens the sample each average is taken over.

`--full-length` matters on `/scan`. Without it, `ros2 topic echo` prints the
first 128 entries of `ranges` and replaces the rest with `'...'`. The scan has
360 beams, so most of the data is hidden by default and a fault in the hidden
beams does not appear.

Fill this in:

| Topic | Expected rate | Observed rate | `frame_id` | Reliability |
|-------|---------------|---------------|------------|-------------|
| `/scan` | 10 Hz | | | |
| `/odom` | 50 Hz | | | |
| `/imu` | 100 Hz | | | |

The `frame_id` column is the one people skip. A scan whose `frame_id` does not
match a frame in the TF tree is unusable by the costmap, and `ros2 topic hz` will
happily report a perfect 10 Hz the whole time.

Now go past what `ros2 topic hz` can tell you.

**Code 3.1: Checking a sensor stream for stalls, jitter and clock problems**

```python
#!/usr/bin/env python3
"""Diagnose a sensor topic beyond its average rate."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from rate_stats import analyse, stamp_lag

# Lab 1, Exercise 1.2. Sensor data is published best effort, so a subscription
# using the default reliable profile will connect and receive nothing.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class SensorDoctor(Node):
    def __init__(self, topic="/scan", expected_hz=10.0, window=100):
        super().__init__("sensor_doctor")
        self.expected_hz = expected_hz
        self.window = window
        self.header_stamps = []
        self.receive_times = []
        self.frame_id = None
        self.create_subscription(LaserScan, topic, self.on_scan, SENSOR_QOS)
        self.get_logger().info(f"watching {topic}, expecting {expected_hz} Hz")

    def on_scan(self, msg):
        # Two different clocks on purpose. The header stamp is when the sensor
        # says the measurement was taken. The receive time is when this node
        # saw it. Comparing them catches the use_sim_time mistake.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.header_stamps.append(stamp)
        self.receive_times.append(self.get_clock().now().nanoseconds / 1e9)
        self.frame_id = msg.header.frame_id

        if len(self.header_stamps) >= self.window:
            self.report()
            self.header_stamps.clear()
            self.receive_times.clear()

    def report(self):
        result = analyse(self.header_stamps, self.expected_hz)
        lag = stamp_lag(self.header_stamps, self.receive_times)
        self.get_logger().info(str(result))
        self.get_logger().info(f"frame_id '{self.frame_id}', mean stamp lag {lag:.4f} s")
        if abs(lag) > 1.0:
            self.get_logger().error(
                "stamp lag over one second. This node and the publisher are "
                "almost certainly on different clocks. Check use_sim_time.")


def main():
    rclpy.init()
    node = SensorDoctor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

The stamp lag check is the useful part. Run this node correctly and the lag is a
few milliseconds. Run it with `use_sim_time` false against a simulator and the
lag is the Unix epoch, roughly 1.7 billion seconds, which is unmistakable.

Try it both ways:

```
ros2 run arc_lab3 sensor_doctor --ros-args -p use_sim_time:=true
ros2 run arc_lab3 sensor_doctor --ros-args -p use_sim_time:=false
```

### Exercise 3.2: three broken sensors (25 minutes)

Three world files each contain one fault. Diagnose each using the workflow rather
than by reading the file, then confirm by reading it.

```
ros2 launch arc_gazebo simulation.launch.py world:=broken_a headless:=true
ros2 launch arc_gazebo simulation.launch.py world:=broken_b headless:=true
ros2 launch arc_gazebo simulation.launch.py world:=broken_c headless:=true
```

| World | Symptom you observe | Command that revealed it | Root cause | Fix |
|-------|---------------------|--------------------------|------------|-----|
| broken_a | | | | |
| broken_b | | | | |
| broken_c | | | | |

The faults are drawn from this set, so a symptom that does not match any of them
means you have found something unintended and should tell your demonstrator:
sensor update rate reduced, `gz_frame_id` not matching any link, the bridge entry
for a topic removed, LiDAR maximum range reduced below the room dimensions, and
noise standard deviation raised by two orders of magnitude.

One of these is deliberately subtle. The data arrives, at the right rate, in the
right frame, and is wrong in a way you only see if you look at the values. That
is the most realistic fault of the three, and it is the reason the workflow ends
with `ros2 topic echo --full-length` rather than starting with it. Keep
`--full-length` on here. The fault may be in the 232 beams that echo hides by
default.

**[SCREENSHOT PLACEHOLDER]**
RViz side by side comparison of the LaserScan display in a healthy world and in
the world with excessive noise, at identical robot poses.
*Instructor note: set the LaserScan display point size to 0.03 and use the same
fixed frame and camera position for both captures, so the difference is
attributable to the data rather than the view.*

### Exercise 3.3: record the bag for next week (20 minutes)

This recording is the input to the occupancy mapping exercise in Lab 4. Take it
seriously.

Drive the robot on a route that observes the whole room, including the corners,
moving slowly enough that consecutive scans overlap substantially.

```
ros2 bag record -o lab03_mapping_run \
  /scan /odom /tf /tf_static /clock /joint_states \
  --storage mcap
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p use_sim_time:=true
```

Four of those topics deserve a comment. `/tf` and `/tf_static` are included
because without them the scans cannot be placed in a common frame, and a mapping
bag without transforms is decorative. `/clock` is included so the replay can drive
simulation time for the nodes consuming it. `/joint_states` costs almost nothing
and lets you reconstruct the robot's configuration.

Verify the recording before you rely on it:

```
ros2 bag info lab03_mapping_run
ros2 bag play lab03_mapping_run --clock
# in another terminal, with the simulator stopped
ros2 topic hz /scan
rviz2 --ros-args -p use_sim_time:=true
```

Check three things in `ros2 bag info`: that all six topics are present, that the
`/scan` message count is roughly ten times the duration in seconds, and that the
duration is long enough to have covered the room. A bag with 40 scans over 90
seconds means the LiDAR was not healthy during the recording, and you would
rather discover that now than next week.

**[GIF PLACEHOLDER]**
The recorded bag replaying in RViz, with the LaserScan and TF displays active and
the robot moving through the warehouse.
*Instructor note: record roughly 20 seconds with the fixed frame set to `odom`,
so the accumulating drift between scans is visible. That drift is the motivation
for Lab 5 and is worth being able to point at.*

### Exercise 3.4: real time factor (5 minutes)

```
ros2 topic echo /clock --once
# note the value, wait ten seconds by a real clock, repeat
```

Compute the ratio of simulated seconds elapsed to real seconds elapsed. Record
it, and record your machine's graphics tier alongside.

This number matters for the rest of the course. It tells you how long a
navigation benchmark will actually take in Lab 6, and it is the reason
reinforcement learning in Lab 9 trains in a fast surrogate rather than in Gazebo.
When you see that argument made in week nine, this measurement is the evidence
for it.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your completed sensor verification table from Exercise 3.1.
2. Your completed fault table from Exercise 3.2, with root causes.
3. `ros2 bag info` output for your mapping run, and the bag itself pushed using
   Git LFS or uploaded to the VLE if it exceeds the repository limit.
4. Your measured real time factor and graphics tier.
5. The `sensor_doctor` output showing the stamp lag with `use_sim_time` false.

Run the self check first:

```
cd ~/arc_ws/src/arc-course
pytest starters/lab03/tests -v
```

---

## Troubleshooting

**Gazebo will not start, or starts and shows a black window.**

```
course-check
echo $LIBGL_ALWAYS_SOFTWARE
```

Tier B is the normal state on the course VM, and the headless launch in Stage 0
is what to use. If `course-check` reports Tier C, the machine cannot run Gazebo
at all. That is an infrastructure condition, not a mistake on your part, and it
does not cost you marks. Record the incident ID.

**A topic exists in Gazebo but not in ROS.**

```
gz topic -l
ros2 topic list
```

Compare the two. A topic present in the first list and absent from the second is
missing from the bridge configuration.

**"Lookup would require extrapolation into the future."**

A node is on the wrong clock. Check `use_sim_time` on every node in the system,
including RViz and including anything you started in a terminal to look at
something.

```
ros2 param get /rviz use_sim_time
ros2 param get /robot_state_publisher use_sim_time
```

**The robot sinks through the floor or vibrates.**

A link has missing or implausible inertial properties, which is the Lab 2 warning
arriving. Check that every link with a collision element has non-zero mass and a
sensible inertia tensor.

**`ros2 topic hz` reports the right rate but the costmap is empty later.**

Check `frame_id`. A correctly timed scan in a frame that does not exist in the TF
tree is unusable, and only `ros2 topic echo` and `tf2_echo` will show you.

**Bag playback produces no TF.**

You did not record `/tf_static`, which is published once with transient local
durability at startup. Without it the fixed sensor transforms are absent for the
entire replay. Record it again.

---

## Connection to Lab 4

You have a robot producing real, noisy, timestamped sensor data, and you have a
recording of it moving through a room.

That recording is a set of range measurements taken from a sequence of known
poses, which is exactly the input an occupancy grid mapping algorithm needs. Next
week you write that algorithm. You will implement the inverse sensor model and
the log odds update by hand, in NumPy, applied to your own bag, and watch a map
of the room you drove through this week assemble itself cell by cell.

Because it runs against a recording rather than a live simulation, it is
deterministic. The same bag and the same code give the same map every time, which
makes it a fair thing to grade and a sensible thing to debug. It also means that
if Gazebo will not run on your machine that day, the lab is unaffected.

---

## References

**Textbooks**

Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
Chapter 6 covers sensor models, including the beam model for range finders, and
explains what the noise parameters in your LiDAR configuration are approximating.

**Official documentation**

Gazebo Harmonic documentation: https://gazebosim.org/docs/harmonic
Sensor configuration and the SDF specification. Note carefully that most material
you will find by searching targets Gazebo Classic, whose syntax differs.

`ros_gz` documentation and repository: https://github.com/gazebosim/ros_gz
The bridge, its type mappings and its configuration file format.

rosbag2: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data
MCAP specification: https://mcap.dev

**Repositories**

`gazebosim/ros_gz` on GitHub, particularly `ros_gz_sim_demos`, which contains
working launch files for every common sensor type on Harmonic. This is the most
reliable source of correct Harmonic syntax.

**Video**

Gazebo Harmonic community tutorials from the Open Robotics channel. Prefer
material dated 2024 or later; anything older is likely to describe Gazebo Classic
and will not work in this environment.

---

## Further reading

`docs/references.md` has a fuller list under **Lab 3. Sensors and data
validation**, with papers, industry write-ups and the documentation worth
keeping open. Every entry says what you get from it and which part of the lab it
connects to.

If you read one thing, read *ROSMonitoring 2.0* (Ghaffari Saadat and others,
FMAS 2024). It attaches automatic monitors to topics that check message content
and ordering, which is the systematic version of the "publishing but wrong"
diagnosis you did by hand today.

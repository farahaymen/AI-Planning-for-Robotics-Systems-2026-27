---
title: "Lab 2: Mobile Robot Kinematics, TF2, URDF and ros2_control"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 2: Mobile Robot Kinematics, TF2, URDF and ros2_control

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1 (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic)
**Packages** `arc_description`, `arc_gazebo`, `robot_state_publisher`, `tf2_ros`, `tf2_tools`, `ros2_control`, `diff_drive_controller`, `xacro`

**Prerequisites**

Lab 1. You should be able to inspect a running graph and diagnose a QoS mismatch
without being told to. Basic trigonometry and matrix multiplication.

---

## Before the session

The kinematics below is the mathematical content of this lab, and it is arithmetic
you can do at home. Working through it now is what makes the two hours in the
laboratory available for the robot description and the transform tree, which are
harder to learn from a page.

### Why this lab matters

In Lab 1 your messages carried a number that meant nothing. From today they carry
physical quantities, and physical quantities have to be consistent with a real
object or the whole stack quietly produces nonsense.

Two numbers govern almost everything. The wheel radius converts wheel rotation
into distance travelled. The wheel separation converts a difference in wheel
speeds into a rotation rate. Get either slightly wrong and the robot still drives,
still reports odometry, still looks completely healthy, and ends up somewhere
other than where it believes it is. In Lab 5 that error will make your map
smear. In Lab 6 it will make Nav2 overshoot goals. No error message will appear
at any point.

The same is true of coordinate frames. A LiDAR mounted ten centimetres forward of
the chassis centre reports obstacles ten centimetres closer than they are, unless
something tells the rest of the system where the sensor sits. That something is
TF2, and it is the piece of ROS that most people find confusing for longest.

### Differential drive kinematics

The robot has two independently driven wheels on a common axis and a passive
caster. Let $r$ be the wheel radius, $L$ the separation between the wheel
contact points, and $\omega_L$, $\omega_R$ the wheel angular velocities.

**Forward kinematics** answers "the wheels are turning at these speeds, how is
the body moving?"

$$
v = \frac{r(\omega_R + \omega_L)}{2}
\qquad
\omega = \frac{r(\omega_R - \omega_L)}{L}
$$

The structure is worth reading rather than memorising. The average of the wheel
speeds gives forward motion, and the difference gives rotation. Equal speeds
produce a straight line. Equal and opposite speeds produce rotation on the spot.

**Inverse kinematics** answers "I want the body to move like this, what should
the wheels do?" This is the direction the controller uses, because Nav2 will hand
you a `geometry_msgs/Twist` and the hardware needs wheel commands.

$$
\omega_L = \frac{v - \omega L / 2}{r}
\qquad
\omega_R = \frac{v + \omega L / 2}{r}
$$

Substitute one into the other and you get back where you started, which is a
useful check and is exactly what one of the supplied unit tests does.

![The differential drive model. Course constants shown are the frozen values in `RobotSpec`.](docs/figures/lab02_diff_drive.png){width=82%}


Try the arithmetic once with the course constants, $r = 0.050$ m and
$L = 0.350$ m. At the robot's limits of $v = 0.50$ m/s and
$\omega = 1.80$ rad/s together, the outer wheel needs

$$
\omega_R = \frac{0.50 + 1.80 \times 0.175}{0.050} = 16.3 \ \text{rad/s}
$$

That number has a consequence you will see in the hardware interface, and it is
the sort of consistency check that separates a robot description that works from
one that appears to.

**Odometry** integrates the twist over time to estimate pose. The obvious version
is

$$
x \mathrel{+}= v \cos\theta \, \Delta t, \quad
y \mathrel{+}= v \sin\theta \, \Delta t, \quad
\theta \mathrel{+}= \omega \Delta t
$$

and it is what most tutorials show. It approximates each control period as a
straight line, which is wrong whenever the robot is turning. When
$\omega \neq 0$ the robot actually traces an arc of radius $R = v / \omega$, and
integrating the arc exactly costs three extra lines:

$$
x \mathrel{+}= R\left(\sin(\theta + \omega \Delta t) - \sin\theta\right), \quad
y \mathrel{-}= R\left(\cos(\theta + \omega \Delta t) - \cos\theta\right), \quad
\theta \mathrel{+}= \omega \Delta t
$$

Read `starters/lab02/diffdrive.py` before the session. It implements all of the
above in about sixty lines, and the tests alongside it demonstrate that a
thousand small steps around a full circle return to the origin, which the
straight line version does not.

### Coordinate frames

ROS uses a small set of conventional frames, and the convention exists so that
software written by different people composes without negotiation.

`base_link` is rigidly attached to the chassis. `base_footprint` is its
projection onto the ground plane, which is what navigation reasons about because
the robot drives on a floor. `odom` is a frame in which the robot's pose is
continuous and smooth but drifts without bound, because it comes from integrating
wheel motion. `map` is a frame in which the pose does not drift but does jump,
because it comes from matching sensor data against a map.

The relationship between those last two is the important idea. Odometry is
smooth and wrong. Localisation is correct and discontinuous. Rather than choosing,
ROS keeps both and expresses the correction as the `map` to `odom` transform,
published by AMCL from Lab 5. Anything needing smooth motion, such as a
controller, uses `odom`. Anything needing global correctness, such as a goal pose,
uses `map`.

The full chain on our robot is

![The transform tree. Each edge has exactly one publisher.](docs/figures/diagram_tf_tree.png){width=72%}

Each transform has exactly one publisher. Two publishers for the same edge is a
real failure mode, it usually means something was launched twice, and the symptom
is a robot that jitters between two positions in RViz.

### URDF, Xacro and robot_state_publisher

The robot description is an XML document listing links, which are rigid bodies
with visual, collision and inertial properties, and joints, which are the
relationships between them. Fixed joints never move. Continuous joints rotate
without limit, which is what wheels do.

URDF is verbose, so we write Xacro, which adds properties, macros and arithmetic
and expands to URDF. The wheel macro in `arc_bot.urdf.xacro` is instantiated
twice with a sign flip, which is both shorter and safer than writing the same
twenty lines twice with one number changed.

`robot_state_publisher` reads the description, subscribes to `/joint_states`, and
publishes the transforms for every joint. Fixed joints are published once as
static transforms. Moving joints are republished continuously. This is why
adding a sensor to the URDF is all you need to do to make its frame available to
the entire system.

### ros2_control

Between "the controller wants 0.3 m/s" and "the motor spins" there is a layer
that nobody enjoys writing twice. `ros2_control` is that layer, and it splits
into two halves that meet at a defined interface.

A **hardware interface** exposes named command interfaces, such as the velocity
of `left_wheel_joint`, and named state interfaces, such as its position. In
simulation this is provided by `gz_ros2_control`. On a physical robot it is a
small C++ class that talks to a motor driver over CAN, EtherCAT or a serial link.

A **controller** consumes those interfaces without knowing what is behind them.
`diff_drive_controller` subscribes to `/cmd_vel`, applies the inverse kinematics
you worked through above, writes wheel velocity commands, reads back wheel
positions, integrates odometry and publishes the `odom` to `base_footprint`
transform.

The consequence is the part worth remembering. Moving this course's software from
Gazebo to a physical robot means replacing one plugin declaration. The
controller configuration, Nav2, and everything you write for the rest of the
semester stays identical. That substitutability is why the layer exists and why
it is worth the ceremony.

---

> ### Industry Perspective: the hardware boundary
>
> The hardware interface is where robotics software meets electrical engineering,
> and it is the part of a robot that is almost never open source, because it is
> specific to the motors and drivers a company chose.
>
> Underneath it sits a fieldbus. **CAN bus** is common on mobile robots and
> vehicles, is robust to electrical noise, and is slow enough that you think about
> message budgets. **EtherCAT** dominates industrial applications needing
> synchronised control of many joints at high rates, and offers deterministic
> cycle times in the sub-millisecond range. Plain **UART or RS-485** is still
> everywhere on small robots because it is simple and adequate.
>
> Two engineering realities follow. First, the control loop rate is bounded by
> the bus and the driver firmware, not by your Python. Second, and less obvious,
> the wheel radius in your configuration is not the manufacturer's specification.
> It is the effective rolling radius, which changes with tyre pressure, load and
> wear. Commissioning a real AMR includes driving a measured distance and
> adjusting the number until reported odometry matches reality. Exercise 2.4 is a
> simulated version of that procedure, and it is a task you would genuinely be
> asked to do in an internship.

---

### Pre-lab quiz

Five questions on the VLE covering the forward and inverse kinematics, the
purpose of `base_footprint` versus `base_link`, why `map` to `odom` exists, what
`robot_state_publisher` needs in order to publish a transform, and what the
hardware interface abstracts.

---

## In the session

### Stage 0: health check and bring-up (10 minutes)

```
course-check
ros2 launch arc_description display.launch.py
```

This starts `robot_state_publisher`, a joint state publisher with sliders, and
RViz. No simulator and no physics yet, because today is about the description
rather than the dynamics.

**[SCREENSHOT PLACEHOLDER]**
RViz showing the robot model with the TF display enabled, all frames visible, and
the joint state slider panel alongside.
*Instructor note: set the RViz fixed frame to `base_footprint` and enable both
RobotModel and TF displays. Set the TF marker scale large enough that the axes at
`laser_link` are clearly offset forward from `base_link`, since that offset is
the point of the whole lab.*

### Stage 1: demonstration and the fault (15 minutes)

Your demonstrator will drive the robot, show the transform tree, then break one
relationship in the description and hand it back. Diagnosing it is part of the
exit task.

### Exercise 2.1: read the tree (15 minutes)

```
ros2 run tf2_tools view_frames
evince frames.pdf
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 topic echo /robot_description --once | head -40
```

`tf2_echo` prints the translation and rotation between any two frames. Compare
the translation it reports for `base_link` to `laser_link` against the origin in
the `laser_joint` block of `arc_bot.urdf.xacro`. They must agree, because one
produced the other.

Record in your exit task: how many frames exist, which node publishes each edge,
and which edges are static.

### Exercise 2.2: add a sensor frame (20 minutes)

Add a mounting point for a camera you will not use until later. The point is the
procedure, not the camera.

**Code 2.1: A new link and fixed joint in the robot description**

```xml
<!-- Add to arc_bot.urdf.xacro, before the include lines at the bottom -->

<link name="camera_link">
  <visual>
    <geometry><box size="0.03 0.09 0.025"/></geometry>
    <material name="arc_dark"><color rgba="0.15 0.15 0.18 1"/></material>
  </visual>
  <collision>
    <geometry><box size="0.03 0.09 0.025"/></geometry>
  </collision>
  <inertial>
    <mass value="0.08"/>
    <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="${base_length/2 - 0.01} 0 ${base_height/2 + 0.03}" rpy="0 0.15 0"/>
</joint>

<!-- Cameras in ROS use z forward and x right, while the robot uses x forward.
     Rather than arguing with either convention, publish a second frame that is
     the same physical point with the optical axes. Every ROS camera driver does
     this, and code that ignores it produces images that appear rotated. -->
<link name="camera_optical_link"/>
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_link"/>
  <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
</joint>
```

Two details matter more than they look.

Every link with a collision element needs an inertial element with non-zero mass.
A link without one is silently ignored by some physics engines and causes
solver instability in others, and the failure appears in Lab 3 as a robot that
sinks through the floor or vibrates.

The optical frame is not pedantry. The camera convention and the robot convention
genuinely differ, and publishing both frames costs four lines and prevents an
entire class of confusion later.

Rebuild and verify. Notice that you changed only the description:

```
colcon build --packages-select arc_description --symlink-install
source install/setup.bash
ros2 launch arc_description display.launch.py
ros2 run tf2_ros tf2_echo base_link camera_optical_link
```

The transform now exists across the whole system with no code written anywhere.

### Exercise 2.3: fix a broken transform (15 minutes)

Open `arc_description/urdf/broken_tf.urdf.xacro`, which contains one deliberate
error. Do not read it looking for the mistake. Run it and diagnose it.

```
ros2 launch arc_description display.launch.py model:=broken_tf.urdf.xacro
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link laser_link
```

Symptoms map to causes in a small number of ways, and knowing the mapping is the
skill:

| Symptom | Likely cause |
|---------|--------------|
| `tf2_echo` reports the frame does not exist | The link is not in the description, or a typo in the name |
| Two disconnected trees in `frames.pdf` | A joint names a parent that is not a link |
| Sensor data appears in the wrong place | Wrong `origin` on the joint |
| Frames flicker between two positions | Two publishers for the same transform |
| "Lookup would require extrapolation into the future" | Timing, not geometry, and you will meet it properly in Lab 3 |

Record the symptom, the command that revealed it, and the fix.

### Exercise 2.4: the wheel radius, measured (20 minutes)

This is the exercise that connects the arithmetic to something observable.

Predict first. If the controller is told the wheels are 0.055 m in radius when
they are physically 0.050 m, and the robot actually travels 2.0 m, what distance
will the odometry report?

```python
from diffdrive import odometry_scale_error
print(odometry_scale_error(assumed_radius=0.055, true_radius=0.050))
```

Now measure it. Start the simulation, inspect the controllers, drive a known
distance, and read the odometry.

```
ros2 launch arc_gazebo simulation.launch.py headless:=true
```

**Code 2.2: Driving a fixed distance and reading the reported result**

```python
#!/usr/bin/env python3
"""Drive straight for a fixed duration and report what odometry claims."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomRuler(Node):
    def __init__(self, speed=0.2, duration=10.0):
        super().__init__("odom_ruler")
        self.speed, self.duration = speed, duration
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.start = None
        self.travelled = 0.0
        self.last = None
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last is not None:
            self.travelled += math.hypot(p.x - self.last[0], p.y - self.last[1])
        self.last = (p.x, p.y)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start is None:
            self.start = now
        elapsed = now - self.start

        cmd = Twist()
        if elapsed < self.duration:
            cmd.linear.x = self.speed
        else:
            self.publisher.publish(cmd)          # explicit zero, then report
            expected = self.speed * self.duration
            self.get_logger().info(
                f"commanded {expected:.3f} m, odometry reports "
                f"{self.travelled:.3f} m, ratio {self.travelled / expected:.4f}")
            raise SystemExit
        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = OdomRuler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Run it once with the correct radius, then edit `wheel_radius` in
`arc_bot_controllers.yaml` to 0.055, rebuild, and run it again.

| `wheel_radius` in config | Commanded distance (m) | Reported distance (m) | Ratio | Predicted ratio |
|---------------------------|------------------------|------------------------|-------|-----------------|
| 0.050 | 2.0 | | | 1.000 |
| 0.055 | 2.0 | | | 1.100 |
| 0.045 | 2.0 | | | 0.900 |

Restore the correct value before you leave. Then note what this means for the
rest of the course: every downstream consumer inherits this error. The costmap
places obstacles wrongly, AMCL fights the odometry it is supposed to be
correcting, and Nav2 overshoots. None of them will tell you why.

Finally, inspect the control layer itself:

```
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

The first shows which controllers are loaded and active. The second shows the
named command and state interfaces the hardware exposes. Compare the interface
names against the `ros2_control` block in the Xacro; they come from there
directly.

### Exercise 2.5: build something in C++ (15 minutes)

You will not write C++ in this course, but you will read it, and reading it is
much easier if you have compiled it at least once.

`arc_lab2_cpp` contains a small `rclcpp` node that subscribes to `/joint_states`
and warns when a wheel exceeds a configured speed. Make two changes:

1. Change the declared parameter default `max_wheel_speed` from 20.0 to 16.3,
   the value the kinematics demanded above.
2. Change the subscribed topic from `/joint_states` to `/dynamic_joint_states`
   and then change it back, so that you see the build and run cycle twice.

```
colcon build --packages-select arc_lab2_cpp
source install/setup.bash
ros2 run arc_lab2_cpp wheel_watchdog
ros2 param get /wheel_watchdog max_wheel_speed
```

Read the file while it builds. Notice that the structure is the same as Code 1.1:
a class deriving from `Node`, a subscription with a callback, a declared
parameter. The concepts transfer completely; only the syntax and the build step
differ. That is the point of the exercise, and it is why a Nav2 controller plugin
in C++ will be readable to you in Lab 6.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your modified URDF with the camera and optical frames, and the `tf2_echo`
   output for `base_link` to `camera_optical_link`.
2. The fault you found in Exercise 2.3, the symptom, and the command that
   revealed it.
3. Your completed wheel radius table with predicted and measured ratios.
4. The output of `ros2 control list_hardware_interfaces`.

Run the self check first:

```
cd ~/arc_ws/src/arc-course
pytest starters/lab02/tests -v
```

---

## Troubleshooting

**The robot appears in RViz as a jumble of shapes at the origin.**

No transforms are being published. Check that `robot_state_publisher` is running
and that `/joint_states` has a publisher. Without joint states, non-fixed joints
have no transform.

**Xacro fails to expand.**

```
cd ~/arc_ws/src/arc-course
xacro arc_description/urdf/arc_bot.urdf.xacro > /tmp/check.urdf
check_urdf /tmp/check.urdf
```

Expanding by hand gives a line number. `check_urdf` then verifies the tree is
connected and prints the link hierarchy, which is the fastest way to spot an
orphaned link.

**"Lookup would require extrapolation into the past."**

The transform existed but not at the requested time. Almost always a timing
problem rather than a geometry one, and Lab 3 covers it properly. For today,
confirm nothing is publishing stale timestamps.

**The robot drives but odometry stays at zero.**

```
ros2 control list_controllers
ros2 topic hz /odom
```

If `diff_drive_controller` is `inactive`, it is consuming nothing and publishing
nothing. This is the Lab 1 lifecycle lesson arriving in a new costume.

**The robot turns more slowly than commanded.**

The wheel speeds required exceed the command interface limits and are being
saturated. Compute the demand with `wheel_speed_limits()` and compare it against
the `min` and `max` in the `ros2_control` block.

---

## Connection to Lab 3

You now have a robot description that is geometrically correct, a control layer
that converts velocity commands into wheel motion, and a transform tree that
tells every node where every part of the robot is.

What you do not yet have is any real data. The joint state sliders you used today
are a fiction, the LiDAR link publishes nothing, and there is no physics.

Next week the robot goes into Gazebo. The wheels will slip, the LiDAR will
produce noisy readings at 10 Hz, timestamps will start to matter in a way they
have not so far, and you will meet the difference between wall clock time and
simulation time. You will also record everything to a bag file, which becomes the
input to the mapping exercise in Lab 4, so the quality of the recording you make
next week determines how straightforward that lab is.

---

## References

**Textbooks**

Lynch, K. M. and Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and
Control*. Cambridge University Press. Chapter 13 covers wheeled mobile robots and
derives the differential drive model properly, including the nonholonomic
constraint that this lab treats informally.

Siegwart, R., Nourbakhsh, I. R. and Scaramuzza, D. (2011). *Introduction to
Autonomous Mobile Robots*, 2nd edition. MIT Press. Chapter 3 on locomotion and
kinematics, and chapter 5 on odometry error, which is the theoretical version of
Exercise 2.4.

**Official documentation**

`ros2_control` documentation: https://control.ros.org
The architecture overview is worth reading in full. The distinction between
hardware components and controllers is the whole idea and is explained there
better than in most tutorials.

`tf2` tutorials: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2
Work through the broadcaster and listener tutorials if today felt fast.

REP 103, Standard Units of Measure and Coordinate Conventions, and REP 105,
Coordinate Frames for Mobile Platforms: https://ros.org/reps/rep-0105.html
Short, authoritative, and the reason `map`, `odom` and `base_link` mean the same
thing in everyone's code.

URDF and Xacro documentation: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF

**Repositories**

`ros-controls/ros2_control_demos` on GitHub. Includes a minimal hardware
interface implementation in C++, which is the clearest example of what sits below
the boundary discussed in the industry section.

**Video**

`ros2_control` architecture talks from ROSCon, on the Open Robotics YouTube
channel. The diagrams alone are worth the time, particularly the one showing the
resource manager mediating between controllers and hardware components.

---

## Further reading

`docs/references.md` has a fuller list under **Lab 2. Robot description and
differential drive kinematics**, with papers, industry write-ups and the
documentation worth keeping open. Every entry says what you get from it and
which part of the lab it connects to.

If you read one thing, read *Understanding URDF: A Dataset and Analysis* (Tola
and Corke, 2023). It analyses 322 real URDF files and reports which conventions
and which mistakes actually occur, which is useful before you write your own.

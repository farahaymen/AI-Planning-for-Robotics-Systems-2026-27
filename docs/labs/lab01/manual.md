---
title: "Lab 1: ROS 2 as a Robotics Software System"
subtitle: "Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning"
author: "British University in Egypt"
date: "Duration 2 hours | ARC VM 2026.1"
---

# Lab 1: ROS 2 as a Robotics Software System

**Course** Autonomous Robotics with ROS 2: Mapping, Navigation and Reinforcement Learning
**Duration** 2 hours
**Environment** ARC VM 2026.1 (Ubuntu 24.04, ROS 2 Jazzy)
**Packages** `rclpy`, `std_msgs`, `sensor_msgs`, `example_interfaces`, `lifecycle_msgs`, `arc_lab1`

**Prerequisites**

Comfort with Python classes and callbacks, which you already have from your
Machine Learning and Deep Learning modules. No prior ROS experience is assumed.
No C++ is required today.

---

## Before the session

### Why this lab matters

A robot is not one program. It is a LiDAR driver sampling at 10 Hz, a wheel
controller running at 50 Hz, a planner that thinks for 200 milliseconds at a
time, a safety monitor that must never be delayed by any of them, and a dozen
other processes that were written by different people at different times in
different languages. Some of them run on the robot. Some run on a laptop
connected over a wireless link that drops.

If you wrote all of that as one Python script, the first thing that blocked would
take everything else down with it. This is not a hypothetical. It is the reason
robotics converged on a middleware layer rather than a framework, and it is why
your first lab is about software structure rather than about robots moving.

The habits you build today are the ones that determine whether you can debug your
project in week eleven. Nearly every problem you will hit for the rest of this
course reduces to one of four questions: is the node running, does the topic
exist, is data flowing, and are the two ends actually compatible. Today you learn
to ask them in that order.

### The graph

ROS 2 arranges software as a graph. The vertices are **nodes**, which are
processes doing one job each. The edges are the connections between them, and
there are three kinds, distinguished by how long the work takes and whether you
need to know how it is going.

A **topic** is a stream. A publisher sends messages and any number of subscribers
receive them. Nobody waits for anybody. This is right for sensor data, velocity
commands and anything else that is continuous and current. If a LiDAR scan is
lost it does not matter much, because another one arrives in a tenth of a second.

A **service** is a question with an answer. The caller blocks until the reply
arrives. This suits fast, occasional requests such as asking a node to save a map
or reset a counter. It is the wrong choice for anything slow, because the caller
has no way to see progress and no way to cancel.

An **action** is a long job you want to monitor. It gives you goal acceptance,
periodic feedback while the work proceeds, cancellation, and a final result. This
is how you will talk to Nav2 from Lab 6 onwards, and the reason is exactly the
weakness of services: navigating to a point takes thirty seconds, you want to
know how it is going, and you want to be able to stop it.

**Parameters** are named configuration values a node declares at startup and can
be changed at runtime. Nav2 has hundreds of them. Learning to read and set them
from the command line saves you from editing YAML and restarting all semester.

### Quality of Service, and why your subscriber will receive nothing

This is the single most important thing in the lab.

In ROS 1, a topic connection either existed or it did not. In ROS 2 the transport
is DDS, which lets each endpoint state what it offers or requires, and a
connection forms only if the offer satisfies the requirement. The rule is one
sentence: **the publisher must offer at least as strong a guarantee as the
subscriber requests.**

Two policies matter today.

**Reliability** is either `BEST_EFFORT`, meaning messages may be dropped, or
`RELIABLE`, meaning they are retransmitted until they arrive. Reliable is the
stronger promise. A best effort publisher therefore cannot satisfy a reliable
subscriber. Sensor drivers publish best effort, because retransmitting a LiDAR
scan from four seconds ago is worse than useless.

**Durability** is either `VOLATILE`, meaning subscribers only get messages sent
after they connect, or `TRANSIENT_LOCAL`, meaning the publisher keeps the last
message and delivers it to late joiners. Maps and robot descriptions are
published transient local, because they are sent once at startup and everything
that needs them starts at a different time.

Here is the part that makes this worth a whole section. **A mismatch produces no
error.** The subscription is created successfully. The node starts normally. The
callback simply never fires. There is no message in the log, no exception, and
nothing obviously wrong. Students lose hours to this, and so do professionals.

The defence is to predict rather than discover. `starters/lab01/qos_rules.py`
implements the compatibility rule as a function, so you can work out what should
happen before you run anything, then confirm it against the system.

![QoS compatibility, generated by calling `qos_rules.check`. The red cells produce no error message of any kind.](docs/figures/lab01_qos_matrix.png){width=78%}


### Managed nodes

Some nodes should not start working the moment they are launched. A localisation
node that begins publishing transforms before it has a map, or a controller that
accepts commands before its hardware is confirmed present, is worse than a node
that has not started.

ROS 2 provides managed nodes with an explicit lifecycle: `unconfigured`,
`inactive`, `active` and `finalized`, with defined transitions between them. A
node in `inactive` exists, holds its resources, and does nothing.

You are meeting this in week one rather than week eight for a practical reason.
From Lab 6 onwards every Nav2 server is a managed node, and the most common cause
of a navigation stack that comes up cleanly and then does nothing at all is that
one of them never reached `active`. If your first instinct when navigation fails
is to check the goal, you will waste a lot of time. If your first instinct is
`ros2 lifecycle get`, you will not.

---

> ### Engineering Practice: the middleware underneath
>
> ROS 2 does not implement its own transport. It defines an abstract middleware
> interface, the RMW, and delegates to an implementation of the DDS standard.
> Jazzy ships with Fast DDS as the default, and Cyclone DDS is a widely used
> alternative selected by setting one environment variable.
>
> This matters in practice more than it sounds. The two implementations differ in
> discovery behaviour, in how they handle large messages, and in how they behave
> on a lossy wireless link. Teams deploying real fleets do change this variable,
> and they change it because of measured behaviour rather than preference.
>
> Zenoh has become a serious third option, particularly for robots communicating
> over unreliable networks or across the internet, where DDS discovery struggles.
> You do not need it for this course, but you should know it exists, because it
> comes up in industry conversations about multi-robot and cloud connected
> systems.
>
> One consequence of DDS discovery is worth internalising now. By default, nodes
> find each other by multicasting across the local network. In a laboratory with
> thirty machines that means thirty students potentially discovering and
> commanding each other's robots. The course VM sets
> `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and gives each seat its own
> `ROS_DOMAIN_ID` to prevent this. You will see the effect deliberately in
> Exercise 1.4.

---

### Pre-lab quiz

Five questions on the VLE, closing one hour before your session, covering the
three communication patterns, when to use each, the QoS compatibility rule, what
an `inactive` managed node does, and what `ROS_DOMAIN_ID` controls.

---

## In the session

### Stage 0: health check (10 minutes)

```
course-check
```

Note any incident ID and tell your demonstrator before starting. Then set up your
workspace and fork the course repository. Everything you do today has to be
pushed before you leave, because the virtual machine does not survive the end of
the session.

```
cd ~/arc_ws/src/arc-course
git remote -v
git checkout -b lab01-$(whoami)
```

### Stage 1: demonstration (15 minutes)

Your demonstrator will show a small system of connected nodes, then break the
connection between two of them in a way that produces no error message at all.
Watch what the graph looks like when it is healthy so that you have something to
compare against.

### Exercise 1.1: two nodes that talk (20 minutes)

You are going to write a publisher that imitates a sensor and a subscriber that
consumes it. The message content is deliberately trivial. The structure is the
point.

**Code 1.1: A publisher standing in for a sensor**

```python
#!/usr/bin/env python3
"""Publishes a fake range reading at 10 Hz, the rate of the real LiDAR.

Exercise 1.2 asks you to change the publisher QoS to best effort and observe
that the default subscriber then receives nothing, with no error anywhere.

Exercise 1.3 asks you to change the rate at runtime with `ros2 param set`.
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32


class RangeSource(Node):
    def __init__(self):
        super().__init__("range_source")
        # Declared, not hard coded, so `ros2 param set` works at runtime.
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("amplitude", 2.0)
        self.rate = float(self.get_parameter("rate_hz").value)

        self.publisher = self.create_publisher(Float32, "range", 10)
        # A timer, not a while loop. rclpy.spin hands the thread to an executor;
        # a while loop here starves every other callback and the node looks
        # alive while silently ignoring everything.
        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.k = 0

        # Declaring a parameter makes it settable, not live. A timer keeps the
        # period it was created with, so changing rate_hz does nothing until the
        # timer is rebuilt. This callback does that.
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info(f"publishing /range at {self.rate} Hz")

    def on_parameters(self, params):
        for p in params:
            if p.name != "rate_hz":
                continue
            # Reject before applying. Returning successful=False leaves the old
            # value in place, which is better than dividing by zero here.
            if p.value <= 0.0:
                return SetParametersResult(
                    successful=False, reason="rate_hz must be greater than zero")
            self.rate = float(p.value)
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / self.rate, self.tick)
            self.get_logger().info(f"rate changed to {self.rate} Hz")
        return SetParametersResult(successful=True)

    def tick(self):
        amplitude = self.get_parameter("amplitude").value
        msg = Float32()
        msg.data = float(amplitude * (1.5 + math.sin(self.k * 0.1)))
        self.publisher.publish(msg)
        self.k += 1


def main():
    rclpy.init()
    node = RangeSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Three things in that file will recur in every ROS node you write this semester.

The timer instead of a loop is not stylistic. `rclpy.spin` hands the thread to an
executor which dispatches callbacks. A `while True` inside the node would starve
every other callback, and the node would appear alive while silently ignoring
everything. This is a common first mistake and it produces very confusing
symptoms.

Declaring parameters rather than hard coding constants is what allows
`ros2 param set /range_source rate_hz 2.0` to reach the node while it runs.
Declaring alone does not change any behaviour. A timer keeps the period it was
created with, so a new `rate_hz` is stored and the publication rate stays where
it was. `add_on_set_parameters_callback` runs on every set. It cancels the timer
and creates a new one at the new period, which is what makes the rate change
take effect. It also rejects a rate of zero or less before the value is applied,
so the node cannot be made to divide by zero from the command line. Nav2 is
configured entirely this way.

The `try` and `finally` around `spin` means Ctrl-C shuts the node down cleanly
rather than leaving a half-destroyed node in the graph. Orphaned nodes cause
problems in later labs that are hard to attribute.

Now the consumer.

**Code 1.2: A subscriber that reports what it is actually receiving**

```python
#!/usr/bin/env python3
"""Subscribes to /range and reports the rate it is genuinely observing."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RangeMonitor(Node):
    def __init__(self):
        super().__init__("range_monitor")
        self.subscription = self.create_subscription(
            Float32, "range", self.on_range, 10)
        self.count = 0
        self.create_timer(2.0, self.report)

    def on_range(self, msg):
        self.count += 1
        if msg.data < 1.0:
            self.get_logger().warn(f"close range {msg.data:.2f}")

    def report(self):
        # Reporting zero explicitly is the point. A subscriber that says nothing
        # is indistinguishable from one that is not running.
        self.get_logger().info(f"{self.count / 2.0:.1f} Hz over the last 2 s")
        self.count = 0


def main():
    rclpy.init()
    node = RangeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

The reporting timer exists because of a debugging principle worth adopting
generally. A node that logs only when something interesting happens is
indistinguishable, when it goes quiet, from a node that has crashed or a node
whose subscription never matched. Printing the observed rate every two seconds,
including when the rate is zero, turns a silent failure into a visible one.

Build and run:

```
cd ~/arc_ws && colcon build --packages-select arc_lab1 --symlink-install
source install/setup.bash
ros2 run arc_lab1 range_source
# in a second terminal
ros2 run arc_lab1 range_monitor
```

Then inspect the graph from a third terminal:

```
ros2 node list
ros2 topic list
ros2 topic hz /range
ros2 topic echo /range --once
ros2 param get /range_source rate_hz
ros2 param set /range_source rate_hz 2.0
```

The last command should change the reported rate in the monitor within two
seconds, without restarting anything.

**[SCREENSHOT PLACEHOLDER]**
Three terminals side by side: the publisher logging its startup rate, the monitor
reporting a steady 10 Hz, and a third terminal showing `ros2 node list` and
`ros2 topic hz /range`.
*Instructor note: capture after `ros2 param set` has changed the rate to 2.0, so
the monitor line showing 2.0 Hz is visible and the runtime reconfiguration is
evident from the screenshot alone.*

### Exercise 1.2: the silent failure (20 minutes)

Now break it, deliberately, in the way that will happen to you accidentally.

First predict the outcome. Run the rule checker before touching any ROS code:

**Code 1.3: Predicting compatibility before running anything**

```python
from qos_rules import QoS, explain

sensor_publisher = QoS(reliability="best_effort")   # what a LiDAR driver offers
default_subscriber = QoS(reliability="reliable")    # what create_subscription defaults to

print(explain(sensor_publisher, default_subscriber))
```

Write your prediction down. Then change the publisher in Code 1.1 to use a best
effort profile while leaving the subscriber unchanged:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
self.publisher = self.create_publisher(Float32, "range", SENSOR_QOS)
```

Rebuild and run both nodes. The publisher will report that it is publishing. The
monitor will report 0.0 Hz. Neither will report an error.

Now find it with the command that exists for exactly this purpose:

```
ros2 topic info /range --verbose
```

Read the reliability line for the publisher and for the subscriber, and read the
publisher and subscriber counts. Record what you see in your exit task.

Fix it by giving the subscriber a matching profile, and confirm the fix.

This exercise is the reason `sensor_msgs/LaserScan` subscriptions in every later
lab specify their QoS explicitly. When your Lab 6 costmap is empty and your
`/scan` looks fine, this is the first thing to check.

### Exercise 1.3: a managed node (15 minutes)

Run the supplied managed node and watch it do nothing until told otherwise.

```
ros2 run arc_lab1 managed_beacon
# in another terminal
ros2 lifecycle nodes
ros2 lifecycle get /managed_beacon
ros2 topic hz /beacon          # nothing yet

ros2 lifecycle set /managed_beacon configure
ros2 lifecycle get /managed_beacon
ros2 topic hz /beacon          # still nothing

ros2 lifecycle set /managed_beacon activate
ros2 topic hz /beacon          # now it publishes
```

The gap between `configure` and `activate` is the whole point. After configuring,
the node has created its publisher and allocated its resources, so the topic
exists and appears in `ros2 topic list`. It is still not publishing. A topic
existing does not mean data is flowing, and treating those as the same thing is
the source of a great deal of confused debugging.

Deactivate it again and watch the rate drop to zero without the node exiting.

### Exercise 1.4: discovery isolation (10 minutes)

In one terminal:

```
echo $ROS_DOMAIN_ID
ros2 run arc_lab1 range_source
```

In a second terminal, change the domain before running anything:

```
export ROS_DOMAIN_ID=71
ros2 node list
ros2 topic list
```

The publisher is running and you cannot see it. Nothing is broken. The two
terminals are in different DDS domains and are, as far as ROS is concerned, on
different networks.

Set the domain back and confirm the node reappears. Then consider what this looks
like when it happens by accident: a node that is definitely running, a topic that
definitely exists, and a `ros2 topic list` that shows neither. Checking
`ROS_DOMAIN_ID` in every terminal is a five second habit that will save you at
least one long afternoon.

### Exit task (10 minutes)

Commit and push, then submit:

1. Your two working nodes.
2. The exact publisher and subscriber reliability lines from
   `ros2 topic info /range --verbose` in the broken state, and your prediction
   from Code 1.3.
3. One sentence on the difference between a topic that does not exist and a topic
   that exists but carries no data, and the command that distinguishes them.
4. The fault your demonstrator introduced in Stage 1, and the command that found
   it.

Run the self check first:

```
cd ~/arc_ws/src/arc-course
pytest starters/lab01/tests -v
```

---

## Troubleshooting

This is the diagnostic order for the whole course. Use it in sequence rather than
guessing, including in your project. It is short because it does not need to be
long.

**Is the node running?**

```
ros2 node list
ros2 node info /range_source
```

**Does the topic exist?**

```
ros2 topic list
ros2 topic info /range
```

**Is data flowing?**

```
ros2 topic hz /range
ros2 topic echo /range --once
```

**Are the two ends compatible?**

```
ros2 topic info /range --verbose
```

**Are the parameters what you think they are?**

```
ros2 param list /range_source
ros2 param get /range_source rate_hz
```

Beyond that, three specific problems account for most of the rest.

*`ros2 run` says the package does not exist.* You did not source the overlay
after building. Run `source install/setup.bash` from the workspace root, in every
terminal.

*Changes to your Python file have no effect.* You built without
`--symlink-install`, so `install/` holds a copy of the old file. Rebuild with the
flag.

*Two terminals cannot see each other.* Check `ROS_DOMAIN_ID` in both.

---

## Connection to Lab 2

You can now build a system of nodes that exchange messages reliably, and you can
tell the difference between a node that is broken and a node that is merely
misconfigured. That is enough structure to start putting a robot inside it.

Next week the messages start meaning something physical. A velocity command has
to become wheel rotations, which requires knowing the wheel radius and the
distance between the wheels. A LiDAR reading arrives in the sensor's own frame
and has to be interpreted relative to the chassis, which requires knowing where
the sensor is mounted. Getting either of those numbers wrong produces a robot
that drives confidently to the wrong place, with no error message anywhere, which
should now sound like a familiar category of problem.

---

## References

**Textbooks**

Macenski, S., Martín, F. and Ginés, J. (2022). *A Concise Introduction to Robot
Programming with ROS 2*. CRC Press. Chapters 1 to 3 cover the graph, the
communication patterns and workspace structure, and it is written by people who
maintain the code.

**Official documentation**

ROS 2 Jazzy documentation: https://docs.ros.org/en/jazzy
The Beginner CLI Tools and Beginner Client Libraries tutorials cover the same
ground as today at a slower pace, and are the right place to go if any part of
this lab felt rushed.

About Quality of Service settings: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
The authoritative compatibility table. Compare it against `qos_rules.py`.

Managed nodes design article: https://design.ros2.org/articles/node_lifecycle.html
Short, and explains why the lifecycle exists rather than only what the states
are.

**Repositories**

`ros2/examples` and `ros2/demos` on GitHub. Minimal, correct, maintained examples
of every pattern in this lab in both Python and C++.

**Video**

ROS 2 design and architecture talks from ROSCon, available on the Open Robotics
YouTube channel. Prefer the maintainer talks over general tutorials; they explain
the reasoning behind decisions rather than only the syntax.

---

## Further reading

`docs/references.md` has a fuller list under **Lab 1. ROS 2 middleware**, with
papers, industry write-ups and the documentation worth keeping open. Every entry
says what you get from it and which part of the lab it connects to.

If you read one thing, read *Dependency Chain Analysis of ROS 2 DDS QoS
Policies* (Lee, Kang and Park, 2025). It maps how sixteen QoS policies depend on
each other, which is the theory behind the silent failure in Exercise 1.2.

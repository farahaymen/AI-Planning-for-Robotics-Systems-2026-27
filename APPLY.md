# Fix round 2: stale processes, not a launch bug

## What was actually wrong

My previous diagnosis was wrong. The duplicated nodes were not caused by the
launch file structure. The evidence:

```
/controller_manager        x1    lives INSIDE the gazebo process
/gz_ros_control            x1    inside gazebo
/diff_drive_controller     x1    inside gazebo
/joint_state_broadcaster   x1    inside gazebo
/robot_state_publisher     x2    SEPARATE process
/ros_gz_bridge             x2    SEPARATE process
```

The nodes that duplicated are exactly the ones that live in their own process.
`pkill -f "gz sim"` kills only Gazebo, so `robot_state_publisher` and
`parameter_bridge` from an earlier launch survived and kept publishing. Every
new launch added another set. Two bridges publishing `/scan` is why it measured
20 Hz with 2 publishers, and `/imu` 200 Hz.

The launch file needs no change.

## Two files

```
scripts/arc-clean            new: stops every process a launch starts
scripts/course-smoke-test    updated: refuses to run when stale nodes exist
```

Copy into your Windows clone, then:

```
git add -A
git commit -m "Add arc-clean; smoke test refuses to run with stale nodes"
git push
```

## On the VM, after pulling

```bash
cd ~/arc_ws/src/arc-course && git pull
sudo install -m 0755 scripts/arc-clean        /usr/local/bin/arc-clean
sudo install -m 0755 scripts/course-smoke-test /usr/local/bin/course-smoke-test
```

## Use this from now on

Replace `pkill -f "gz sim"` everywhere with:

```bash
arc-clean            # stop everything a launch started
arc-clean --check    # list what is still running, change nothing
```

## Two habits worth keeping

**Never background a launch with `&`.** That is how the orphans were created.
Use a separate terminal tab instead.

**Always use `timeout` with `--once`.** `ros2 topic echo /clock --once` blocks
forever when nothing is publishing, which looks like a hang:

```bash
timeout 5 ros2 topic echo /clock --field clock.sec --once
```

## The preflight check

`course-smoke-test` now lists live ROS nodes before launching anything and
refuses to run if a previous launch is still alive. It checks nodes rather than
process names on purpose: `pgrep -f robot_state_publisher` also matches a text
editor with that file open, and a false refusal is worse than no check.

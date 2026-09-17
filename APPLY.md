# Fix round 4: arc-clean missed the relay

## What was wrong

```
$ arc-clean
warning: 2 ROS node(s) still visible:
/cmd_vel_relay
/cmd_vel_relay
```

`arc-clean` killed the launch process but not the nodes that launch had spawned.
`cmd_vel_relay` was not in its pattern list, so two copies survived from earlier
attempts. Same root cause as round 2, one level deeper: killing a parent does
not kill its children.

## The fix

Rather than adding `cmd_vel_relay` and waiting to be bitten again by the next
node a lab adds, `arc-clean` now also matches `arc_ws/install`. Every node run
from the course workspace has that path in its command line, so the pattern
catches all of them, including ones that do not exist yet.

The named patterns for `cmd_vel_relay`, `sensor_doctor` and `drift_meter` stay
as a belt-and-braces measure for the case where a node is run directly from
source rather than from the install tree.

## Apply

One file: `scripts/arc-clean`

```
cd /d F:\arc-course
xcopy /E /Y "%USERPROFILE%\Desktop\arc-fixes-4\scripts" scripts\
git add -A
git commit -m "arc-clean: also match nodes launched from the workspace"
git push
```

On the VM:

```bash
cd ~/arc_ws/src/arc-course && git pull
sudo install -m 0755 scripts/arc-clean /usr/local/bin/arc-clean
```

No rebuild needed; this is not a package file.

## Verify

```bash
arc-clean
ros2 node list
```

`ros2 node list` must print nothing. Then:

```bash
ros2 launch arc_gazebo simulation.launch.py headless:=true
```

and in another terminal, once the braking messages appear:

```bash
ros2 node list | grep -c cmd_vel_relay     # exactly 1
ros2 topic info /cmd_vel                   # ONE type: geometry_msgs/msg/Twist
```

Then stop and run the gate:

```bash
arc-clean
course-smoke-test --headless
```

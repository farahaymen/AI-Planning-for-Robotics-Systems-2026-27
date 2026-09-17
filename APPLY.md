# Fix round 3: the robot would not move

## What was wrong

```
$ ros2 topic info /cmd_vel
Type: ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped']
Publisher count: 1
Subscription count: 1
```

Two message types on one topic. The publisher was sending `Twist`; the
controller was listening for `TwistStamped`. They never connected, so the
controller received nothing and kept braking.

`diff_drive_controller` in this version subscribes to **TwistStamped only**. The
`use_stamped_vel` parameter that used to select the unstamped form has been
removed, confirmed by:

```
$ ros2 param list /diff_drive_controller | grep -i stamp
  twist_covariance_diagonal
```

Our config set `use_stamped_vel: false` and it was silently ignored.

## Why this mattered beyond one test

Almost everything in the course publishes an unstamped `Twist`:

| Publisher | Used in |
|---|---|
| `teleop_twist_keyboard` | Lab 3 driving, Lab 5 mapping run |
| `arc_lab5 drift_meter` | Lab 2 and Lab 5 odometry measurement |
| `arc_rl RosNavEnv` | Lab 9 and Lab 10 policy execution |
| Nav2 `controller_server` | Lab 6 onward |

All four would have failed the same silent way. `teleop_twist_keyboard` is not
ours to change, so switching everything to TwistStamped was not an option.

## The fix: convert at the boundary

```
anything --Twist--> /cmd_vel --[cmd_vel_relay]--> TwistStamped --> controller
```

One small node fills in the header stamp, which is the only thing the unstamped
message lacks. Everything else keeps publishing the type it already publishes,
and `/cmd_vel` keeps the type students see in every tutorial.

## Files

```
arc_gazebo/scripts/cmd_vel_relay.py            new
arc_gazebo/CMakeLists.txt                      installs the relay
arc_gazebo/launch/simulation.launch.py         starts it; cmd_vel remap removed
arc_description/config/arc_bot_controllers.yaml  dead parameter removed
scripts/course-smoke-test                      checks the relay and the type
scripts/arc-clean                              from round 2, include if not yet pushed
```

## Apply

On Windows, from your clone:

```
xcopy /E /Y "%USERPROFILE%\Desktop\arc-fixes-3\arc_gazebo"      arc_gazebo\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-fixes-3\arc_description" arc_description\
xcopy /E /Y "%USERPROFILE%\Desktop\arc-fixes-3\scripts"         scripts\

git add -A
git commit -m "Add cmd_vel_relay: diff_drive_controller is TwistStamped-only"
git push
```

On the VM:

```bash
cd ~/arc_ws/src/arc-course && git pull
sudo install -m 0755 scripts/arc-clean         /usr/local/bin/arc-clean
sudo install -m 0755 scripts/course-smoke-test /usr/local/bin/course-smoke-test
cd ~/arc_ws && colcon build --symlink-install && source install/setup.bash
```

A rebuild is required this time: `install(PROGRAMS)` is a CMake change, so
`--symlink-install` alone will not place the new script.

## Verify

```bash
arc-clean
ros2 launch arc_gazebo simulation.launch.py headless:=true
```

Then, in another terminal:

```bash
ros2 topic info /cmd_vel          # ONE type now, geometry_msgs/msg/Twist
ros2 node list | grep relay       # /cmd_vel_relay
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

The braking messages should stop and `/odom` x should climb by about 0.2 per
second. Then:

```bash
arc-clean
course-smoke-test --headless
```

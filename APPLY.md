# arc-fixes, round 3

Two things in this bundle. One unblocks the simulator, one is new lab material.

## 1. The LiDAR fix (this is the one that matters)

`arc_description/urdf/arc_bot.urdf.xacro`

Your last smoke test said:

```
/scan sees the room   FAIL   min 0.12 m, max 0.12 m - every beam reads the same,
                             the LiDAR is inside geometry
```

Every other check passed, so the robot drives, the controllers run, the clock
advances and the TF tree resolves. It just cannot see.

**What was wrong.** The laser link's visual cylinder was centred on the link
origin, which is where the beams start. Gazebo's `gpu_lidar` raycasts against
the **rendered** scene, and the rendered scene is built from **visual** geometry.
So all 360 beams started inside the sensor's own housing and returned the
configured minimum range, 0.12 m, forever. The scan looked healthy: 360 beams, a
perfect 10 Hz, plausible numbers. In RViz it is a tight red ring that never
changes as the robot drives.

The trap is that removing the `<collision>` element does not fix it. Collision
geometry is what a CPU raycast sensor uses. I got this wrong first time round and
your `gz sdf -p` output is what proved it: the collision was gone and the scan
still read 0.12.

**The fix.** Offset the visual 0.04 m downward so the housing sits below the beam
plane, and drop the collision element entirely, since a sensor mount needs
neither. Housing now spans 0.175 to 0.205 m above the ground, the beam plane is
at 0.230 m, so there is 0.025 m of clearance.

After applying it, the smoke test line should read something like
`min 0.35 m, max 9.60 m`.

### Why it did not land last time

Your run was from `~/.local/share/Trash/files/arc_ws/src/arc-course`, and the
commit was still `3fa5149`, and `arc-setup` reported `0 file(s) needed it`. Those
three together say the file never reached the commit you pushed. Worth checking
on the Linux side before you copy anything:

```bash
grep -c "0 0 -0.04" ~/arc_ws/src/arc-course/arc_description/urdf/arc_bot.urdf.xacro
```

`1` means it is already there and you only need `arc-setup`. `0` means copy it
over on Windows as below. Either way, there is a second copy of the whole
workspace sitting in your Trash; delete it, so that a future `cd` into the wrong
one cannot cost you an evening again.

## 2. Lab 4 planning materials, new

`starters/lab04_planning/`

Complete and tested: 93 tests passing against the reference, all four figures
generated from the code rather than drawn. Pure Python, no ROS, so it runs
anywhere. See `starters/lab04_planning/README.md` for what each of the six maps
is for and why.

Contents:

```
gridmap.py              six maps, chosen so the algorithms actually disagree
planners_skeleton.py    what students get. Nine TODOs
planners.py             reference implementation
tests/test_planners.py  93 tests, doubling as the specification
compare.py              the comparison table, and ASCII paths
figures.py              regenerates the four figures
figures/                the four figures, generated
README.md               the exercise
```

## Copying it across

From the root of your Windows clone:

```
xcopy /E /Y path\to\arc-fixes\arc_description arc_description\
xcopy /E /Y path\to\arc-fixes\arc_gazebo      arc_gazebo\
xcopy /E /Y path\to\arc-fixes\starters        starters\
xcopy /E /Y path\to\arc-fixes\scripts         scripts\
xcopy /E /Y path\to\arc-fixes\arc_lab1        arc_lab1\
xcopy /E /Y path\to\arc-fixes\arc_lab3        arc_lab3\
xcopy /E /Y path\to\arc-fixes\arc_lab4        arc_lab4\
xcopy /E /Y path\to\arc-fixes\arc_lab5        arc_lab5\
xcopy /E /Y path\to\arc-fixes\arc_lab7        arc_lab7\

git status --short
```

**Read that `git status` before committing.** It has to list
`arc_description/urdf/arc_bot.urdf.xacro`. If it does not, the copy did not land
and pushing will change nothing. This is the third time a file has looked copied
and not been in the commit, so it is worth the ten seconds.

```
git add -A
git commit -m "Fix LiDAR visual occluding the beam plane; add Lab 4 planning materials"
git push
```

Then on the VM:

```bash
arc-setup
```

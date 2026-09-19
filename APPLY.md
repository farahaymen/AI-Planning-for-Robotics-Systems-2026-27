# arc-fixes, round 4

Everything needed to make the environment pass, plus the Lab 4 planning
material. Copy the ten items **inside** the `arc` folder into the root of your
clone, replacing when asked, then commit and push.

## VM setting, not in this zip

Before anything else, with the VM powered off:

**Settings, Display, untick 3D acceleration.** Video memory 128 MB.

This is load bearing. VirtualBox's SVGA3D driver advertises OpenGL 4.1 core but
its implementation is incomplete, and ogre2 silently renders nothing on it.
With 3D acceleration off the guest falls back to llvmpipe, which gives a
complete 4.5 core profile. Measured real time factor is about 0.92 either way,
so it costs the simulation nothing. The desktop feels slower because GNOME is
composited in software; the labs run headless.

## What changed

**`arc_gazebo/launch/simulation.launch.py`**
Removed `--headless-rendering` and added a `render_engine` argument defaulting
to `ogre2`. The flag forced the EGL path, EGL insisted on `/dev/dri/card0` and
could not produce a context, so the LiDAR rendered nothing and every beam
returned its configured minimum of 0.12 m at a perfect 10 Hz. The full
measurement table is in the file's comments and in
`docs/vm_graphics_and_gpu_sensors.md`.

**`scripts/course-smoke-test`**
Four fixes, each from a real failure this weekend:

- Rate measurement took the first `average rate:` line, which is the shortest
  and least settled window, and it landed while the test was still starting
  processes. It reported `/odom` at 33.3 Hz where a twenty second measurement
  gives 49.7 Hz with a standard deviation under a millisecond. It now reads
  several reports and returns a settled one.
- `ros2 topic echo` truncates arrays at 128 elements by default, so the scan
  check had only ever inspected 128 of 360 beams without saying so. It now
  passes `--full-length` and reports the beam count.
- Timeouts raised: 40 s to settle, 90 s for controllers. The old 45 s limit
  failed on software rendering, which looked like a broken environment and was
  a broken test.
- New `graphics: renderer` and `graphics: OpenGL core profile` checks, run
  before the simulation starts, so that when the LiDAR check fails the reason
  is already on screen. The renderer check names the VM setting to change.

**`docs/vm_graphics_and_gpu_sensors.md`** (new)
The whole diagnosis, the measurement table, and why software rendering is the
right answer here rather than a compromise.

**`starters/lab04_planning/`** (new)
Lab 4 planning exercise. Pure Python, no ROS, no simulator, so it runs on any
machine including a lab PC with nothing installed but Python and NumPy.
93 tests passing, four figures generated from the code. See its README for what
each of the six maps is for.

## Copying it across

Open the unzipped `arc` folder, select the ten items inside it, and copy them
into the folder that contains your `.git`. Choose **Replace the files in the
destination**. Copy the contents, not the `arc` folder itself.

Then, in PowerShell from that folder:

```powershell
git add -A
git commit -m "Fix LiDAR render path; add graphics checks and Lab 4 planning"
git push
```

On the VM:

```bash
cd ~
arc-setup
```

## One thing to verify afterwards

The vertical-samples block you added to `arc_bot.gazebo.xacro` while we were
diagnosing this was a workaround for Ogre 1.x. This zip ships the standard
single-row 2D scan, which is what SLAM Toolbox expects and what a real LiDAR
produces, and it renders four times less. ogre2 should handle it.

If `/scan sees the room` fails after applying this, that assumption was wrong
and the fix is to put the block back:

```bash
cd ~/arc_ws/src/arc-course
python3 - <<'EOF'
import pathlib
p = pathlib.Path("arc_description/urdf/arc_bot.gazebo.xacro")
s = p.read_text()
old = "          </horizontal>\n        </scan>"
new = ("          </horizontal>\n          <vertical>\n"
       "            <samples>4</samples>\n            <resolution>1</resolution>\n"
       "            <min_angle>-0.02</min_angle>\n            <max_angle>0.02</max_angle>\n"
       "          </vertical>\n        </scan>")
assert old in s
p.write_text(s.replace(old, new))
print("vertical block restored")
EOF
cd ~/arc_ws && colcon build --packages-select arc_description --symlink-install --allow-overriding arc_description
```

Tell me either way and I will make it permanent.

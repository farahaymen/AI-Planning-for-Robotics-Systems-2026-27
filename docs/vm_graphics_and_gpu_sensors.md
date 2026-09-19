# VM graphics and GPU sensors

This page exists because a silent sensor failure cost a full day of debugging
before the semester. Everything here was measured on the reference VM, not
inferred. Read it before changing the render engine, the launch flags, or the
VM's display settings.

## The required configuration

| setting | value |
| --- | --- |
| VirtualBox, Display, 3D acceleration | **off** |
| Graphics controller | VMSVGA |
| Video memory | 128 MB |
| Sensor render engine | **ogre2** |
| `--headless-rendering` | **never passed** |

With that configuration the reference VM measures a real time factor of about
0.92, `/odom` at 49.7 Hz with a standard deviation under a millisecond, `/scan`
at 9.93 Hz, and a LiDAR reading 0.91 m to the nearest wall, which matches the
world geometry to within a centimetre.

## What goes wrong, and why it is hard to see

A GPU LiDAR does not trace rays against collision geometry. It renders the
scene from the sensor's viewpoint and reads the depth buffer. If that render
silently produces nothing, every ray reads zero distance, and the sensor clamps
zero to its configured `range_min`.

The result is a scan that passes every superficial check:

- the topic exists
- the publication rate is exactly right, 10.0 Hz
- the message carries exactly 360 beams
- every value is a plausible number, 0.12 m
- no error appears in any log

and the robot is completely blind. In RViz it is a tight ring that never
changes as the robot drives. `ros2 topic hz` is perfect. Only checking the
spread of the values catches it, which is why `course-smoke-test` does.

## The measurements

Taken with Gazebo's own `gpu_lidar_sensor.sdf`, which contains none of this
course's code, so that the robot, the world, the URDF and the ROS bridge were
all excluded as causes before any of them was adjusted.

| engine | context | graphics | result |
| --- | --- | --- | --- |
| ogre2 | EGL | SVGA3D | `eglInitialize failed` on `/dev/dri/card0` |
| ogre2 | EGL | llvmpipe forced | `eglInitialize` fails, then segfaults |
| ogre2 | GLX | SVGA3D | renders nothing, silently |
| ogre | GLX | SVGA3D | works at 160 degrees, 75 percent dead at 360 |
| ogre2 | GLX | llvmpipe | works, full 360 degrees |

Three facts follow.

**`--headless-rendering` forces EGL.** EGL then enumerates `/dev/dri/card0`,
the virtual GPU, insists on using it, and cannot produce a context. Removing
the flag leaves `-s`, which gives a server with no GUI. That is all "headless"
ever needed to mean here.

**ogre2 needs a conformant OpenGL 3.3 core context.** VirtualBox's SVGA3D
driver advertises 4.1 core but its implementation is incomplete, and ogre2
renders nothing on it without complaining. Turning 3D acceleration off falls
back to llvmpipe, Mesa's software rasteriser, which provides a complete 4.5
core profile. Conformance is the point, not speed.

**Ogre 1.x is not a workaround.** It renders, because it only needs OpenGL 2.x,
but it cannot produce a single-row render target and it stitches a full circle
from cube faces badly. Measured at 360 degrees, three quarters of the rays were
still dead.

## Why software rendering is acceptable here

The intuition that software rendering must be too slow is wrong for this
workload, and the measurement says so: real time factor about 0.92, against
0.85 to 1.0 with hardware acceleration.

The render load is small. A 360 beam scan at 10 Hz is 3,600 rays per second
into a low-polygon warehouse. What llvmpipe finds expensive is compositing a
full GNOME desktop, which is why the VM feels sluggish while the simulation
itself does not. The labs run headless for anything measured.

There is also a benefit worth stating plainly. Software rendering removes the
graphics driver as a variable. Every seat behaves identically regardless of
what hardware it has, which is exactly what a Golden VM is for.

## If you move off VirtualBox

Nothing above is VirtualBox-specific except the SVGA3D driver. On a machine
with a real GPU, ogre2 works without any of this, and `--headless-rendering`
becomes usable again because EGL can then get a context from a real device.

The `render_engine` launch argument exists for that case:

```bash
ros2 launch arc_gazebo simulation.launch.py render_engine:=ogre
```

That is for diagnosis only. Do not ship it: it cannot render this robot's scan
correctly.

## How this is now caught

`course-smoke-test` checks three things that together make a silent recurrence
impossible:

- `graphics: renderer` fails if the renderer is SVGA3D, and says which VM
  setting to change.
- `graphics: OpenGL core profile` fails below 3.3.
- `/scan sees the room` fails unless the ranges have a real spread, and reports
  how many beams it parsed so a truncated reading cannot masquerade as a
  complete one.

The graphics checks run before the simulation starts, so when the LiDAR check
fails the explanation is already on screen.

## The second casualty: RViz's Map display

Software rendering fixed the sensors. It did not fix every consumer of a GPU,
and one of them matters to Lab 5.

With SLAM running, RViz logs:

```
[ERROR] GLSL link result: active samplers with a different type refer to the
        same texture image unit
```

and the Map display shows grey static rather than the occupancy grid.

This is not a SLAM failure and not a topic problem. RViz draws the Map display
through Ogre with a shader that binds two samplers of different types to one
texture unit. Hardware drivers tolerate that; Mesa's GLSL compiler, which is
what llvmpipe uses, rejects the link and the display falls back to an
uninitialised texture. The map on `/map` is correct throughout. Only the
picture of it is wrong.

Confirm it rather than believing it. With the map still running:

```bash
ros2 topic echo /map --field info --once
```

A plausible `width`, `height` and `resolution` means SLAM is working and the
fault is entirely in the drawing.

Two things reduce the damage:

- **Map display, Color Scheme: `raw`.** The `map` scheme is the one that needs
  the second sampler. `raw` sometimes links where `map` does not, and costs
  nothing to try.
- **`arc-map-view`.** Draws `/map` with matplotlib, which is a software
  rasteriser with no OpenGL path at all, so it cannot fail this way on any
  machine. It also writes the `.pgm` and `.yaml` pair that Nav2 loads, a
  contact sheet, and an animation of the map assembling itself.

The rest of RViz is unaffected: the laser scan, the TF tree, the robot model
and the path displays all render normally. It is specifically the Map display.

For teaching, `arc-map-view` is the better tool regardless of the bug. Watching
the floor area climb while you drive tells a student when a room is finished;
a picture of a map does not. And the loop closure, which is the moment Lab 5
exists to show, is over in one frame. Two contact sheet tiles either side of it
make the point permanently, where a live window makes it once.

# Images for the main README

Drop files here using the names below and the placeholders in
[`../../README.md`](../../README.md) become real pictures. Each slot in the
README names the file it wants, so nothing has to be wired up.

Nothing in this folder is installed by CMake or used at runtime — these are
documentation only.

## The slots

| File | What it should show | Where it appears |
|---|---|---|
| `robot-hero.jpg` | The assembled robot, three-quarter view, on the floor with the lidar facing the camera-left. The one picture that answers "what is this?" | Top of the README |
| `cad-iso.png` | Isometric render from Fusion 360, plain background, arm in a neutral pose | Gallery |
| `cad-top.png` | **Top-down** render with the three wheels visible. Worth having because the wheel angles are the thing people get wrong | Gallery, and the wheel-angle section |
| `robot-real.jpg` | The robot as actually built, so the CAD and reality can be compared | Gallery |
| `rviz-teleop.png` | RViz with the robot model, a `/scan` and the camera image visible at once | Gallery, Driving section |
| `gazebo.png` | Gazebo Harmonic with the robot spawned in `mmr_world.sdf` | Gallery, Gazebo section |
| `frame-tree.png` | Annotated frame diagram: `base_footprint`, `base_link`, the three wheels at 60/180/300°, `laser_frame` | Frame tree section |
| `wiring.jpg` | The ESP32 and motor wiring, close enough to read which driver output goes to which wheel | Wheel wiring section |

## A note on `wiring.jpg`

That one is not decoration. `tools/firmware_diff.py` reports that the firmware
and the URDF disagree about which way is left, and the wiring is the missing
evidence that would settle it — the mapping from ESP32 output index to physical
wheel is not in any source file. A photo clear enough to trace the leads is
genuinely useful to anyone picking this up, including you in six months.

## Suggested handling

- Screenshots (`rviz-*`, `gazebo-*`, `cad-*`) as **PNG**; photographs as **JPG**.
- Around **1600 px** on the long edge is plenty. Full-resolution phone photos
  bloat the clone for no visible benefit.
- The README renders these at roughly 380–760 px wide, so anything larger is
  only useful for zooming.

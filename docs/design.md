# Design notes

<sub>[← README](../README.md) · [Quick start](../README.md#quick-start) · [Troubleshooting](troubleshooting.md) · [SLAM & Nav2](slam-and-nav2.md) · [Configuration](configuration.md) · [Hardware](hardware.md) · **Design** · [Status](status.md)</sub>

---

Why things are the way they are. None of this is needed to drive
the robot; each entry is here because something went wrong without it.

## What talks to what (real robot)

```
  PC ─────────────────────────────┐        ┌──────────────── Raspberry Pi
                                  │  WiFi  │
  kb_teleop  (mmr_pkg)            │        │   robot_state_publisher ─┐
        │ /cmd_vel   20–30 Hz     │  DDS   │   joint_state_publisher ─┼─▶ /tf
        └─────────────────────────┼───────▶│   sllidar_ros2 ──────────┼─▶ /scan
                                  │        │   v4l2_camera ───────────┴─▶ /image_raw
  rviz2 ◀── /scan /image_raw ─────┤        │   esp32_bridge
            /tf /robot_description│        │        │ UDP :1234
            /diagnostics ─────────┤        └────────┼──────────────────
                                  │                 ▼
                                  └────────── ESP32  "A,angle,speed,rot"
                                                     │ PWM ×3
                                                     ▼  three omniwheels
```

```bash

# Pi
ros2 launch mmr_pkg robot.launch.py

# PC, terminal 1
ros2 launch mmr_pkg teleop.launch.py

# PC, terminal 2 — must keep keyboard focus
ros2 run mmr_pkg kb_teleop
```

## Why the logic lives outside the nodes

`bridge_core.py` and `teleop_keys.py` import no ROS. The nodes that wrap them
(`esp32_bridge.py`, `kb_teleop.py`) contain no arithmetic. This split is load
bearing, and it exists because of a specific failure:

> `esp32_bridge.py` read `self.max_v`, `self.max_v_pwm`, `self.max_w` and
> `self.max_w_pwm` on the last statement of `__init__`, after those values had
> moved onto a collaborator object. The constructor raised `AttributeError`
> before `rclpy.spin()` was ever reached — **the node had never once been able
> to start** — and the test suite was green throughout, because importing the
> node requires `rclpy` and the tests run on a laptop that does not have it.

A test suite that cannot import the thing it is testing will report success
forever. So the policy — watchdog, limiter, smoother, link state, key map — sits
in files that a bare Python interpreter can import, and all 119 tests exercise
the real code rather than a mock of it. Two further guards close the gap that
remains:

```bash
python tools/check_repo.py     # static sweep: no ROS, no build, under a second
python -m pytest test/ -q      # 119 cases
```

`check_repo.py` parses every class and fails on any `self.x` that is read but
never assigned, which is exactly the bug above. It also rejects `--` inside an
XML comment (illegal, not merely discouraged — it has broken `package.xml` and
`ros2_control.xacro` in this repo on three separate occasions, with an error
message that names a line number but not the problem), catches a script that is
not in `install(PROGRAMS …)`, a shim importing a module that does not exist, a
`test_*.py` not registered with `ament_add_pytest_test`, `sllidar_ros2` creeping
back into `package.xml`, and a `build/` directory in the source tree. Every one
of those is a mistake that was actually made here, not a hypothetical.

Neither command needs ROS, hardware, or a build, so there is no excuse to skip
them. Neither proves a single packet reaches the ESP32.

## The IK, and how to diff it against your firmware

The brief asked for the kinematics in one clearly commented function so it can
be compared against the ESP32. It is `inverse_kinematics()` in
**`mmr_pkg/kiwi_kinematics.py`** — a plain Python module with **no ROS imports
at all**, so you can run it standalone:

```bash
python -c "
from mmr_pkg.kiwi_kinematics import KiwiKinematics
k = KiwiKinematics()
print(k.inverse_kinematics(0.2, 0.0, 0.0))   # forward 0.2 m/s
"
```

The whole of it:

```python
w_i = (vx·sin(α_i) − vy·cos(α_i) − L·ωz) / R
```

with `α = (60°, 180°, 300°)`, `L = 0.135500 m`, `R = 0.030 m`. `kiwi_drive_node.py`
is a *ROS wrapper only* and contains no kinematics — that separation is
deliberate, so the file you diff is small.

**If your firmware disagrees, check the sign convention first**, not the
algebra. See [Wheel sign convention](hardware.md#wheel-sign-convention) — and note the
[30° wheel-angle discrepancy](hardware.md#the-30-wheel-angle-discrepancy--read-this-before-driving)
between the brief and the STEP, which is the single most likely source of a
mismatch.

`test/test_kiwi_kinematics.py` has 16 tests (round-trip to 1e-12, closed form vs
pseudo-inverse, per-wheel direction, and a test that re-reads the expanded URDF
so the module and `base.xacro` cannot drift apart). They need no ROS:

```bash
python -m pytest test/ -q      # 16 passed
```

## Option B — moving the kinematics into ROS

The better long-term design, once the wiring above is settled: change the
sketch to accept three wheel PWMs instead of a polar command, and let
`kiwi_kinematics.py` do the mixing. `kiwi_kinematics` then becomes the single
source of truth for both Gazebo and hardware, the geometry (`R`, `L`) enters the
maths properly, and `rot` stops being an arbitrary knob whose ratio to `speed`
has no physical meaning. It costs one reflash. Not done here because it was not
needed to answer the question, and because it is unsafe to do before the wiring
is confirmed.

## The path to real odometry, and why none of it is faked

There is no *wheel* odometry: no wheel motion in `/joint_states` beyond the zeros
that keep the TF tree connected, and nothing in `robot.launch.py` publishes
`odom → base_footprint`. That is deliberate. This robot has no encoders, so any
wheel-derived pose would be a number invented by software and presented as a
measurement — and integrating `/cmd_vel` would be exactly that, a report of what
we *asked* the robot to do rather than what it did. On a link that loses ~7.7% of
its packets those are very different things. Fake odometry is worse than none,
because everything downstream — SLAM, Nav2, any `map` frame — silently trusts it.

`slam.launch.py` does publish `odom → base_footprint`, and it is not an exception
to the above: `rf2o_laser_odometry` estimates planar motion from the range flow
between consecutive lidar scans, which is a real measurement in real metres taken
by a real sensor. It is a *different* measurement from encoders, with different
failure modes — see [SLAM and Nav2](slam-and-nav2.md) — but it is a measurement.
The rest of this section is about the encoder path, which is still absent.

What *is* in place is the shape the real thing plugs into. The chain is:

```
  encoders ─▶ /joint_states ─▶ kiwi FK ─▶ nav_msgs/Odometry ─▶ odom → base_link
             (real positions)   (exists)      (to write)          (to write)
```

The middle link already exists and is tested: `kiwi_kinematics.py` holds both
directions of the kinematics, so forward kinematics is not new work. What is
missing is genuinely missing — the hardware. To close it you would:

1. Add encoders and report their counts from the ESP32. The current protocol has
   no room for this; the ACK would need to carry wheel positions, which is a
   firmware change.
2. Publish real positions on `/joint_states` and **drop `joint_state_publisher`
   from `robot.launch.py`** — the two would fight over the same topic.
3. Integrate FK to a pose and publish `nav_msgs/Odometry` plus the
   `odom → base_link` transform.

Until step 1 exists, steps 2 and 3 have nothing truthful to publish.

Worth doing even though `rf2o` already produces an `odom` frame, because the two
are not redundant. `rf2o` fails where scan matching fails — blank corridors, open
halls, fast rotation — and it is not independent of `slam_toolbox`'s own scan
matcher, so the two agreeing is not evidence that either is right. Encoders fail
in the opposite places (wheel slip, carpet) and are genuinely independent, which
is what makes fusing them worth the firmware change.

**The camera needs no URDF link to be visible.** RViz's *Image* display carries
no TF filter and renders whatever arrives. The *Camera* display is the one that
needs both TF and `CameraInfo` — and it additionally needs a real intrinsic
calibration, which this camera does not have: `v4l2_camera` publishes an
all-zero `CameraInfo` when calibration is missing, and RViz then computes
`p[3]/p[0]` = `0/0`, rejecting the frame as *"invalid position calculation
(nans or infs)"*. So the Image display is not a workaround, it is the correct
tool here. See TODO 14 if you want a real `camera_link`.

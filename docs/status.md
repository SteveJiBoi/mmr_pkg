# What is and isn't verified, and what is still assumed

<sub>[← README](../README.md) · [Quick start](../README.md#quick-start) · [Troubleshooting](troubleshooting.md) · [SLAM & Nav2](slam-and-nav2.md) · [Configuration](configuration.md) · [Hardware](hardware.md) · [Design](design.md) · **Status**</sub>

---

## Assumptions and TODOs

Marked `TODO` in the source, listed here so none of them hide.

1. ~~**`laser_frame` z is approximate.**~~ **RESOLVED in Phase 2.** The Slamtec
   C1 drawing (rev 1.2, Fig 4-1) dimensions the scan plane at 29.800 mm above
   the mounting face; the collision hull's lowest face is a genuinely planar
   48-vertex surface at −28.179 mm spanning exactly ±27.800 mm, i.e. the
   datasheet's 55.6 mm mounting base. So `laser_z = 0.106801 m`, and as a
   cross-check that puts the mounting face at `base_link` z = 0.077001 m — the
   top plate's upper surface to within 1 micron. The Phase 1 turret-mid-height
   guess was low by 1.621 mm.
2. **Inertials are present, but ONE MASS IS UNSOURCED.** `use_inertial` is now
   `true`. The tensor *shapes* are exact — integrals over the actual Phase 1
   collision geometry via the parallel-axis theorem
   (`tools/estimate_inertia.py`), deliberately not over the decimated,
   non-watertight visual mesh (trimesh returns a number for that regardless,
   which is the trap). The *scales* are the issue:
   - `base_link` 1.0 kg — from the brief. Sourced.
   - `laser_frame` 0.110 kg — datasheet typical. Sourced.
   - **`wheel_mass` 0.100 kg — invented. Please weigh one wheel.** This is the
     only unsourced number in the package. It is flagged in `base.xacro`, in
     `estimate_inertia.py` output, and here. It affects how the robot
     accelerates in Gazebo but nothing about its geometry.

   `inertial_from_fusion` is still available in `inertial_macros.xacro` if you
   want to paste real Fusion 360 numbers; it handles both conversions (kg·mm²
   → kg·m², and Fusion's **opposite sign convention** for products of inertia,
   Ixy_urdf = −Ixy_fusion).
3. **A camera is modelled although the brief says there is none.** An Innomaker
   U20CAM-1080P on a `BaseCamMount`, facing forward. It is currently merged into
   the `base_link` visual, with its hull in `camera_pod.stl`. Say the word and it
   can be split into a proper `camera_link` + optical frame — the mount transform
   is measurable from the same STEP. See TODO 14: you do **not** need this to see
   the video feed in RViz.
4. **`arm_gripper_frame_link` has a zero inertia tensor** (mass 1e-9). That is
   upstream's dummy frame, kept verbatim. Harmless for RViz, and harmless in
   Gazebo too: it is attached by a fixed joint, so it gets lumped into its
   parent and never reaches the physics engine as a body. Deliberately *not*
   given a `<preserveFixedJoint>` — preserving a fixed joint whose child is
   effectively massless makes sdformat delete the link outright.
5. **Maintainer email and licence** in `package.xml` are placeholders. The
   vendored arm assets are Apache-2.0 from TheRobotStudio.
6. **No battery** is modelled in the CAD, so none is described.
7. **Calibration variant unconfirmed.** §5.4 asked me to check which of
   `so101_new_calib.urdf` / `so101_old_calib.urdf` matches your physical arm; I
   have not had an answer, so `new_calib` is in place as the upstream default.
   This is not cosmetic — the two differ in link names, in `gripper_frame_link`
   (absent in `old_calib`) and in joint origins, so if your LeRobot calibration
   is the old one the arm's pose will be wrong. Both files are kept in
   `tools/upstream/` and switching is a one-line change to `SRC` in
   `tools/gen_arm_xacro.py` plus a re-run.

Added in Phase 2:

8. **Friction coefficients are modelling choices, not measurements.**
   `roller_mu = 0.05`, `rolling_mu = 1.0` in `gazebo.xacro`. Nothing in the
   STEP or any datasheet gives a friction coefficient. Tune against the real
   robot.
9. **`max_wheel_speed` defaults to 0.0, meaning unlimited.** No motor datasheet
   was supplied, so any limit would be invented. When you know the real figure,
   set it as a ROS parameter on `kiwi_drive_node`; the node then scales the
   whole twist uniformly rather than clamping wheels individually, which would
   silently change the direction of travel.
10. **`gz:expressed_in` on `fdir1` is traced through source but never run** —
    the attribute-preserving code path was read in `parser_urdf.cc` and the
    frame semantics in DART's `ContactSurface.cpp`, but no Gazebo executed
    here. See
    [Omniwheel friction](hardware.md#omniwheel-friction--the-part-most-likely-to-need-tuning)
    for the one-line `gz sdf -p` check and the exact fallback.
11. **No hardware interface for the real robot.** `sim:=false` intentionally
    names a plugin that does not exist so it fails loudly. Phase 3 sidesteps
    this entirely: the ESP32 is reached over UDP, not through `ros2_control`.
12. **⚠ WHEEL WIRING UNCONFIRMED — the highest-value thing you can check.**
    `tools/firmware_diff.py` shows the firmware and `kiwi_kinematics.py`
    disagree by a reflection on the lateral axis, and that exactly one of 48
    wirings reconciles them (indices 0↔2 swapped, all three motor polarities
    reversed). Which wheel sits on which ESP32 pin group is **not in the source
    code**, so this is a hypothesis. Ten minutes with the robot on blocks
    settles it. Until it is settled, do not trust any comparison between
    simulation and hardware. See
    [the finding](hardware.md#-the-firmware-and-the-simulation-disagree-about-which-way-is-left).
13. **`cmd_vel` is not yet in m/s.** The bridge's speed map is a declared
    normalisation, not a measurement — the firmware has no encoders and no
    closed loop. The procedure to make it real is in
    [Calibrating the ESP32 bridge](configuration.md#calibrating-the-esp32-bridge).
    `rf2o_laser_odometry` now makes this **measurable** for the first time: it
    reports metres travelled, independently of the PWM map, so a commanded
    `cmd_vel` can finally be compared against a distance. Until someone runs
    that comparison, every velocity in this package is a fraction of full scale
    wearing the units of m/s.
14. **No `camera_link`, deliberately.** Not needed for the RViz *Image*
    display, which has no TF filter. The *Camera* display would need one — plus
    an intrinsic calibration this camera does not have, without which RViz
    rejects the frame outright. Splitting out a `camera_link` is easy; getting
    a real calibration means running `camera_calibration` with a checkerboard.
    Ask if you want either.
15. **Acceleration limits default to 0.0 — unlimited — because there is no
    measured figure.** The robot has no encoders, so its real acceleration
    capability is unknown, and any non-zero default would be a number I made
    up. The limiter itself is implemented and tested; it is switched off
    pending a measurement. If the base lurches on a step input, raise
    `max_linear_accel` in `config/esp32_bridge.yaml` until it does not, and
    record what you used. Note that the *braking* limits are intended to stay
    at 0.0 even after that: a slow ramp down is a robot that will not stop
    when told.
16. **`key_timeout` (0.6 s) is inferred from typical terminal behaviour, not
    measured on your machine.** A terminal reports no key-up event, so release
    is detected by auto-repeat ceasing, and the timeout must exceed the initial
    repeat delay — commonly ~0.5 s, but configurable per system. Wrong in one
    direction the robot coasts after you let go; wrong in the other it stutters
    while you hold. Tune it on the robot; the value is a ROS parameter.
17. **Nav2's velocity and acceleration limits are not measured.** `vx_max`,
    `vy_max` and `wz_max` in `config/nav2.yaml` are 0.35 m/s and 1.0 rad/s
    *expressed in the same fictional units as `cmd_vel`* — see TODO 13. They are
    below `kb_teleop`'s 0.5 m/s default, which is the only defensible thing to
    say about them: Nav2 will drive more slowly than you do by hand. The
    acceleration limits are upstream's, inherited unchanged because there is no
    measurement to replace them with. Do not read any of these six numbers as a
    statement about what this robot can physically do.
18. **The Nav2 footprint assumes the arm is stowed.** `robot_radius: 0.15` is
    derived, not guessed — wheel centres at 0.1355 m plus a 0.0136 m roller
    half-envelope gives 0.1491 m — but it describes the *base*. The SO-101 can
    reach well outside that circle, and nothing in this package tells Nav2 where
    the arm currently is. Drive with the arm folded, or replace `robot_radius`
    with a `footprint` polygon sized to the arm's actual pose.
19. **The `rf2o` odometry has never been run.** Neither has `slam_toolbox` nor
    Nav2 — see below. `config/nav2.yaml` is upstream's file with fourteen
    edits, each asserted to apply exactly once and each verified by re-parsing
    the result, which proves the file says what I meant. It does not prove the
    robot navigates.

---

## What is and isn't verified

**This machine has no ROS 2.** `xacro`, `check_urdf`, `rviz2`, `colcon` and
`ros2` are all absent and were not run. To avoid claiming unverified results,
offline equivalents were written; they are development aids, and the real tools
remain the authority.

### Checked here

```bash
python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro -o generated/robot.urdf
python tools/check_urdf_lite.py generated/robot.urdf          # 0 failures, 2 warnings
python tools/check_phase2.py generated/robot.urdf config/controllers.yaml   # 0 failures
python tools/check_repo.py                        # 0 failures over 138 checks
python tools/kiwi_check.py                                        # exit 0
python tools/firmware_diff.py               # exit 1: reports a real disagreement
python -m pytest test/ -q                                    # 119 passed
```

**Description** (`check_urdf_lite.py`) — 0 failures, 2 expected warnings:

- All seven xacro files are well-formed XML and expand cleanly (32 properties,
  12 macros, 6 includes).
- One root link, no orphans, no cycles, joints = links − 1.
- No duplicate link or joint names.
- **Exactly 9 movable joints**, names and types exactly as specified; every
  revolute joint has a `<limit>`.
- **All three wheels bottom out at z = 0.000000000**, and all three spin axes
  are radially outward (dot = 1.000000) at bearings 60 / 180 / 300°.
- All 45 mesh references use `package://mmr_pkg/`, and all 45 files exist.
- No collision geometry points at a visual mesh; every `.obj` has its `.mtl`.
- No placeholder identity inertias.
- Arm xacro matches upstream on links, joint names, types, limits and origins.
- Kiwi sign convention verified against the URDF (`tools/kiwi_check.py`).

The two warnings are documented gaps: `arm_gripper_frame_link`'s zero inertia
(upstream's, and it gets lumped away in Gazebo anyway) and `base_footprint`
having no inertial (correct — sdformat allocates one).

**Control wiring** (`check_phase2.py`) — 0 failures, 0 warnings:

- The hardware plugin is `gz_ros2_control/GazeboSimSystem`, and all 9 movable
  joints are claimed by `<ros2_control>` — no more, no fewer.
- All three controller type strings match the upstream pluginlib exports.
- Every joint each controller claims exposes the command interface it needs.
- **Wheel index → URDF mounting bearing agrees with `kiwi_kinematics.py`** for
  all three wheels (60 / 180 / 300°), as do `base_radius` and `wheel_radius`.
- Every `<gazebo reference=…>` names a link that exists.
- `fdir1` carries a *literal* `gz:expressed_in` prefix. This one is checked
  against raw bytes rather than a parsed tree on purpose: sdformat parses with
  TinyXML2, which does no namespace resolution, so an XML library that
  helpfully rewrites the prefix to `ns0:` would break the friction silently.
  (My own `xacro_lite.py` did exactly that until it was fixed.)

`gazebo:=false` was confirmed to emit **zero** `<gazebo>` and `<ros2_control>`
tags, and both variants expand to 14 links / 13 joints / **9 movable**.

**ESP32 protocol** (`test_esp32_bridge.py`) — 22 cases, no ROS needed:

- Every keypress produces **byte-for-byte the packet `esp32/controller.py`
  would have sent** for the equivalent twist. The test contains a literal
  transcription of `controller.py`'s key handling to compare against, including
  its `int()` truncation and 0–360° wrap.
- Output always matches the firmware's `sscanf("%c,%d,%d,%d")` shape and fits
  its 64-byte buffer — no floats, no exponents — even for absurd input.
- Saturation clamps rather than wrapping (an overflow into reverse at full
  throttle would be a memorable bug).
- `strafe_sign` / `yaw_sign` flip only what they claim to.
- `prevent_clipping` preserves the translation:rotation ratio.

**Bridge policy** (`test_bridge_core.py`) — 45 cases, no ROS needed:

- Command timeout: a stale `/cmd_vel` yields zero, and a **backwards clock step**
  reads as stale rather than fresh. That is not paranoia — a Pi with no RTC
  steps its clock when NTP first syncs, and "elapsed time is negative" must not
  be mistaken for "the command just arrived".
- Velocity limiting scales the magnitude, so the commanded heading survives.
- Acceleration limiting ramps the linear vector; `dt <= 0` or no configured
  limit jumps straight to the target rather than stalling.
- A stale command stops **hard**, ignoring the deceleration limit. There is a
  test named for the trap, because `shaper.step(*watchdog.command(now), dt)`
  reads like a stop but actually ramps down — `resolve()` is the one supported
  way to combine the two.
- Link state: never-seen decays to down, so a wrong `esp32_ip` is reported
  rather than sitting silently in "never seen" forever.
- Endpoint resolution rejects an empty IP and an out-of-range port, and treats
  any digits-and-dots string as a literal address that is **never** handed to a
  resolver — deterministic across machines, and no DNS timeout at startup.

**Keyboard** (`test_teleop_keys.py`) — 36 cases, no ROS, no terminal, no clock:

- Each of the six keys maps to its own axis in REP-103 body frame; `s` is bound
  to nothing; uppercase behaves as lowercase; unknown keys are ignored rather
  than treated as stop; keys combine into diagonals and cancel when opposed.
- Release-by-timeout, per-key and independent, including the backwards-clock case.
- SPACE clears the held set, so a key still repeating cannot restart the robot.
- Speed steps clamp to a ceiling and to a `MIN_SPEED` floor, below which the
  robot could not overcome its own stiction and teleop would just look broken.
- The binding table is checked for coherence: no key with two jobs, every
  binding a unit vector on one axis, all six directions present.

`TeleopState` takes time as an argument, which is why a held key, a released key
and a clock that steps backwards are all just numbers in these tests.

**Firmware cross-check** (`firmware_diff.py`) — reads the `.ino` directly, so it
cannot drift from the sketch. It **reports a genuine disagreement**; see
[the finding](hardware.md#-the-firmware-and-the-simulation-disagree-about-which-way-is-left).
It also verifies the *shape* of the firmware's mixing line, not just its
constants, so changing `cos` to `sin` or dropping the minus on `rot` is caught
rather than silently invalidating every table above.

### NOT checked here — please run these

```bash
xacro urdf/mmr_bot.urdf.xacro > /tmp/robot.urdf          # must run clean
check_urdf /tmp/robot.urdf                               # 1 tree, no orphans, 9 movable
ros2 launch mmr_pkg display.launch.py                    # 0 missing-mesh warnings, 0 TF errors
colcon build --packages-select mmr_pkg                   # never run here
colcon test  --packages-select mmr_pkg                   # 119 pytest cases

gz sdf -p /tmp/robot.urdf | grep -A2 fdir1               # does gz:expressed_in survive?
ros2 launch mmr_pkg gazebo.launch.py
ros2 control list_controllers                            # 3 controllers, all `active`
ros2 topic echo /scan --once                             # frame_id must be `laser_frame`
```

Phase 3, on the real robot — **none of this was run; there is no ROS, no Pi and
no ESP32 on this machine**:

```bash
ros2 multicast send / receive          # PC <-> Pi discovery, before anything else
ros2 pkg list | grep mmr_pkg           # package is found after sourcing
ros2 pkg executables mmr_pkg           # must list esp32_bridge, kb_teleop, kiwi_drive_node
ros2 launch mmr_pkg robot.launch.py esp32_ip:=10.229.5.249
ros2 run mmr_pkg kb_teleop             # own terminal, must keep focus
ros2 topic hz /cmd_vel                 # ~25 Hz while a key is held
ros2 topic hz /scan /image_raw         # ~10 Hz and ~30 Hz
ros2 topic echo /diagnostics           # link state, reply ratio, cmd_vel age
```

Phase 4 — mapping and navigation. **Not one line of this has been run.** Both
launch files were written against the upstream parameter files and the packages'
documented interfaces; nothing here has seen a lidar:

```bash
ros2 launch mmr_pkg slam.launch.py
ros2 topic hz /odom                    # ~10 Hz, from rf2o, not from encoders
ros2 run tf2_ros tf2_echo odom base_footprint    # must resolve
ros2 topic hz /map                     # slam_toolbox is publishing a map at all

ros2 launch mmr_pkg nav2.launch.py
ros2 lifecycle get /controller_server  # every managed node must reach `active`
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}}}'
```

The specific things to watch for, because each one fails quietly rather than
loudly:

- **Does `rf2o` produce a pose that tracks reality?** Push the robot a measured
  1 m and read `/odom`. If it reports something else, that ratio is the
  calibration TODO 13 has been waiting for.
- **Does Nav2 ever command `vy`?** `ros2 topic echo /cmd_vel` while it drives
  around an obstacle. If `linear.y` is always exactly zero, one of the three
  holonomic settings did not take — [SLAM and Nav2](slam-and-nav2.md) lists
  them.
- **Does the map hold together on a loop?** Drive a closed circuit and see
  whether the start and end walls land on top of each other.
- **Does the compressed image transport actually engage?**
  `ros2 topic hz /image_raw/compressed` on the Pi. If the topic does not exist,
  the plugin is not installed and the feed is still raw.

**The `install(PROGRAMS …)` line and the executable list are the ones to check
first.** `ros2 pkg executables` is what caught the original packaging bug, and
`kb_teleop` is newly added to that install line — `check_repo.py` verifies every
file in `scripts/` appears in `CMakeLists.txt` and that each shim imports a
module that exists, but only `colcon build` proves it installs.

Then, by eye in RViz:

- all three wheels touch the ground plane;
- each `joint_state_publisher_gui` slider moves the correct link about the
  correct axis;
- the part colours came through on `base_link`.

And in Gazebo — these are the Phase 2 claims I could not test:

- **Strafing.** `{linear: {y: 0.2}}` should track cleanly sideways. If it
  stalls or veers, the friction `fdir1` did not survive conversion.
- **Direction.** `{linear: {x: 0.2}}` should drive toward the lidar. If the
  robot moves the wrong way, the sign convention or the wheel order is
  inverted — check `tools/check_phase2.py` output first.
- **The lidar returns data at all.** Silence usually means the world is missing
  the Sensors system plugin or its `<render_engine>`.

And on the real robot — the Phase 3 claims I could not test:

- **That it drives at all.** The bridge reproduces `controller.py` byte for
  byte in unit tests, but no packet has ever left this machine.
- **Which way it strafes.** Press `a`; if the robot goes right instead of left,
  set `strafe_sign: -1`. This is expected to be a coin flip until the wheel
  wiring in TODO 12 is settled.
- **That the keyboard's release timing feels right.** `key_timeout` is 0.6 s
  because a terminal's initial auto-repeat delay is typically ~0.5 s, but that
  delay is configurable per system. If the robot keeps moving too long after you
  lift a key, lower it; if it stutters while you hold one, raise it.
- **That the scan lands in the right place.** Drive forward and watch whether
  the scan of a wall in front of you sits ahead of the robot. If it is rotated,
  `laser_frame`'s yaw is wrong; if mirrored, the lidar's `inverted` parameter
  disagrees with the URDF.
- **Whether 20 Hz survives your WiFi.** The firmware's deadman is 400 ms. If
  the robot stutters, packets are being dropped, and the fix is a higher
  `publish_rate`, not a longer deadman.
- **Whether the measured link is good enough at all.** Pi → ESP32 was measured
  at ~138 ms average and ~257 ms peak, against a 400 ms deadman. That is a
  143 ms margin on the worst sample, and roughly 7.7% of packets are lost. The
  bridge sends on a timer so consecutive losses are survivable, but **this is a
  network problem and nothing in this package fixes it.** Raising the rate buys
  redundancy, not latency. If the margin is not acceptable, the answer is a
  better radio path or a wired link, not more software.

`xacro_lite.py` implements only the subset of xacro this package uses. If real
`xacro` disagrees with it, real `xacro` is right — tell me and I will fix the
description.

<h1 align="center">mmr_bot</h1>

<p align="center">
  <b>A three-wheeled omnidirectional robot you can drive from your keyboard.</b><br>
  ROS 2 Jazzy description, Gazebo simulation, and a UDP bridge to its ESP32.
</p>

<p align="center">
  <sub>
    <a href="#quick-start">Quick start</a> ·
    <a href="#driving-it">Driving it</a> ·
    <a href="#when-it-doesnt-work">Troubleshooting</a> ·
    <a href="#configuration">Configuration</a> ·
    <a href="#what-is-and-isnt-verified">What's verified</a>
  </sub>
</p>

> **📷 `docs/images/robot-hero.jpg`** — *the assembled robot, three-quarter view.
> Drop the file in and replace this line with*
> `![mmr_bot](docs/images/robot-hero.jpg)`

|  |  |
|---|---|
| **Base** | Kiwi drive — 3 omniwheels at 60° / 180° / 300°, Ø 60 mm, on a Ø 240 mm plate |
| **Arm** | SO-101, 6 movable joints, vendored from [TheRobotStudio](https://github.com/TheRobotStudio/SO-ARM100) |
| **Sensors** | RPLIDAR C1 (USB), USB camera. **No encoders** — which is why there is no odometry |
| **Compute** | Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) → WiFi → ESP32 (motor PWM) |
| **Geometry** | Measured from the Fusion 360 STEP export (`mmr_bot.step`). Nothing invented — see [TODOs](#assumptions-and-todos) |

> ### ⚠️ Read this before trusting anything below
>
> This package was built on a Windows machine **with no ROS 2 installed**.
> `xacro`, `check_urdf`, `rviz2`, `colcon`, `ros2` and **Gazebo** were never run,
> and neither the Pi nor the ESP32 was ever reachable from it — **no packet from
> this code has ever left the machine that wrote it.**
>
> What *was* run is a set of offline reimplementations, and they are listed
> honestly in [What is and isn't verified](#what-is-and-isnt-verified). Claims
> that need hardware are marked individually rather than glossed over.

---

## Gallery

|  |  |
|:--:|:--:|
| **📷 `docs/images/cad-iso.png`**<br><sub>Isometric CAD render</sub> | **📷 `docs/images/robot-real.jpg`**<br><sub>The robot as built</sub> |
| **📷 `docs/images/rviz-teleop.png`**<br><sub>RViz — model, `/scan` and camera</sub> | **📷 `docs/images/gazebo.png`**<br><sub>Gazebo Harmonic</sub> |

<sub>Drop the files into `docs/images/` and swap each cell for
`![caption](docs/images/&lt;file&gt;)`. See
<a href="docs/images/README.md"><code>docs/images/README.md</code></a> for what
each shot should show and why <code>wiring.jpg</code> is the most useful one.</sub>

---

## Quick start

```bash
cd ~/ros2_ws/src
cp -r /path/to/mmr_pkg .
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select mmr_pkg
source install/setup.bash
```

**Phase 1 — description in RViz:**

```bash
ros2 launch mmr_pkg display.launch.py
```

Arguments: `gui:=false` swaps the slider GUI for a plain `joint_state_publisher`;
`model:=` and `rviz_config:=` override the paths.

**Phase 2 — Gazebo:**

```bash
ros2 launch mmr_pkg gazebo.launch.py

# strafe sideways: the motion that only works if omniwheel friction is right
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.2}}'
```

Arguments: `headless:=true`, `rviz:=true`, `world:=`, `spawn_z:=`.

**Phase 3 — the real robot:**

```bash
ros2 launch mmr_pkg robot.launch.py esp32_ip:=10.229.5.249   # on the Pi
ros2 launch mmr_pkg teleop.launch.py                          # on the PC
ros2 run mmr_pkg kb_teleop                            # PC, own terminal, needs focus
```

Arguments: `esp32_ip:=`, `lidar:=false`, `camera:=false`, `bridge:=false`,
`serial_port:=`, `video_device:=`, `pixel_format:=`, `image_width:=`,
`image_height:=`, `camera_info_url:=`, `bridge_params:=`.

Each of the three drivers can be disabled independently, and **one failing does
not take the others down**: no node in `robot.launch.py` has an `on_exit`
handler, so an unplugged camera stops the camera and leaves the lidar and the
drive bridge running. That is the intended behaviour, and it is worth stating
because adding a single `Shutdown` handler would quietly undo it.

Before running static checks or tests, no build and no ROS is needed:

```bash
python tools/check_repo.py     # 0 failures over 70 checks
python -m pytest test/ -q      # 119 passed
```

The description also builds without any simulation tags, which is what
`display.launch.py` uses:

```bash
xacro urdf/mmr_bot.urdf.xacro gazebo:=false > /tmp/robot.urdf
```

---

## Driving it

This works with **no firmware change**. The ESP32 sketch already accepts a polar
command over UDP and does its own wheel mixing, so `mmr_pkg/esp32_protocol.py`
reproduces what `esp32/controller.py` already sends — byte for byte, which
`test/test_esp32_bridge.py` enforces.

> **📷 `docs/images/rviz-teleop.png`** — *RViz while driving: robot model, live
> `/scan`, and the camera feed.*

### The key map

```
         q   w   e          w / x   forward / back
           a   d            a / d   strafe left / right
             x              q / e   turn left / right

    SPACE  stop now         + / -   faster / slower
    k      quit             [ / ]   turn slower / faster
```

Note **`x` is reverse, not `s`** — `s` is deliberately bound to nothing.

The key map is a table at the top of `mmr_pkg/teleop_keys.py` and that is the
only place to change it. A test asserts no key has two jobs, that every binding
is a unit vector on exactly one axis, and that all six directions are present,
so a bad edit fails the suite rather than surprising you on the robot.

Defaults are 0.5 m/s and 1.0 rad/s. With the bridge's default normalisation
that is about half PWM — brisk for an indoor omni base, so start there.

**`kb_teleop` is not started by any launch file, on purpose.** It reads raw
keypresses from stdin, and `ros2 launch` gives its children no controlling
terminal, so launching it would produce a node that starts, looks healthy,
publishes a steady stream of zeros, and never registers a keystroke — which is
indistinguishable from a broken robot. It refuses to start when stdin is not a
TTY rather than pretend.

**A terminal cannot detect key release.** There is no key-up event; release is
inferred from auto-repeat stopping, so `key_timeout` (0.6 s) must exceed the
terminal's initial repeat delay (~0.5 s). The cost is a bounded lag between
lifting a key and the robot stopping. SPACE is immediate and clears the held-key
set — not just the output, so a key still physically down cannot restart the
robot on its next repeat — and the firmware's 400 ms deadman is underneath
everything regardless.

`teleop_twist_keyboard` still works as a second opinion on the same `/cmd_vel`,
which is useful for deciding whether a problem is the keyboard node or the
bridge. Hold **shift** with that one: its unshifted `j`/`l` turn, and only the
shifted `J`/`L` strafe.

---

## When it doesn't work

Ordered by how often each one is the answer. Every check here is a command you
can run, not a thing to eyeball.

### The robot doesn't move

| What you see | Most likely cause | What to do |
|---|---|---|
| No movement, **no error anywhere** | Wrong `esp32_ip`. UDP has no connection, so packets to a wrong host vanish in complete silence — this is the single most confusing failure this system has | `ros2 topic echo /diagnostics` — look for `state: down` with `replies_received: 0`. Then check the address the ESP32 prints on its serial console at boot |
| Moves, then stops after ~0.4 s, repeatedly | The firmware's 400 ms deadman is firing because packets stopped arriving | `ros2 topic hz /cmd_vel` should show ~25 Hz while a key is held. If it does, the loss is on the network |
| Stutters or lurches while driving | Packet loss. The measured link drops ~7.7% | Raise `publish_rate` for redundancy — but see [Networking](#networking), because **this is a network problem and software cannot fix it** |
| Nothing happens when you press keys | The terminal lost focus | Click the `kb_teleop` window. It must keep focus |
| `kb_teleop` exits immediately saying stdin is not a TTY | You launched it from a launch file or through a pipe | Run it directly in its own terminal. It refuses rather than silently publishing zeros forever |

### It moves, but wrongly

| What you see | Cause | Fix |
|---|---|---|
| Strafes the **wrong way** (A goes right) | Motor wiring handedness. Expected to be a coin flip until [the wiring is confirmed](#-the-firmware-and-the-simulation-disagree-about-which-way-is-left) | Set `strafe_sign: -1` in `config/esp32_bridge.yaml`. **Do not** edit the angle arithmetic |
| Curves away from the commanded heading when translating *and* spinning at speed | The firmware clamps each wheel to ±255 **after** subtracting rotation, so the mix saturates | `prevent_clipping: true` |
| Drives smoothly but not in the commanded direction; yaw bleeds into translation | Classic symptom of a 30° wheel-angle error | [Read this](#the-30-wheel-angle-discrepancy--read-this-before-driving) |
| Keeps coasting after you release a key | `key_timeout` too long for your terminal's auto-repeat | Lower it. A terminal has no key-up event, so release is *inferred* — see [TODO 16](#assumptions-and-todos) |

### Nothing shows up in RViz

| What you see | Cause | Fix |
|---|---|---|
| `Frame [odom] does not exist` | There is no odometry, so nothing publishes `odom → base_link`. This is correct for a robot with no encoders | Set the fixed frame to `base_footprint` |
| `/scan` publishes but is invisible | `frame_id` is not in the TF tree | `ros2 topic echo /scan --once` — `frame_id` must be `laser_frame`. `robot.launch.py` passes this to the driver already |
| Robot model missing, TF tree stops at `base_link` | `joint_state_publisher` isn't running, so no joint has a position | It is started by `robot.launch.py` on purpose — see [Frames](#frames-and-what-you-will-and-will-not-see-in-rviz) |
| **Camera** display: `invalid position calculation (nans or infs)` | No intrinsic calibration, so `CameraInfo` is all zeros and RViz computes `0/0` | Use the **Image** display, which needs no TF and no calibration. This is the correct tool here, not a workaround |
| PC sees no topics from the Pi at all | Discovery, not the robot | `ros2 multicast send` / `receive`. Check for a leftover `ROS_LOCALHOST_ONLY=1` **first** — see [Networking](#networking) |

### It won't build or won't start

| What you see | Cause | Fix |
|---|---|---|
| `rosdep install` aborts before resolving anything | `sllidar_ros2` named as a dependency. It's a workspace checkout, not a rosdep key | Don't add it to `package.xml`. `check_repo.py` fails if it comes back |
| `failed to create symbolic link … existing path cannot be removed: Is a directory` | A `build/` directory in the source tree collides with colcon's own | This is why the generated URDF lives in `generated/` |
| `ros2 run mmr_pkg <thing>` can't find it | The script isn't in `install(PROGRAMS …)` | `python tools/check_repo.py` |
| A node dies instantly with `AttributeError` | A `self.x` read but never assigned — this exact bug once shipped a node that had *never* been able to start | `python tools/check_repo.py` |
| Camera fails and takes nothing else down | Working as intended | No node has an `on_exit` handler, so one failure never cascades |

**When in doubt, run the two commands that need no ROS, no hardware and no build:**

```bash
python tools/check_repo.py     # static sweep, under a second
python -m pytest test/ -q      # 119 cases
```

---

---

## Frames, and what you will and will not see in RViz

**The lidar `frame_id` is overridden in the launch file.** `sllidar_ros2`
defaults to `laser`, this URDF calls the link `laser_frame`, and a `/scan`
stamped with a frame that is not in the TF tree is invisible in RViz with only a
warning to show for it. `robot.launch.py` passes `frame_id:=laser_frame` rather
than papering over the mismatch with a `static_transform_publisher`.

**`joint_state_publisher` runs on the robot even though there is no hardware.**
`robot_state_publisher` cannot publish TF for a joint unless something reports
its position, and nothing does — no encoders, and the arm is not driven. Without
it the TF tree stops at `base_link`, `laser_frame` never appears, and the scan
has nowhere to go. Publishing zeros makes the tree complete and the geometry
correct. The wheels will never *appear* to turn, which is honest: the robot
genuinely does not know whether they are.

**Fixed frame is `base_footprint`, not `odom`.** There is no odometry, so
nothing can publish `odom → base_footprint`. The robot will sit still in the
middle of the view while the world sweeps around it as you drive. That is
correct for a robot with no encoders, not a bug. Setting the fixed frame to
`odom` just yields *"Frame [odom] does not exist"*. This is also why SLAM is not
on the table yet.

## The path to real odometry, and why none of it is faked

There is no `/odom`, no `odom → base_link` transform, and no wheel motion in
`/joint_states` beyond the zeros that keep the TF tree connected. All three are
absent deliberately: this robot has no encoders, so any of them would be a
number invented by software and presented as a measurement. Fake odometry is
worse than none, because everything downstream — SLAM, Nav2, any `map` frame —
would silently trust it.

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

Only then does the RViz fixed frame become `odom`, and only then is SLAM on the
table. Until step 1 exists, steps 2 and 3 have nothing truthful to publish.

**The camera needs no URDF link to be visible.** RViz's *Image* display carries
no TF filter and renders whatever arrives. The *Camera* display is the one that
needs both TF and `CameraInfo` — and it additionally needs a real intrinsic
calibration, which this camera does not have: `v4l2_camera` publishes an
all-zero `CameraInfo` when calibration is missing, and RViz then computes
`p[3]/p[0]` = `0/0`, rejecting the frame as *"invalid position calculation
(nans or infs)"*. So the Image display is not a workaround, it is the correct
tool here. See TODO 14 if you want a real `camera_link`.

---

## Networking

Both machines need the same `ROS_DOMAIN_ID`, the same subnet, and working
multicast.

```bash
ros2 multicast send        # on one machine
ros2 multicast receive     # on the other
```

Two Jazzy-specific traps:

- **`ROS_LOCALHOST_ONLY` is deprecated but still honoured, and it overrides
  `ROS_AUTOMATIC_DISCOVERY_RANGE`.** A leftover `export ROS_LOCALHOST_ONLY=1`
  in `.bashrc` from a Humble-era tutorial breaks multi-machine discovery no
  matter what else you set. Check this first.
- An **invalid** `ROS_AUTOMATIC_DISCOVERY_RANGE` value fails *closed*, silently
  falling back to `LOCALHOST`. Valid values are `SUBNET` (default),
  `LOCALHOST`, `OFF`, `SYSTEM_DEFAULT`.

If your access point has client isolation enabled — common on guest and campus
WiFi — multicast will not cross it. Use `ROS_STATIC_PEERS` (semicolon
separated) for unicast discovery instead.

The ESP32 joins SSID `test` and is on `10.229.5.249`; the Pi is on
`10.229.5.237`. Same `/24`, so the UDP path is direct. Both are DHCP leases —
pin them in the router or expect them to move.

---

## Configuration

### Bridge parameters

`esp32_bridge` converts `/cmd_vel` to the firmware's `"A,angle,speed,rot"`
datagrams at 20 Hz. The rate is not cosmetic: the sketch brakes all three
motors if it hears nothing for `DEADMAN_MS` (400 ms), so the bridge transmits
on a timer rather than on message arrival, and keeps transmitting `A,0,0,0`
while parked so the link indication stays live. Two independent watchdogs
guard it — the bridge stops sending motion if `/cmd_vel` goes quiet for
`cmd_timeout`, and the firmware brakes regardless if packets stop, which is the
one that survives the bridge being killed or the PC going to sleep.

Tuning lives in **`config/esp32_bridge.yaml`**, which `robot.launch.py` loads,
so there is one place to change a number. `esp32_ip` is deliberately *not* in
it — it is per-robot and follows the DHCP lease, so it belongs on the command
line where a stale value cannot be committed:

```bash
ros2 launch mmr_pkg robot.launch.py esp32_ip:=10.229.5.249
```

| Parameter | Default | Meaning |
|---|---|---|
| `esp32_ip` | *required* | No default; a wrong IP fails silently over UDP |
| `esp32_port` | `1234` | Must match `PORT` in the sketch |
| `max_linear_speed` | `1.0` | The `cmd_vel` that maps to `max_linear_pwm` |
| `max_angular_speed` | `1.0` | The `cmd_vel` that maps to `max_angular_pwm` |
| `max_linear_pwm` | `180` | `controller.py`'s cruise value |
| `max_angular_pwm` | `80` | `controller.py`'s Q/E value |
| `max_linear_accel` | `0.0` | m/s². **0 = no limit**; see below |
| `max_angular_accel` | `0.0` | rad/s². 0 = no limit |
| `max_linear_decel` | `0.0` | m/s². 0 = brake as hard as commanded |
| `max_angular_decel` | `0.0` | rad/s². 0 = brake as hard as commanded |
| `publish_rate` | `20.0` | Hz. Must stay well above the deadman's 2.5 Hz |
| `cmd_timeout` | `0.5` | s of `/cmd_vel` silence before commanding zero |
| `reply_timeout` | `1.0` | s of ACK silence before the link is called down |
| `diagnostic_period` | `1.0` | s between `/diagnostics` publications |
| `strafe_sign` | `1` | Flip to `-1` if A/D come out swapped |
| `yaw_sign` | `-1` | `-1` reproduces `controller.py` exactly |
| `prevent_clipping` | `false` | See below |
| `publish_replies` | `false` | Republish every ACK on `~/esp32_reply` while debugging |

**The acceleration limits default to off, and that is a deliberate blank, not an
oversight.** This robot has no encoders and there is no measured acceleration
figure for it, so any non-zero default would be invented. Raise them if the base
lurches on a step input. Note the *braking* limits default to unlimited on
purpose: a slow ramp up is comfortable, but a slow ramp down is a robot that
will not stop when you tell it to. Limiting is applied to the linear **vector**,
not per-axis, so ramping never bends the direction of travel.

Velocity limiting has the same property. Clamping `(2.0, 1.0)` per-axis against a
ceiling of 1.0 gives `(1.0, 1.0)` — a 45° error in a robot that was told to go
mostly forward. The magnitude is scaled instead, so the heading survives and only
the speed is reduced. The same rule appears in three places (`prevent_clipping`,
the velocity limiter and the smoother) because it is the same mistake three
times over.

---

### Is it actually working? — `/diagnostics`

Rather than logging continuously, the bridge publishes two `DiagnosticStatus`
entries at 1 Hz, so the questions worth asking have topic-shaped answers:

```bash
ros2 topic echo /diagnostics                # bridge link + command state
ros2 topic hz /cmd_vel                      # is the keyboard publishing?
ros2 topic hz /scan                         # is the lidar publishing?
ros2 topic hz /image_raw                    # is the camera publishing?
```

`esp32_bridge: link` carries the link state, `last_reply_age_s`, an RTT
*estimate*, packets sent, replies received, the reply ratio and the send-error
count. `esp32_bridge: command` carries `cmd_vel_age_s`, the timeout, the three
velocities actually sent and the last packet on the wire.

Two honesty notes on those numbers. **`rtt_estimate_s` is an estimate and can
never be better than one**, because the protocol has no sequence number — replies
are paired with sends in FIFO order, and one lost packet mis-pairs everything
after it until the queue drains. `last_reply_age_s` is exact, and it is the
number that relates to the 400 ms deadman, so prefer it. A real RTT needs a
sequence number in the firmware, which is out of scope here.

Link transitions are reported once, on change, rather than repeated every tick —
including the case where the ESP32 has *never* replied, which is what a wrong
`esp32_ip` looks like. UDP has no connection, so packets to a wrong host vanish
in silence; without that distinction the most confusing failure this system has
would still be a silent one.

`prevent_clipping` addresses a real defect in the firmware: it computes
`power_i = speed*cos(θ−α_i) − rot` and then clamps **each wheel** to ±255. So a
fast translation combined with a fast spin saturates, and the robot curves away
from the commanded heading instead of merely going slower. Enabling this scales
both terms by one factor, preserving the ratio and therefore the path. It
defaults to `false` so behaviour matches `controller.py` out of the box.

---

### Calibrating the ESP32 bridge

**`cmd_vel` on this robot is not yet in metres per second, and this package
does not pretend otherwise.** The firmware has no encoders, no wheel radius and
no closed loop; `speed` and `rot` are raw 8-bit PWM, roughly proportional to
motor *voltage*, which varies with battery charge, payload and floor friction.

So the mapping is a declared normalisation, not a measurement. The PWM ends
(180, 80) are `controller.py`'s own known-good values. The SI ends are `1.0`
placeholders. Until you measure it, treat `cmd_vel` as *fraction of full scale*.

To make it real:

1. Clear ~4 m of floor, mark a start and end line.
2. `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'`
   and time the run with a stopwatch.
3. `max_linear_speed = distance / (time × 0.5)`.
4. Repeat with `angular: {z: 0.5}` and a full 360° spin for
   `max_angular_speed`.

Until then, **there is no odometry**, and the RViz fixed frame is
`base_footprint` rather than `odom` for exactly that reason — see below.

---

## The robot

> **📷 `docs/images/cad-top.png`** — *top-down render showing the three wheels.
> Worth adding: the wheel angles are the thing people get wrong.*

### Frame tree

```
base_footprint                          ground plane, under the wheel-triad centre
 └── base_link                          [base_footprint_joint, fixed]   z = +0.045
     ├── wheel_0_link                   [wheel_0_joint, continuous]      60°
     ├── wheel_1_link                   [wheel_1_joint, continuous]     180°
     ├── wheel_2_link                   [wheel_2_joint, continuous]     300°
     ├── laser_frame                    [laser_joint, fixed]            RPLIDAR C1
     └── arm_base_link                  [arm_base_joint, fixed]
         └── arm_shoulder_link          [shoulder_pan,   revolute]
             └── arm_upper_arm_link     [shoulder_lift,  revolute]
                 └── arm_lower_arm_link [elbow_flex,     revolute]
                     └── arm_wrist_link [wrist_flex,     revolute]
                         └── arm_gripper_link            [wrist_roll, revolute]
                             ├── arm_gripper_frame_link  [gripper_frame_joint, fixed]
                             └── arm_moving_jaw_so101_v1_link  [gripper, revolute]
```

**14 links, 13 joints, of which 9 are movable** — 3 wheels + 6 arm, exactly as
the brief requires. The chassis contributes no movable joint other than the
three wheels.

### Derived geometry

Everything below was measured from the STEP. Nothing is invented. The "source"
column names the script that produced it; all live in `tools/`.

| Quantity | Value | Source |
|---|---|---|
| STEP units | `SI_UNIT(.MILLI.,.METRE.)`, Z-up | `step_parse.py` |
| Assembly | real multi-part: 339 products, 556 occurrences, 1 root, 19 top-level components | `step_parse.py` |
| Wheel radius | **0.030 m** | `measure_detail.py` — max radius 30.000 mm on all three |
| Wheel hub width | **0.019 m** | two `Dasar` discs of 9.500 mm |
| Wheel roller envelope | 0.027287 m wide, roller r = 0.028934 m | rollers sit ~1 mm inside the hub radius, so the hub is the contact surface as drawn |
| Base radius (centre→wheel) | **0.135500 m**, all three equal to <0.001 mm | `frames.py` |
| Base plate | round, Ø **0.240002 m**, 0.012 m thick, spans z = 0.000…0.012 | `extract_meshes.py` — outer radius present in all 36 angular bins at 120.000 ± 0.001 mm |
| Top plate | Ø 0.240 m disc + forward tab to r = 0.140202 m, 0.005 m thick, z = 0.072…0.077 | ditto |
| `base_link` height | **0.045 m** above ground | underside of base plate |
| Wheel axle z | −0.015 m relative to `base_link` | measured; also *derived* in the xacro, see below |
| Wheel mounting angles | **60°, 180°, 300°** | measured bearings 60.00 / 180.00 / 300.01 |
| Arm mount | xyz (0.014091, −0.000442, 0.079200) m, rpy (0, 0, 0) | `arm_mount.py` |
| Lidar mount | xyz (0.133781, 0.000004, 0.105180) m, rpy (0, 0, 0) | `measure_arm.py`; **z is approximate, see TODOs** |
| Battery | **not modelled in the CAD** | — |

#### Frame convention

The CAD origin is *not* the wheel-triad centre. The three wheel centres'
centroid is at CAD (0.196, 0.001) mm, and `base_link` is placed there so the
kiwi kinematics are exactly symmetric. The full transform is

```
p_link = Rz(+90°) · ( p_cad − (0.196, 0.001, 44.941) mm )
```

i.e. **robot +X (forward) = CAD −Y**, the direction the lidar, camera and arm
all face. You confirmed this choice; see [Corrections](#corrections-to-the-brief).

#### The axle-height consistency check

The brief asked for this explicitly. Wheel centres measure CAD z = 29.941 mm and
the wheel circles bottom out at CAD z = −0.059 mm — a difference of exactly
30.000 mm, the measured wheel radius. So the CAD is self-consistent and simply
sits 0.059 mm below z = 0. Taking ground as the wheel-circle tangent makes every
height land on a round number (axle 30.000, base plate underside 45.000, top
plate top 122.000), which is strong evidence it is the intended datum.

In `base.xacro` the axle height is **derived, never typed**:

```xml
<xacro:property name="wheel_z" value="${wheel_radius - base_link_height}"/>
```

so wheel radius and axle height cannot drift apart. `tools/check_urdf_lite.py`
confirms all three wheels bottom out at z = 0.000000000.

---

### Wheel sign convention

*Required by the brief. Verified numerically against the generated URDF by
`tools/kiwi_check.py`, not asserted from memory.*

Each `wheel_i_link` frame has its **local +X pointing radially outward** along
the mounting angle, and that is the spin axis (`<axis xyz="1 0 0"/>`). Rolling
is therefore tangential, at mounting angle − 90°.

With spin axis `u = (cos α, sin α, 0)` and the contact patch at `−R ẑ`, a
positive joint velocity `ω` drives the chassis along

```
d = (sin α, −cos α, 0)        i.e. bearing α − 90°
```

**A positive velocity on one wheel alone pushes the robot toward
`mounting angle − 90°`, and equal positive velocity on all three rotates the
robot CLOCKWISE seen from above (negative yaw).**

Specifically, all three at +1 rad/s gives `wz = −0.2214 rad/s` and zero
translation (verified to 1e-12).

For Phase 2 reference, the inverse kinematics implied by this convention is

```
ω_i = ( vx·sin α_i − vy·cos α_i − L·ωz ) / R        L = 0.135500, R = 0.030
```

FK/IK round-trips to 2e-15 over 2000 random twists. Sanity check: driving
straight forward gives `ω = [28.87, 0, −28.87]` — wheel 1 sits at 180° with its
spin axis along X, so it free-rolls sideways and correctly does not turn.

⚠️ **Diff this against your ESP32 firmware before driving anything.** If the
firmware disagrees on sign or wheel order, change the firmware or the
`wheel_*_angle_deg` properties — not both.

---

## Things that will bite you

Three findings that change how the robot behaves, kept together because each one
produces motion that *looks* fine while being wrong.

### ⚠ The firmware and the simulation disagree about which way is left

Run it yourself:

```bash
python tools/firmware_diff.py
```

Section 9 asked for the IK to be isolated so it could be diffed against the
firmware. Now that the firmware exists, that diff is automated, and it does not
come out clean:

| key | firmware `power[]` | `kiwi_kinematics` `w[]` | |
|---|---|---|---|
| W forward | `[1.0, 0.0, -1.0]` | `[1.0, 0.0, -1.0]` | agree |
| S back | `[-1.0, 0.0, 1.0]` | `[-1.0, 0.0, 1.0]` | agree |
| D right | `[-0.5, 1.0, -0.5]` | `[0.5, -1.0, 0.5]` | **negated** |
| A left | `[0.5, -1.0, 0.5]` | `[-0.5, 1.0, -0.5]` | **negated** |

They agree exactly on the forward axis and are exactly negated on the lateral
one. That is a **reflection**, and no relabelling or rotation of the robot frame
can undo a reflection — so this is a genuine handedness difference, not
bookkeeping.

A caution about reading the two `{60, 180, 300}` arrays and concluding they
match: they are *different quantities*. The URDF number is where the wheel is
**bolted**; `wheelDeg[]` in the sketch is the direction that wheel is assumed to
**push**. For a wheel whose axle points radially outward those differ by 90°, so
the arrays being identical is a warning sign rather than a reassurance.

Of the 48 possible wirings (3! index orders × 2³ motor polarities), **exactly
one** reconciles the firmware with the measured URDF:

```
ESP32 index 0 (dir 19, pwm 23)  ->  wheel_2_link, motor leads reversed
ESP32 index 1 (dir 25, pwm 26)  ->  wheel_1_link, motor leads reversed
ESP32 index 2 (dir 14, pwm 12)  ->  wheel_0_link, motor leads reversed
```

**This is a hypothesis, not a measurement.** Which wheel is on which pin group,
and which way each motor's leads are soldered, are not in the source code and
cannot be derived from it. Confirm on the robot: put it on blocks, drive one
wheel at a time, and note which wheel turns and which way. A positive command
on `wheel_N` should push the chassis toward *(that wheel's mounting angle −
90°)*.

Rule one thing out first: `// (180 - a) fixes left/right` is an *operator
judgement*. If it was made standing **in front of** the robot facing it, left
and right are mirrored and the firmware is the thing that is wrong.

None of this blocks teleop — the bridge reproduces `controller.py` verbatim, so
the robot drives today exactly as it already does. It matters the moment you
compare simulation against hardware, or move to Option B.

> **📷 `docs/images/wiring.jpg`** — *the ESP32 and motor wiring, close enough to
> trace which driver output feeds which wheel. This is the missing evidence that
> would settle the finding above; it is not in any source file.*

---

### The 30° wheel-angle discrepancy — read this before driving

This is the one correction that can silently ruin the drive, because a 30°
error in the wheel angles produces motion that still *looks* holonomic: the
robot drives smoothly, just not in the commanded direction, and yaw bleeds into
translation.

The three omniwheel instances measure, straight out of the GLB:

| Instance | CAD bearing | radius | robot-frame bearing |
|---|---|---|---|
| `RodaOmni 60 mm v1:1` | 330.004° | 135.501 mm | **60.004°** |
| `RodaOmni 60 mm v1:3` | 90.004° | 135.499 mm | **180.004°** |
| `RodaOmni 60 mm v1:2` | 210.005° | 135.503 mm | **300.005°** |

Uniformly 30° away from the brief's 30/150/270, so it is a reference-frame
disagreement, not a modelling slip. Forward is pinned independently: the
`RPLIDAR C1 v1:1` instance sits at CAD bearing **270.000°**, i.e. exactly along
CAD −Y, and the arm and camera face the same way. Taking that as robot +X, the
wheels land at 60/180/300 — one wheel dead astern, two ahead at ±60°, the usual
kiwi layout. The brief's angles would instead put a wheel dead abeam to
starboard, which no instance in the STEP does.

The shared +0.004° is the residual in the origin estimate, not a real asymmetry.

**If your ESP32 firmware assumes 30/150/270**, then it and this description
disagree by 30° and one of them is wrong about the physical robot. Change
whichever is wrong — the `wheel_*_angle_deg` properties in `base.xacro`, or the
firmware — but **not both**, and re-run `tools/kiwi_check.py` afterwards. The
drive node takes the angles as a ROS parameter precisely so this can be
corrected without editing code.

Also worth recording: **the STEP carries no joint or mate data** — all 556
placements are rigid. So the brief's concern about extraction inventing a
spurious revolute joint is structurally impossible here; every joint in this
package was authored deliberately.

---

### Corrections to the brief

The brief asked to be told when a supplied number disagreed with the STEP.
Four did.

| Brief said | STEP says | Resolution |
|---|---|---|
| wheel mounting angles **30° / 150° / 270°** | **60.004° / 180.004° / 300.005°** — a uniform **30° offset** | the STEP's angles are used. See below; this one matters most |
| omniwheel "60 mm — confirm RADIUS or DIAMETER" | max radius **30.000 mm** on all three; part is named `RodaOmni 60 mm` | 60 mm is a **diameter**; radius 0.030 m |
| base plate "200 mm — round or square?" | **round**, Ø **240.002 mm** (roundness ratio 1.0001; a square would give 1.414) | plate radius 0.120 m — the 200 mm figure is wrong |
| wheel axle height 27 mm | axle sits **30.000 mm** above the wheel-circle tangent, exactly the wheel radius | 27 mm is wrong; 30 mm is used, and the xacro derives it so it cannot disagree with the radius |

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
    here. See *Omniwheel friction* under
    [Under the hood → Gazebo and `ros2_control`](#under-the-hood) for the
    one-line `gz sdf -p` check and the exact fallback.
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
    [the finding](#-the-firmware-and-the-simulation-disagree-about-which-way-is-left).
13. **`cmd_vel` is not yet in m/s.** The bridge's speed map is a declared
    normalisation, not a measurement — the firmware has no encoders and no
    closed loop. The procedure to make it real is in
    [Calibrating the ESP32 bridge](#calibrating-the-esp32-bridge). Until then
    there is no odometry, and therefore no SLAM.
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
python tools/check_repo.py                        # 0 failures over 70 checks
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
[the finding](#-the-firmware-and-the-simulation-disagree-about-which-way-is-left).
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

---

## Under the hood

Design reasoning and provenance for every number above. Collapsed because you do
not need any of it to drive the robot — but it is the part that explains *why*
things are the way they are, and each one is here because something went wrong
without it.

<details>
<summary><b>How the pieces fit together</b> — nodes, topics, and the ROS-free split</summary>

### What talks to what (real robot)

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

### Why the logic lives outside the nodes

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

</details>

<details>
<summary><b>Kinematics</b> — the IK, and diffing it against your firmware</summary>

### The IK, and how to diff it against your firmware

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
algebra. See [Wheel sign convention](#wheel-sign-convention) — and note the
[30° wheel-angle discrepancy](#the-30-wheel-angle-discrepancy--read-this-before-driving)
between the brief and the STEP, which is the single most likely source of a
mismatch.

`test/test_kiwi_kinematics.py` has 16 tests (round-trip to 1e-12, closed form vs
pseudo-inverse, per-wheel direction, and a test that re-reads the expanded URDF
so the module and `base.xacro` cannot drift apart). They need no ROS:

```bash
python -m pytest test/ -q      # 16 passed
```

### Option B — moving the kinematics into ROS

The better long-term design, once the wiring above is settled: change the
sketch to accept three wheel PWMs instead of a polar command, and let
`kiwi_kinematics.py` do the mixing. `kiwi_kinematics` then becomes the single
source of truth for both Gazebo and hardware, the geometry (`R`, `L`) enters the
maths properly, and `rot` stops being an arbitrary knob whose ratio to `speed`
has no physical meaning. It costs one reflash. Not done here because it was not
needed to answer the question, and because it is unsafe to do before the wiring
is confirmed.

</details>

<details>
<summary><b>Meshes</b> — decimation policy and one honest trade-off</summary>

### Meshes

One mesh per link, all in metres, all referenced by `package://mmr_pkg/…`.
Regenerate with `python tools/extract_meshes.py`.

#### Visual

| File | Triangles | Notes |
|---|---|---|
| `meshes/visual/base_link.obj` | 44,265 | + `.mtl` with **27 CAD part colours** |
| `meshes/visual/wheel.obj` | 13,872 | + `.mtl`; one file serves all three wheels |
| `meshes/visual/rplidar_c1.obj` | 12,000 | + `.mtl` |

OBJ was chosen over STL so the `.mtl` carries the part colours, as the brief
asked. The URDF deliberately sets **no** `<material>` on these visuals — a URDF
material would override the mesh colours in RViz.

The wheel mesh is exported into the canonical wheel-link frame (local +X = spin
axis), which is why one file serves all three. Verified after export: radius
0.03001 m about local +X, half-width 0.01365 m — a direct check that the
CAD→link transform is correct.

#### Decimation policy, and one honest trade-off

Two bugs were found and fixed while producing these, both worth knowing about:

1. **Unmerged vertices silently destroy decimation.** `trimesh.load(process=False)`
   leaves STL/GLB vertices split (STL is a pure triangle soup; cascadio splits
   at every B-rep patch boundary, ~39% duplicates). Quadric simplification then
   cannot collapse edges and just *deletes triangles* — an early run hit the
   triangle budget while shrinking the model by up to 18 mm. `merge_vertices()`
   before simplifying cut the median error 45×.

2. **A uniform decimation ratio spends the budget on things nobody can see.**
   The CAD carries 18,316 triangles on a 4.4 mm spring and 56,116 on the Pi
   cooler's fin stack. Triangles are now allocated in proportion to each part's
   **surface area**.

Quadric decimation also has a hard topological floor — thin fins, coil springs
and screw threads are high-genus and cannot be collapsed without changing
topology. Parts that still overshoot their target by 3× are replaced by their
convex hull; every substitution is printed when you run the script.

> **The trade-off:** the Pi cooler's **fin stack** (solidity 0.30) and **fan
> rotor** (0.17) become solid blocks. That is where hulling visibly loses shape.
> It buys the 50k budget: keeping them costs ~101k triangles for `base_link`
> instead of ~44k. Set `HULL_OVERSHOOT` in `tools/extract_meshes.py` to a large
> number to keep full fin detail. Everything else hulled is a connector, screw,
> washer or motor body that is convex anyway.

#### Collision

Never a full-detail visual mesh — verified by `check_urdf_lite.py`.

- **Wheels and base plate are cylinder primitives**, as required. Wheel:
  r = 0.030, l = 0.019, rotated onto +X. Base plate: r = 0.120, l = 0.012.
- Top plate: cylinder r = 0.120, l = 0.005. The forward lidar tab is *not*
  covered, but the lidar's own hull sits over that region.
- `motor_pod.stl` — convex hull of motor + mount, extracted in a canonical 0°
  frame and instantiated three times at the wheel angles. Verified that
  `BaseMotor v1:3` really is the 180° instance before relying on it.
- `camera_pod.stl`, `pi_stack.stl` — convex hulls.
- `meshes/collision/arm/*.stl` — see below.

</details>

<details>
<summary><b>The SO-101 arm</b> — vendoring, collision decomposition, prefixes</summary>

### SO-101 arm

Vendored, **not** regenerated from the STEP. Source:
`Simulation/SO101/so101_new_calib.urdf` plus its 13 STLs, copied to
`meshes/arm/`. **Which calibration variant matches your physical robot is still
an open question (TODO 7)** — `new_calib` was used as the default because it is
the current one upstream, not because it has been confirmed. The STLs are in
metres, confirmed against the CAD part
(`base_so101_v2.stl` measures 110.92 mm, byte-identical to the copy used to
solve the mount), so no `scale` attribute is needed.

`urdf/arm.xacro` is **generated** by `tools/gen_arm_xacro.py` as a *text-level*
transform of the upstream URDF, so every origin, limit and inertia string is
carried across byte-for-byte — no number can be mistyped, and the result stays
diffable against a future upstream release. The generator then re-parses its own
output and asserts that link set, joint names, joint types, limits and origins
all still match upstream.

What changed:

| Change | Why |
|---|---|
| Link names → `${prefix}…`, default `arm_` | upstream's root is also called `base_link`, which would collide with the chassis root |
| **Joint names unchanged** | a LeRobot policy is trained against this exact action space |
| `<limit>` values unchanged | as instructed |
| Mesh paths → `package://mmr_pkg/meshes/arm/…` | no relative `assets/` paths |
| Collision → `meshes/collision/arm/…` | upstream aims collision at the full-detail visual mesh |
| `<transmission>` blocks dropped (6) | ROS 1 `PositionJointInterface` tags; Jazzy uses `<ros2_control>`, added in Phase 2 |
| Stray `<origin>` inside `gripper_frame_link` dropped | not valid URDF, silently ignored anyway |

#### Arm collision meshes

Each part is **convex-decomposed** with CoACD (`tools/arm_collision.py`), not
reduced to a single hull: the moving jaw, the wrist fork and the motor holders
are deeply concave, and one hull per part fills the gripper opening solid — not
"honestly close enough" for a gripper. All 13 parts, 8 hulls each; the result is
**105,524 collision triangles against 322,564 visual (32.7%)**.

Fidelity, as reported by the tool (4000 points sampled on the visual surface,
distance to the nearest point of the collision surface, in mm):

| | p50 | p90 | max |
|---|---|---|---|
| best (`sts3215_03a_*`, `waveshare_mounting_plate`) | 0.000–0.002 | 0.003–1.35 | 1.9–3.9 |
| typical | 0.12–0.25 | 1.8–3.2 | 3.5–7.1 |
| worst (`base_so101_v2`) | 1.03 | 5.11 | **9.38** |

Two honest caveats on that table. **It is one-sided:** it measures visual →
collision, so a hull that bulges into empty space away from any visual surface
is under-counted. And **`base_so101_v2` at 9.4 mm max is the one to look at
first** — it is the flattest, most shell-like part, so 8 hulls fit it worst.
Raise `MAX_HULLS` for that part if the arm turns out to collide with the
chassis in Phase 2. Since the union of hulls contains the original, all error is
material *added*, never removed — the safe direction for collision, and the
reason the gripper's approach clearance is the thing to re-check in sim.

That the decomposition actually *preserved* concavity — rather than quietly
collapsing to a blob — was checked by comparing each result's volume against
that part's single convex hull:

| Part | CoACD ÷ single hull |
|---|---|
| `base_motor_holder` | 0.35 |
| `motor_holder_wrist` | 0.39 |
| `wrist_roll_pitch` | 0.48 |
| **`moving_jaw`** | **0.53** |
| `upper_arm` | 0.69 |
| `sts3215_03a*` (servo bodies) | ≈ 1.00 |

A ratio of 1.0 means no concavity was kept. The moving jaw at 0.53 is the one
that matters: one hull would have nearly doubled its volume, and all of that
extra is the jaw opening filling in solid. The two servo bodies sitting at ≈1.0
is correct, not a failure — they genuinely are convex boxes. (These ratios are
*upper* bounds: the concatenated pieces overlap, and overlapping volume is
double-counted, so the true figures are lower still.) Bounding boxes grew by at
most 0.61 mm, on `motor_holder_so101_base_v1`.

The pieces for one part are concatenated into a single STL so the URDF keeps
upstream's one-`<collision>`-per-part structure. **For Phase 2**, if the solver
needs guaranteed convexity, split them into one `<collision>` element per piece.

#### Two corrections to the brief's §5

- **§5.3 is a no-op.** The premise ("missing base collision") is false: the
  current `so101_new_calib.urdf` `base_link` has 4 visuals *and* 4 collisions.
  Nothing was substituted; adding a redundant primitive would have been wrong.
  The real collision problem was different — upstream points collision at the
  visual meshes — and is fixed above.
- **`so101_old_calib.urdf` would not have been a drop-in alternative:** it
  renames every link and has no `gripper_frame_link`.

#### Prefix caveat

Per instructions, links are prefixed but joints are not. That is right for a
single arm, but it means **instantiating the macro twice would collide on joint
names**. If you ever add a second arm, the joint names have to be prefixed too —
and at that point the LeRobot action-space mapping needs revisiting anyway.

</details>

<details>
<summary><b>Gazebo and ros2_control</b> — simulation wiring and friction</summary>

#### Simulation and control

##### What talks to what (simulation)

```
/cmd_vel ──▶ kiwi_drive_node ──▶ /wheel_velocity_controller/commands
                                          │  Float64MultiArray, 3 entries
                                          ▼
                        ros2_control  (gz_ros2_control/GazeboSimSystem)
                                          │
                                          ▼
                     wheel_0/1/2_joint velocity interfaces ──▶ physics

joint_state_broadcaster ──▶ /joint_states ──▶ robot_state_publisher ──▶ /tf
gpu_lidar ──▶ gz topic /scan ──▶ ros_gz_bridge ──▶ /scan (sensor_msgs/LaserScan)
```

#### Omniwheel friction — the part most likely to need tuning

A cylinder collision grips in every direction, which would weld a kiwi drive to
the floor. Real omniwheels slide freely along their own axle, so `gazebo.xacro`
sets anisotropic friction: `mu` along `fdir1` (the axle, slips), `mu2`
perpendicular (rolling, grips).

`fdir1` is `(1,0,0)` in `wheel_N_link`. That is safe because the wheel spins
about its own local **+X**, and a rotation about X leaves X invariant — so the
direction is constant in world terms as the wheel turns. (Mecanum wheels, with
rollers at 45°, do not have this property and genuinely need chassis-frame
`fdir1`; that is why the official Gazebo mecanum example looks more complicated.)

Two caveats:

1. **`mu = 0.05` and `mu2 = 1.0` are modelling choices, not measurements.**
   Nothing in the STEP or any datasheet gives a friction coefficient. They are
   the first knobs to reach for if the sim drives differently from the hardware.
2. **The friction block is written the un-obvious way on purpose.** sdformat
   accepts friction from a URDF two ways, and they are not equivalent:

   | Form | Attributes on `<fdir1>` |
   |---|---|
   | flat — `<gazebo reference=…><mu1/><mu2/><fdir1/>` | **silently dropped** |
   | blob — `<gazebo reference=…><collision><surface>…` | preserved |

   The flat form is what most examples use, and it reads `<fdir1>` through a
   text-only helper that then re-emits a fresh element, so `gz:expressed_in`
   would vanish without a word. This package uses the nested blob form, which
   sdformat grafts in via a deep clone that copies attributes. (Inside the
   blob the tag is SDF's `<mu>`, *not* `<mu1>` — that spelling belongs only to
   the flat form.)

   I still could not run Gazebo, so check it once:
   ```bash
   xacro urdf/mmr_bot.urdf.xacro > /tmp/r.urdf && gz sdf -p /tmp/r.urdf | grep -A2 fdir1
   ```
   If the attribute *has* been dropped, the fix is **not** to leave `1 0 0`
   bare. DART resolves a bare `fdir1` in the **collision** frame, which here
   carries `rpy="0 π/2 0"`, so the fallback is `<fdir1>0 0 1</fdir1>` —
   collision-local Z, which that rotation maps onto the link's +X axle.
   **Failure signature:** driving forwards and rotating look fine, but
   strafing is sluggish, veers, or stalls.

#### Fixed joints do not survive URDF → SDF conversion

This surprises people, so it is worth stating plainly. sdformat 14 lumps
fixed-joint children **into their parent**, and our root is `base_footprint`, so
the model Gazebo actually simulates has one chassis body named `base_footprint`
— `base_link` and `laser_frame` are *absorbed into it*. `gz topic -l` showing no
`base_link` is correct, not a bug.

This is harmless here, and deliberately left alone:

- TF is unaffected — `robot_state_publisher` builds `/tf` from the URDF, not
  from Gazebo's SDF.
- The lidar `<sensor>` is pose-compensated as it is moved, so it stays in the
  right place, and `<gz_frame_id>laser_frame</gz_frame_id>` makes the published
  `LaserScan` carry a `frame_id` that matches TF.
- `base_footprint` having no `<inertial>` is fine; sdformat allocates one and
  accumulates the lumped masses.

**Do not "fix" this with `<disableFixedJointLumping>`.** That tag does not
preserve a fixed joint — it converts it to a *revolute* joint with (0,0) limits,
which would take this robot from the specified 9 movable joints to 13. If you
ever do need a frame kept, the correct tag is `<preserveFixedJoint>`, and its
`reference` is the **joint** name, not the child link.

#### Controllers

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all |
| `wheel_velocity_controller` | `velocity_controllers/JointGroupVelocityController` | `wheel_0/1/2_joint` |
| `arm_position_controller` | `position_controllers/JointGroupPositionController` | the 6 SO-101 joints |

Every one of those plugin strings was read from the pluginlib export XML on the
`jazzy` branch of `ros2_controllers`, not written from memory.

**Joint order is load-bearing.** `Float64MultiArray` carries no names, so the
`joints:` order in `config/controllers.yaml` is the only thing binding a number
to a wheel. It must match `wheel_angles_deg` in `kiwi_drive_node`. Reorder one
without the other and the robot drives off at the wrong heading with no error
anywhere. `tools/check_phase2.py` exists specifically to catch this — it
cross-checks the YAML, the `<ros2_control>` block, the URDF joint origins and
`kiwi_kinematics.py` against each other.

Arm limits are **not** repeated in `controllers.yaml`. They live on the
`<limit>` tags in `arm.xacro`, carried verbatim from upstream, and ros2_control
reads them from the URDF. One source of truth.

#### The arm is position-controlled, the wheels velocity-controlled

A LeRobot policy emits joint positions, so `position` is the interface that
matches the trained action space, and the joint names stay **unprefixed**
(`shoulder_pan`, not `arm_shoulder_pan`) exactly as in Phase 1. Do not tidy
those.

#### Still no hardware interface

`ros2_control.xacro` takes a `sim` parameter, but only `sim:=true` resolves to a
real plugin. There is no ESP32 `hardware_interface` in this package. Expanding
with `sim:=false` deliberately emits a plugin name that does not exist, so
ros2_control fails loudly at load rather than silently driving nothing.

Phase 3 below does not use `ros2_control` at all — on real hardware the ESP32
*is* the hardware interface, reached over UDP.

</details>

---

## Repository layout

```
mmr_pkg/
├── package.xml, CMakeLists.txt
├── urdf/       mmr_bot.urdf.xacro, base.xacro, sensors.xacro, arm.xacro,
│               inertial_macros.xacro
│               ros2_control.xacro, gazebo.xacro        ← Phase 2
├── meshes/     visual/ (+ .mtl), collision/, arm/, collision/arm/
├── config/     controllers.yaml                        ← Phase 2
│               esp32_bridge.yaml                       ← Phase 3 bridge tuning
├── worlds/     mmr_world.sdf                           ← Phase 2
├── launch/     display.launch.py, gazebo.launch.py
│               robot.launch.py, teleop.launch.py        ← Phase 3
├── rviz/       display.rviz, teleop.rviz                ← Phase 3
├── generated/  robot.urdf, robot_phase1.urdf   ← expanded xacro, NOT named build/
├── esp32/      MotionTestOriginal/MotionTestOriginal.ino  ← the firmware
│               controller.py         ← the original pygame client
├── mmr_pkg/    kiwi_kinematics.py   ← THE FILE TO DIFF AGAINST YOUR FIRMWARE
│               kiwi_drive_node.py   ← ROS wrapper, no kinematics in it
│               esp32_protocol.py    ← wire protocol + ROS→PWM, no ROS imports
│               bridge_core.py       ← watchdog/limiter/link state, no ROS imports
│               esp32_bridge.py      ← ROS wrapper, no arithmetic in it
│               teleop_keys.py       ← THE KEY MAP, one place. No ROS imports
│               kb_teleop.py         ← terminal + ROS wrapper, no key logic in it
├── scripts/    kiwi_drive_node, esp32_bridge, kb_teleop ← `ros2 run` entry points
├── test/       test_kiwi_kinematics.py    16 cases ┐
│               test_esp32_bridge.py       22 cases │ 119 total,
│               test_bridge_core.py        45 cases │ none need ROS
│               test_teleop_keys.py        36 cases ┘
└── tools/      STEP parsing, measurement and mesh extraction scripts, plus the
                offline xacro/URDF/control checkers, firmware_diff.py and
                check_repo.py. Not installed by CMake — these are provenance for
                every number above, not runtime code.
```

The `mmr_pkg/` modules come in pairs on purpose: a file with **no ROS imports**
holding the logic, and a thin ROS node wrapping it. That is why the 119 tests
run on a laptop with no ROS installed, and it is not a stylistic preference —
see *"Why the logic lives outside the nodes"* under
[Under the hood](#under-the-hood).

`generated/` was called `build/` until it collided with colcon's own build
directory and produced `failed to create symbolic link … because existing path
cannot be removed: Is a directory`. Do not name anything in the source tree
`build/`, `install/` or `log/`; `tools/check_repo.py` now fails if you do.

The `esp32/` directory is the firmware and its original pygame client, kept in
the tree because they are the *authority* on how the robot actually behaves.
`tools/firmware_diff.py` reads the `.ino` directly, so it cannot drift.

Note `mmr_pkg/` (the Python module) and `urdf/` are siblings. The package is
`ament_cmake` because it is description-first; `ament_cmake_python` bolts the
one Python library onto it rather than splitting into two packages.

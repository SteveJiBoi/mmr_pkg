# Configuration

<sub>[← README](../README.md) · [Quick start](../README.md#quick-start) · [Troubleshooting](troubleshooting.md) · [SLAM & Nav2](slam-and-nav2.md) · **Configuration** · [Hardware](hardware.md) · [Design](design.md) · [Status](status.md)</sub>

---

One file per concern: `config/esp32_bridge.yaml` for the drive
bridge, `config/slam_toolbox.yaml` and `config/nav2.yaml` for mapping
and navigation (those two are covered in
[slam-and-nav2.md](slam-and-nav2.md)).

## Launch arguments

Every argument every launch file accepts, with its default. Anything shown as
*path* defaults to a file inside this package and only needs overriding if you
are pointing at your own.

| Launch file | Argument | Default |
|---|---|---|
| `display.launch.py` | `model` | *path* — `urdf/mmr_bot.urdf.xacro` |
| | `rviz_config` | *path* — `rviz/display.rviz` |
| | `gui` | `true` — `false` swaps the slider GUI for a plain `joint_state_publisher` |
| `gazebo.launch.py` | `world` | *path* — `worlds/mmr_world.sdf` |
| | `headless` | `false` |
| | `spawn_z` | `0.0` |
| | `rviz` | `false` |
| `robot.launch.py` | `esp32_ip` | `10.229.5.249` — **set this**; it follows the DHCP lease |
| | `bridge_params` | *path* — `config/esp32_bridge.yaml` |
| | `model` | *path* |
| | `serial_port` | `/dev/ttyUSB0` (lidar) |
| | `video_device` | `/dev/video0` |
| | `pixel_format` | `YUYV` — the **camera's** format. YUYV/GREY only, *not* MJPEG |
| | `image_width` / `image_height` | `640` / `480` — drop to `320`/`240` if the feed lags |
| | `camera_info_url` | `''` — no intrinsic calibration; see TODO 14 |
| | `lidar`, `camera`, `bridge` | `true` — each disables independently |
| `teleop.launch.py` | `rviz_config` | *path* — `rviz/teleop.rviz` |
| `slam.launch.py` | `slam_params` | *path* — `config/slam_toolbox.yaml` |
| | `scan_topic` | `/scan` |
| | `odom_source` | `rf2o` — `none` if something else already publishes `odom → base_footprint` |
| | `use_sim_time` | `false` |
| `nav2.launch.py` | `params_file` | *path* — `config/nav2.yaml` |
| | `map` | `''` — empty means `slam_toolbox` is providing `map → odom` live |
| | `use_sim_time` | `false` |
| | `autostart` | `true` |

The description also builds with no simulation tags at all, which is what
`display.launch.py` uses:

```bash
xacro urdf/mmr_bot.urdf.xacro gazebo:=false > /tmp/robot.urdf
```

And the one command worth running first in Gazebo, because it is the motion that
only works if the omniwheel friction survived the URDF → SDF conversion:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.2}}'
```

## Teleop

The key map is a table at the top of `mmr_pkg/teleop_keys.py` and that is the
only place to change it. A test asserts no key has two jobs, that every binding
is a unit vector on exactly one axis, and that all six directions are present,
so a bad edit fails the suite rather than surprising you on the robot.

Defaults are 0.5 m/s and 1.0 rad/s. With the bridge's default normalisation that
is about half PWM — brisk for an indoor omni base, so start there. Those units
are declared, not measured; see [TODO 13](status.md#assumptions-and-todos).

**SPACE clears the held-key set, not just the output.** A key that is still
physically down therefore cannot restart the robot on its next auto-repeat. That
distinction is the difference between an emergency stop and a pause.

**`key_timeout` (0.6 s) must exceed your terminal's initial auto-repeat delay**
(~0.5 s typically, but configurable per system). A terminal reports no key-up
event, so release is *inferred* from auto-repeat ceasing. The cost is a bounded
lag between lifting a key and the robot stopping; the firmware's 400 ms deadman
sits underneath regardless.

`teleop_twist_keyboard` still works as a second opinion on the same `/cmd_vel`,
which is how you decide whether a problem is the keyboard node or the bridge.
Hold **shift** with that one: its unshifted `j`/`l` turn, and only the shifted
`J`/`L` strafe.

## Bridge parameters

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

## Is it actually working? — `/diagnostics`

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

## Calibrating the ESP32 bridge

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

**There is now a better way to do step 2, and it does not need a stopwatch.**
`slam.launch.py` runs `rf2o_laser_odometry`, which reports metres travelled from
the lidar, entirely independently of the PWM map:

```bash
ros2 launch mmr_pkg slam.launch.py                # in another terminal
ros2 topic echo /odom --field pose.pose.position  # read start and end
```

That is the first measurement of real distance this robot has ever been able to
make. Until someone takes it, treat every velocity in this package — including
Nav2's — as a fraction of full scale wearing the units of m/s. See
[SLAM and Nav2](slam-and-nav2.md#units-these-are-not-metres-per-second).

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

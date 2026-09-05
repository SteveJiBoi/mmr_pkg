# SLAM and Nav2

<sub>[← README](../README.md) · [Quick start](../README.md#quick-start) · [Troubleshooting](troubleshooting.md) · **SLAM & Nav2** · [Configuration](configuration.md) · [Hardware](hardware.md) · [Design](design.md) · [Status](status.md)</sub>

---

Mapping with `slam_toolbox` and autonomous navigation with Nav2, on a robot with
**no encoders**. That constraint is the whole story, so it comes first.

```bash
ros2 launch mmr_pkg robot.launch.py esp32_ip:=10.229.5.249   # Pi
ros2 launch mmr_pkg slam.launch.py                            # desktop
ros2 launch mmr_pkg nav2.launch.py                            # desktop
ros2 run  mmr_pkg kb_teleop                                   # desktop, drive it around
```

**None of this has been run.** There is no ROS on the machine that wrote it.
See [status.md](status.md#not-checked-here--please-run-these) for the specific
things to watch when you do.

---

## The odometry problem

`slam_toolbox` does not invent odometry, and neither does Nav2. Both require a
TF `odom → base_footprint` to already exist; `slam_toolbox` refines that estimate
by matching scans, and Nav2 plans in a frame that is anchored to it. Give them a
robot with nothing publishing `odom` and they start cleanly, activate cleanly,
and then sit logging transform timeouts forever. Nothing says "you have no
odometry."

This robot has no encoders. So something has to produce that transform, and
there are only two honest options:

| | What it measures | Why not |
|---|---|---|
| Integrate `/cmd_vel` | Nothing. It reports what we **asked** the robot to do | The ESP32 has no closed loop, and ~7.7% of the packets carrying those commands are lost. The output would look exactly like odometry and be fiction |
| Scan matching (`rf2o`) | Planar motion between consecutive lidar scans, in metres | Nothing — this is what is used |

`rf2o_laser_odometry` estimates the range flow between successive scans and
integrates it into a pose. That is a real measurement from a real sensor, which
is the only reason it is acceptable here. Faking odometry would be worse than
having none, because everything downstream silently trusts it.

### What rf2o costs you

Stated plainly, because these are not edge cases — they are Tuesday:

- **It fails in featureless space.** A long blank corridor, or the middle of an
  empty hall, gives scan matching nothing to lock onto and the estimate slides.
  Encoders would not care. This is the single biggest practical difference.
- **It is not independent of `slam_toolbox`.** `slam_toolbox` also matches
  scans, so the two agreeing does not mean either is right — they share a
  failure mode. On a robot with encoders they would be genuinely independent
  sources of evidence. If the map smears, raise `distance_variance_penalty` in
  `config/slam_toolbox.yaml` to make it lean less on rf2o's guess.
- **It degrades with fast rotation.** The C1 scans at 10 Hz; spin quickly and
  consecutive scans overlap too little to match. **Turn slowly while mapping.**

`launch/slam.launch.py` sets rf2o's `freq: 10.0` rather than its own default of
20.0, because the C1 produces 10 scans a second and asking for 20 does not
create information — it re-runs the estimator on scans it has already used.

### rf2o is a workspace checkout, not a rosdep key

Exactly like `sllidar_ros2`, and for exactly the same reason it is **not** in
`package.xml`: naming a key that the ROS index does not have makes
`rosdep install` abort before it resolves anything else.

```bash
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/ros2_ws && colcon build --packages-select rf2o_laser_odometry
```

It matters more than it looks: it is the **only** source of `odom` on this
robot, so neither `slam_toolbox` nor Nav2 will do anything at all without it.

---

## Where each node runs, and why

Everything in `slam.launch.py` and `nav2.launch.py` runs on the **desktop**. No
node in either file touches hardware.

`/scan` is about 20 kB/s — 500 points at 10 Hz — so shipping it over WiFi is
cheap. `slam_toolbox`'s scan matching and loop closure are not cheap, and the
Pi 5 is already running the lidar driver, the camera and the UDP bridge. Moving
the expensive half to the desktop costs almost no bandwidth.

```
  Pi                              WiFi          Desktop
  ──                              ────          ───────
  sllidar_ros2  ── /scan ────────────────────▶  rf2o_laser_odometry
  robot_state_publisher ─ /tf ───────────────▶     └─ /odom, odom → base_footprint
  esp32_bridge  ◀── /cmd_vel ─────────────────  slam_toolbox
                                                   └─ /map, map → odom
                                                Nav2 (controller, planner, bt)
                                                   └─ /cmd_vel
```

Note the loop: Nav2 publishes `/cmd_vel`, which crosses the WiFi to the bridge
and then the second WiFi hop to the ESP32. The measured Pi → ESP32 path averages
~138 ms with a ~257 ms peak against the firmware's 400 ms deadman. Nav2's
control loop assumes its commands arrive; this one is the least reliable link in
the system, and no Nav2 parameter fixes that.

---

## Exactly one thing may publish `map → odom`

This is an either/or, and getting it wrong makes the robot jump between two
poses rather than produce an error:

| | `map → odom` from | `slam_toolbox` |
|---|---|---|
| `nav2.launch.py` (default `map:=""`) | `slam_toolbox`, via `slam.launch.py` | **must be running** |
| `nav2.launch.py map:=~/my_map.yaml` | `map_server` + AMCL, from `nav2_bringup` | **must NOT be running** |

`nav2.launch.py` only includes `localization_launch.py` when `map` is non-empty,
so the two can never both start *from this package*. It cannot stop you starting
`slam.launch.py` by hand alongside a saved map, which is the way to get two
publishers fighting.

In **both** modes you still need `odom → base_footprint`, which Nav2 does not
produce either. Navigating a saved map still needs rf2o running.

Save a map when the room looks right:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

---

## The three holonomic traps

Nav2's stock parameters describe a differential-drive robot. Three of the
defaults do not error on a kiwi drive — they quietly delete sideways motion. The
result is a robot that drives, turns, never strafes, and logs nothing about it.
This is why `config/nav2.yaml` is not optional.

**1. `controller_server/min_y_velocity_threshold: 0.5`**
A deadband. Any commanded `vy` below 0.5 is treated as zero, which on this robot
is nearly every strafe it will ever be asked for. Set to `0.001`, matching what
the x and theta thresholds are for.

**2. `velocity_smoother/max_velocity: [0.5, 0.0, 2.0]`**
The middle element is Y. Upstream's `0.0` clamps every strafe to zero
*downstream* of the controller — so the controller can be correctly configured,
emit a perfect sideways command, and have it zeroed on the way out. Set to
`[0.35, 0.35, 1.0]`.

**3. `amcl/robot_model_type: "nav2_amcl::DifferentialMotionModel"`**
The differential motion model has no way to represent `vy`, so every strafe is
scored as an impossible motion and the particle cloud diverges. Set to
`OmniMotionModel`. Only bites in saved-map mode, which is why it is easy to miss
while mapping works fine.

Two more that are not silent but are wrong for this robot:

- **`FollowPath/motion_model: "DiffDrive"` → `"Omni"`.** MPPI samples the
  trajectories it considers; with `DiffDrive` it never samples one containing
  `vy`, so it cannot choose a strafe even when a strafe is the answer.
- **`PreferForwardCritic` removed.** It penalises any trajectory that is not
  forward motion. This robot has three identical wheels and no front — there is
  no reason for it to prefer one heading over another, and paying a cost to
  rotate before translating is a pure loss.

### The rest of the changes

| Parameter | Upstream | Here | Why |
|---|---|---|---|
| `laser_max_range` / `min_range` (amcl) | 100.0 / 0.0 | 12.0 / 0.05 | The C1's actual reach |
| `robot_base_frame` (both costmaps) | `base_link` | `base_footprint` | `base_link` is 45 mm up; costmaps are a floor-plane problem |
| `robot_radius` (both costmaps) | 0.22 | 0.15 | **Derived:** wheel centres at 0.1355 m + roller half-envelope 0.0136 m = 0.1491 m |
| `inflation_radius` | 0.70 | 0.35 | Inflation is measured from the robot centre; 0.70 on a 0.30 m robot fills a doorway |
| `collision_monitor` `scan.min_height` | 0.15 | 0.05 | See below |

**The `collision_monitor` height is a real near-miss.** `laser_frame` sits at
0.106801 m above `base_link`, which is itself 0.045 m above the ground, so the
scan plane is at **0.151801 m**. Upstream's `min_height: 0.15` leaves it a
**1.8 mm** margin — on the one node whose entire job is stopping the robot before
it hits something. A millimetre of measurement error or a change in tyre
compression and the collision monitor sees an empty world.

---

## Units: these are not metres per second

`vx_max`, `vy_max` and `wz_max` are 0.35 and 1.0, and they are in the same
fictional units as `/cmd_vel` everywhere else in this package. The ESP32 bridge
maps `cmd_vel` 1.0 to `max_linear_pwm`, a *declared normalisation*, not a
measurement — there is no encoder and no closed loop, so nothing has ever
compared a commanded speed to a distance travelled. See
[TODO 13](status.md#assumptions-and-todos).

The only defensible statement about these numbers is a relative one: 0.35 is
below `kb_teleop`'s 0.5 default, so Nav2 will drive more slowly than you do by
hand.

**rf2o changes this, and it is the most valuable thing in this document.** It
reports metres travelled, derived from the lidar and completely independent of
the PWM map. So for the first time there is a way to make `cmd_vel` real:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.35}}'
ros2 topic echo /odom --field pose.pose.position    # over a measured 10 s
```

Distance over time is the true speed for a commanded 0.35. Put the ratio into
`max_linear_speed` in `config/esp32_bridge.yaml` and every velocity in this
package becomes a measurement. The full procedure, including the rotational
case, is in [Calibrating the ESP32 bridge](configuration.md#calibrating-the-esp32-bridge).

---

## Files

| File | What it is |
|---|---|
| `launch/slam.launch.py` | rf2o + `async_slam_toolbox_node`. Desktop |
| `launch/nav2.launch.py` | Thin wrapper around `nav2_bringup`'s own launch files. Desktop |
| `config/slam_toolbox.yaml` | Upstream `mapper_params_online_async.yaml` with 3 changes, each marked `CHANGED` |
| `config/nav2.yaml` | Upstream `nav2_params.yaml` with 14 changes, each marked `CHANGED` |

Both config files are kept **whole** rather than trimmed to the parameters this
package tunes. Two reasons: they can be diffed against upstream when you change
distro, and Nav2's lifecycle manager refuses to activate the *entire* stack if
any managed node fails to configure — so deleting a section you believe is
unused is how you lose everything.

`async_slam_toolbox_node`, not `sync`: the synchronous node blocks until every
scan is processed, which on a live robot means falling progressively further
behind rather than dropping a scan and staying current.

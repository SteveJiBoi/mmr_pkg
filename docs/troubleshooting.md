# When it doesn't work

<sub>[← README](../README.md) · [Quick start](../README.md#quick-start) · **Troubleshooting** · [SLAM & Nav2](slam-and-nav2.md) · [Configuration](configuration.md) · [Hardware](hardware.md) · [Design](design.md) · [Status](status.md)</sub>

---

Ordered by how often each one is the answer. Every check here is a command you
can run, not a thing to eyeball.

## The robot doesn't move

| What you see | Most likely cause | What to do |
|---|---|---|
| No movement, **no error anywhere** | Wrong `esp32_ip`. UDP has no connection, so packets to a wrong host vanish in complete silence — this is the single most confusing failure this system has | `ros2 topic echo /diagnostics` — look for `state: down` with `replies_received: 0`. Then check the address the ESP32 prints on its serial console at boot |
| Moves, then stops after ~0.4 s, repeatedly | The firmware's 400 ms deadman is firing because packets stopped arriving | `ros2 topic hz /cmd_vel` should show ~25 Hz while a key is held. If it does, the loss is on the network |
| Stutters or lurches while driving | Packet loss. The measured link drops ~7.7% | Raise `publish_rate` for redundancy — but see [Networking](configuration.md#networking), because **this is a network problem and software cannot fix it** |
| Nothing happens when you press keys | The terminal lost focus | Click the `kb_teleop` window. It must keep focus |
| `kb_teleop` exits immediately saying stdin is not a TTY | You launched it from a launch file or through a pipe | Run it directly in its own terminal. It refuses rather than silently publishing zeros forever |

### Bisect it in four commands

Each one tells you which half of the chain to stop looking at. Run them in
order and stop at the first that fails.

```bash
ros2 topic hz /cmd_vel        # 1. desktop: is the keyboard producing anything?
ros2 topic list | grep scan   # 2. desktop: can you see the Pi's graph at all?
ros2 topic echo /diagnostics  # 3. Pi: is the bridge reaching the ESP32?
python tools/find_esp32.py    # 4. Pi: what address is the ESP32 actually on?
```

1. **No `/cmd_vel`** — the problem is `kb_teleop`, not the robot. It must be
   the focused terminal, and it must be a real TTY.
2. **`/cmd_vel` is there but no `/scan`** — the two machines cannot see each
   other. Same network, same `ROS_DOMAIN_ID`, and no client isolation on the
   AP. `/cmd_vel` published on the desktop never reaches a bridge on the Pi.
3. **`state: down` with `replies_received: 0`** — ROS is fine and the ESP32 is
   not answering. Go to step 4.
4. **`find_esp32.py` reports an address that is not the one you launched
   with** — that was the whole problem. Relaunch with the address it found.

### The ESP32 address is a DHCP lease, not a fixed property

The single most common cause of "no movement, no error". The board prints its
address once at boot and nothing reports it again; that number then lives in
your launch command until a lease expires or the network changes underneath
it. `esp32_ip` in `launch/robot.launch.py` defaults to `10.229.5.249`, which
was correct on the network it was written on and is **not** a guarantee.

```bash
python tools/find_esp32.py             # sweeps every local subnet
python tools/find_esp32.py 10.0.0.0/24 # or just one
```

It uses the firmware's own `PING` → `PONG`, so a reply proves the board is
powered, on WiFi, listening and parsing — not merely that something holds the
address, which is all an ICMP ping proves. **Run it on the Pi**, or on any
machine on the robot's subnet.

### Can I drive it over the USB cable instead?

**Not without a firmware change.** It is worth being precise about why, because
the cable is genuinely useful for diagnosing this — just not for control.

`esp32/MotionTestOriginal/MotionTestOriginal.ino` contains no `Serial.read()`,
no `Serial.available()`, nothing that reads the port at all. `Serial` is
print-only debug output. The only thing that ever produces a motor command is
`parsePacket()`, and its single caller is the `udp.onPacket()` callback. So
commands arrive by UDP or they do not arrive.

Worse, WiFi is not optional in the current sketch:

```cpp
WiFi.begin(ssid, password);
if (WiFi.waitForConnectResult() != WL_CONNECTED) {
  Serial.println("WiFi failed");
  while (1) delay(1000);          // never returns
}
```

If the network is not there, `setup()` never finishes, `udp.listen()` never
runs, and the board sits in that loop with the motors stopped no matter what
ROS does. A robot cabled to the Pi with no WiFi is a robot that cannot be
driven by this firmware.

**What the cable is for.** It carries the boot log, which is the fastest
diagnostic available and the only thing that distinguishes the two failures
above from each other:

```bash
screen /dev/ttyUSB0 115200      # or /dev/ttyACM0; Ctrl-A then K to quit
```

Tap the board's EN/RST button and read:

| Boot log says | Meaning |
|---|---|
| `Ready. IP: <address>` then `UDP listening on port 1234` | Healthy. Use that address as `esp32_ip:=` |
| `WiFi failed` | Stuck in the retry loop. Fix the network or the credentials; nothing on the ROS side can help |
| nothing at all | Wrong port, wrong baud, or the board is not powered |

The SSID and password are compiled in (`ssid = "test"`), so a new network means
reflashing, not a config change. If you want serial control as a real
transport, that is a firmware feature to add — read `Serial` in `loop()` and
feed the same line into `parsePacket()`'s parser — plus a matching transport in
`mmr_pkg/esp32_bridge.py`, which is UDP-only today.

## It moves, but wrongly

| What you see | Cause | Fix |
|---|---|---|
| Strafes the **wrong way** (A goes right) | Motor wiring handedness. Expected to be a coin flip until [the wiring is confirmed](hardware.md#-the-firmware-and-the-simulation-disagree-about-which-way-is-left) | Set `strafe_sign: -1` in `config/esp32_bridge.yaml`. **Do not** edit the angle arithmetic |
| Curves away from the commanded heading when translating *and* spinning at speed | The firmware clamps each wheel to ±255 **after** subtracting rotation, so the mix saturates | `prevent_clipping: true` |
| Drives smoothly but not in the commanded direction; yaw bleeds into translation | Classic symptom of a 30° wheel-angle error | [Read this](hardware.md#the-30-wheel-angle-discrepancy--read-this-before-driving) |
| Keeps coasting after you release a key | `key_timeout` too long for your terminal's auto-repeat | Lower it. A terminal has no key-up event, so release is *inferred* — see [TODO 16](status.md#assumptions-and-todos) |

## The map never appears

| What you see | Cause | What to do |
|---|---|---|
| RViz Map display empty, `/map` has **0 publishers**, no `map` frame — while `/scan` and `odom → base_footprint` are both fine | `slam_toolbox` is a **lifecycle node** sitting in `unconfigured`. It subscribes to nothing and logs nothing | `ros2 lifecycle get /slam_toolbox` — want `active [3]`. Fixed in `slam.launch.py`, which now drives configure+activate; if you start `slam_toolbox` by hand you must do it yourself |
| `/odom` never publishes; rf2o logs `Waiting for laser_scans....` | rf2o is not receiving `/scan`. On a split desktop/Pi setup this is nearly always the network, not the code | `ros2 topic hz /scan` on the machine running rf2o. rf2o subscribes `best_effort`, so it is **not** a QoS mismatch |
| Two `/rf2o_laser_odometry` nodes, or ROS warns about duplicate node names | SLAM started twice — `teleop.launch.py slam:=true` **and** `slam.launch.py` | Run one or the other. Duplicate node names are undefined behaviour: parameter sets collide and instances shadow each other |
| Map builds but is geometrically wrong | rf2o initialised before `base_footprint → laser_frame` arrived. Its `setLaserPoseFromTf()` logs the failure, then carries on with a **zero transform**, and the caller ignores the return value | Restart rf2o once the Pi's TF is up. Upstream bug, not fixable from config |

The lifecycle one deserves emphasis: every other part of the chain reports its
own failure, and this one does not. `slam_toolbox` appears in `ros2 node list`,
answers parameter queries, exits cleanly, and produces nothing. The only thing
that tells you is `ros2 lifecycle get`.

## Nothing shows up in RViz

| What you see | Cause | Fix |
|---|---|---|
| `Frame [odom] does not exist` | Nothing is publishing `odom → base_footprint`. Correct on its own: the robot has no encoders, so `robot.launch.py` alone cannot produce odometry | Either set the fixed frame to `base_footprint`, or start `slam.launch.py`, which runs `rf2o_laser_odometry` and derives `odom` from the lidar — see [SLAM and Nav2](slam-and-nav2.md) |
| `/scan` publishes but is invisible | `frame_id` is not in the TF tree | `ros2 topic echo /scan --once` — `frame_id` must be `laser_frame`. `robot.launch.py` passes this to the driver already |
| Robot model missing, TF tree stops at `base_link` | `joint_state_publisher` isn't running, so no joint has a position | It is started by `robot.launch.py` on purpose — see [Frames](hardware.md#frames-and-what-you-will-and-will-not-see-in-rviz) |
| **Camera** display: `invalid position calculation (nans or infs)` | No intrinsic calibration, so `CameraInfo` is all zeros and RViz computes `0/0` | Use the **Image** display, which needs no TF and no calibration. This is the correct tool here, not a workaround |
| PC sees no topics from the Pi at all | Discovery, not the robot | `ros2 multicast send` / `receive`. Check for a leftover `ROS_LOCALHOST_ONLY=1` **first** — see [Networking](configuration.md#networking) |

## The camera feed lags

Do the arithmetic before touching anything: 640×480 in `rgb8` is
**921,600 bytes per frame**, and at 30 fps that is **221 Mbit/s**. No WiFi link
in this setup carries that. The feed does not "lag" — it is being delivered as
fast as the radio allows, and everything else is queueing behind it.

| What you see | Cause | Fix |
|---|---|---|
| Video runs seconds behind the robot; `/scan` and `/cmd_vel` are also sluggish | Raw transport. The default `/image_raw` is uncompressed | Install `compressed_image_transport` **on both machines** and set the RViz Image display's *Transport Hint* to `compressed`. `rviz/teleop.rviz` already does |
| RViz aborts the moment you add or enable the Image display | `compressed_image_transport` is missing **on the desktop**. RViz Jazzy offers the `compressed` hint whether or not the plugin is installed and aborts when it is not — so a missing dependency looks like a crash | `sudo apt install ros-jazzy-image-transport-plugins` on the desktop too |
| `ros2 topic list` shows no `/image_raw/compressed` | The plugin is missing **on the Pi**, so `v4l2_camera` can only publish raw | Same install, on the Pi. `robot.launch.py` logs a warning at startup when it cannot find the package |
| Feed is smooth, then falls behind and never catches up | RViz queue depth. Five 640×480 `rgb8` frames is 4.6 MB of backlog, and RViz shows you the **oldest** frame in the queue | *Depth* `1`, *Reliability Policy* `Best Effort`. Both are already set in `rviz/teleop.rviz` |
| Still too slow with compression on | The frame is simply too big for the link | `ros2 launch mmr_pkg robot.launch.py image_width:=320 image_height:=240`. Halving each dimension is a 4× cut |

Two things that look like fixes and are not:

- **`pixel_format:=MJPEG`.** `v4l2_camera`'s `pixel_format` takes the *camera's*
  format, and the supported set is YUYV/GREY — not MJPEG. Setting it does not
  compress the ROS topic; compression on the wire is `image_transport`'s job.
- **Raising the ROS queue size.** A bigger queue makes the backlog longer, which
  makes the lag worse, not better. Depth 1 is the fix.

## SLAM or Nav2 does nothing

| What you see | Cause | Fix |
|---|---|---|
| `slam_toolbox` logs transform timeouts and `/map` never appears | Nothing publishes `odom → base_footprint`. This robot has no encoders | Run `slam.launch.py` with its default `odom_source:=rf2o`. See [SLAM and Nav2](slam-and-nav2.md) |
| `Package 'rf2o_laser_odometry' not found` | It is a workspace checkout, not a rosdep key, exactly like `sllidar_ros2` | Clone `MAPIRlab/rf2o_laser_odometry` branch `ros2` next to this package and `colcon build` |
| Nav2 activates cleanly, then rejects every goal as untransformable | `nav2.launch.py` was started with the default `map:=""` but `slam.launch.py` is not running, so nothing publishes `map → odom` | Start `slam.launch.py` first — the launch file logs this exact reminder |
| The robot jumps between two poses | Both `slam_toolbox` and AMCL are publishing `map → odom` | Pick one: live mapping (`map:=""`) **or** a saved map (`map:=<file>`), never both |
| Navigates fine, but never strafes | Nav2's stock parameters silently delete `vy` | Use `config/nav2.yaml`, which is the default. The three traps are listed in [SLAM and Nav2](slam-and-nav2.md) |
| The map smears in a long, featureless corridor | `rf2o` is scan matching, and a blank corridor gives it nothing to lock onto. This is the honest cost of having no encoders | Map in feature-rich space, turn slowly, and see the scan-matcher notes in `config/slam_toolbox.yaml` |

## It won't build or won't start

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

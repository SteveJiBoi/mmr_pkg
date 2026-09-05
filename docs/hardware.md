# Hardware — frames, geometry and provenance

<sub>[← README](../README.md) · [Quick start](../README.md#quick-start) · [Troubleshooting](troubleshooting.md) · [SLAM & Nav2](slam-and-nav2.md) · [Configuration](configuration.md) · **Hardware** · [Design](design.md) · [Status](status.md)</sub>

---

Every number here is measured from the Fusion 360 STEP export
(`mmr_bot.step`) by a script in `tools/`, not typed in by hand. Where
something could not be measured it is a TODO in
[status.md](status.md#assumptions-and-todos), not a plausible-looking
guess.

## Frame tree

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

## Derived geometry

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

### Frame convention

The CAD origin is *not* the wheel-triad centre. The three wheel centres'
centroid is at CAD (0.196, 0.001) mm, and `base_link` is placed there so the
kiwi kinematics are exactly symmetric. The full transform is

```
p_link = Rz(+90°) · ( p_cad − (0.196, 0.001, 44.941) mm )
```

i.e. **robot +X (forward) = CAD −Y**, the direction the lidar, camera and arm
all face. You confirmed this choice; see [Corrections](#corrections-to-the-brief).

### The axle-height consistency check

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

## Wheel sign convention

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

**With only `robot.launch.py` running, the fixed frame is `base_footprint`, not
`odom`.** Nothing in that launch file publishes `odom → base_footprint`, because
the robot has no encoders. The robot sits still in the middle of the view while
the world sweeps around it as you drive; that is correct, not a bug, and setting
the fixed frame to `odom` yields *"Frame [odom] does not exist"*.

**`slam.launch.py` is what fills that gap**, by running `rf2o_laser_odometry` to
derive `odom → base_footprint` from the lidar rather than from encoders. Once it
is up, `odom` and `map` are both real frames and are the right ones to view in.
What that costs — and it does cost something — is in
[SLAM and Nav2](slam-and-nav2.md).

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

## Meshes

One mesh per link, all in metres, all referenced by `package://mmr_pkg/…`.
Regenerate with `python tools/extract_meshes.py`.

### Visual

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

### Decimation policy, and one honest trade-off

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

### Collision

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

## The SO-101 arm

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

### Arm collision meshes

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

### Two corrections to the brief's §5

- **§5.3 is a no-op.** The premise ("missing base collision") is false: the
  current `so101_new_calib.urdf` `base_link` has 4 visuals *and* 4 collisions.
  Nothing was substituted; adding a redundant primitive would have been wrong.
  The real collision problem was different — upstream points collision at the
  visual meshes — and is fixed above.
- **`so101_old_calib.urdf` would not have been a drop-in alternative:** it
  renames every link and has no `gripper_frame_link`.

### Prefix caveat

Per instructions, links are prefixed but joints are not. That is right for a
single arm, but it means **instantiating the macro twice would collide on joint
names**. If you ever add a second arm, the joint names have to be prefixed too —
and at that point the LeRobot action-space mapping needs revisiting anyway.

## Simulation and control

### What talks to what (simulation)

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

### Omniwheel friction — the part most likely to need tuning

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

### Fixed joints do not survive URDF → SDF conversion

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

### Controllers

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

### The arm is position-controlled, the wheels velocity-controlled

A LeRobot policy emits joint positions, so `position` is the interface that
matches the trained action space, and the joint names stay **unprefixed**
(`shoulder_pan`, not `arm_shoulder_pan`) exactly as in Phase 1. Do not tidy
those.

### Still no hardware interface

`ros2_control.xacro` takes a `sim` parameter, but only `sim:=true` resolves to a
real plugin. There is no ESP32 `hardware_interface` in this package. Expanding
with `sim:=false` deliberately emits a plugin name that does not exist, so
ros2_control fails loudly at load rather than silently driving nothing.

The real robot does not use `ros2_control` at all — on hardware the ESP32 *is*
the hardware interface, reached over UDP by `esp32_bridge`.

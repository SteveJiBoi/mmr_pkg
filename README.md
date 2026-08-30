# mmr_pkg — `mmr_bot` description + kiwi drive

URDF/xacro description of a three-wheeled kiwi-drive holonomic base carrying an
SO-101 arm, a Raspberry Pi 5 and an RPLIDAR C1. Geometry is derived from the
Fusion 360 STEP export (`mmr_bot.step`); the arm is vendored from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100).

- **Phase 1** — description only: meshes, frames, inertials, RViz.
- **Phase 2** — Gazebo Harmonic, `ros2_control`, and the `kiwi_drive_node`
  whose IK is meant to be diffed against your ESP32 firmware.

> **Read the [Verification status](#verification-status) section before trusting
> anything here.** This package was built on a Windows machine with no ROS 2
> install. `xacro`, `check_urdf`, `rviz2`, `colcon` and **Gazebo** were *never
> run*. The checks that *were* run are offline reimplementations, listed below.
> Phase 2 in particular contains simulation behaviour that literally cannot be
> confirmed without running it — those points are called out individually.

---

## Build and run

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

The description also builds without any simulation tags, which is what
`display.launch.py` uses:

```bash
xacro urdf/mmr_bot.urdf.xacro gazebo:=false > /tmp/robot.urdf
```

---

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

---

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

---

## SO-101 arm

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
   is measurable from the same STEP.
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
    [Omniwheel friction](#omniwheel-friction--the-part-most-likely-to-need-tuning)
    for the one-line `gz sdf -p` check and the exact fallback.
11. **No hardware interface for the real robot.** `sim:=false` intentionally
    names a plugin that does not exist so it fails loudly.

---

## Corrections to the brief

The brief asked to be told when a supplied number disagreed with the STEP.
Four did.

| Brief said | STEP says | Resolution |
|---|---|---|
| wheel mounting angles **30° / 150° / 270°** | **60.004° / 180.004° / 300.005°** — a uniform **30° offset** | the STEP's angles are used. See below; this one matters most |
| omniwheel "60 mm — confirm RADIUS or DIAMETER" | max radius **30.000 mm** on all three; part is named `RodaOmni 60 mm` | 60 mm is a **diameter**; radius 0.030 m |
| base plate "200 mm — round or square?" | **round**, Ø **240.002 mm** (roundness ratio 1.0001; a square would give 1.414) | plate radius 0.120 m — the 200 mm figure is wrong |
| wheel axle height 27 mm | axle sits **30.000 mm** above the wheel-circle tangent, exactly the wheel radius | 27 mm is wrong; 30 mm is used, and the xacro derives it so it cannot disagree with the radius |

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

## Verification status

**This machine has no ROS 2.** `xacro`, `check_urdf`, `rviz2`, `colcon` and
`ros2` are all absent and were not run. To avoid claiming unverified results,
offline equivalents were written; they are development aids, and the real tools
remain the authority.

### Checked here

```bash
python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro -o build/robot.urdf
python tools/check_urdf_lite.py build/robot.urdf          # 0 failures, 2 warnings
python tools/check_phase2.py build/robot.urdf config/controllers.yaml   # 0 failures
python -m pytest test/ -q                                 # 16 passed
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

### NOT checked here — please run these

```bash
xacro urdf/mmr_bot.urdf.xacro > /tmp/robot.urdf          # must run clean
check_urdf /tmp/robot.urdf                               # 1 tree, no orphans, 9 movable
ros2 launch mmr_pkg display.launch.py                    # 0 missing-mesh warnings, 0 TF errors
colcon test --packages-select mmr_pkg                    # 16 pytest cases

gz sdf -p /tmp/robot.urdf | grep -A2 fdir1               # does gz:expressed_in survive?
ros2 launch mmr_pkg gazebo.launch.py
ros2 control list_controllers                            # 3 controllers, all `active`
ros2 topic echo /scan --once                             # frame_id must be `laser_frame`
```

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

`xacro_lite.py` implements only the subset of xacro this package uses. If real
`xacro` disagrees with it, real `xacro` is right — tell me and I will fix the
description.

---

## Phase 2 — simulation and control

### What talks to what

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
├── worlds/     mmr_world.sdf                           ← Phase 2
├── launch/     display.launch.py, gazebo.launch.py
├── rviz/       display.rviz
├── mmr_pkg/    kiwi_kinematics.py   ← THE FILE TO DIFF AGAINST YOUR FIRMWARE
│               kiwi_drive_node.py   ← ROS wrapper, no kinematics in it
├── scripts/    kiwi_drive_node      ← `ros2 run` entry point
├── test/       test_kiwi_kinematics.py                 ← 16 cases, no ROS needed
└── tools/      STEP parsing, measurement and mesh extraction scripts, plus the
                offline xacro/URDF/control checkers. Not installed by CMake —
                these are provenance for every number above, not runtime code.
```

Note `mmr_pkg/` (the Python module) and `urdf/` are siblings. The package is
`ament_cmake` because it is description-first; `ament_cmake_python` bolts the

one Python library onto it rather than splitting into two packages.


stevejidev@stevejidev-B850M-C:~/dev_ws$ ros2 launch mmr_pkg gazebo.launch.py
[INFO] [launch]: All log files can be found below /home/stevejidev/.ros/log/2026-08-30-10-15-09-930581-stevejidev-B850M-C-6128
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gazebo-1]: process started with pid [6132]
[INFO] [robot_state_publisher-2]: process started with pid [6133]
[INFO] [create-3]: process started with pid [6134]
[INFO] [parameter_bridge-4]: process started with pid [6136]
[INFO] [kiwi_drive_node-5]: process started with pid [6137]
[robot_state_publisher-2] [INFO] [1788056110.242740999] [robot_state_publisher]: Robot initialized
[create-3] [INFO] [1788056110.246862426] [ros_gz_sim]: Requesting list of world names.
[parameter_bridge-4] [INFO] [1788056110.260350257] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)] (Lazy 0)
[parameter_bridge-4] [INFO] [1788056110.260693701] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/scan (gz.msgs.LaserScan) -> /scan (sensor_msgs/msg/LaserScan)] (Lazy 0)
[kiwi_drive_node-5] [INFO] [1788056110.540630822] [kiwi_drive_node]: kiwi_drive_node up. KiwiKinematics(angles_deg=[60.0, 180.0, 300.0], R=0.03, L=0.1355)
[kiwi_drive_node-5]   cmd_vel      : cmd_vel (Twist)
[kiwi_drive_node-5]   wheel command: /wheel_velocity_controller/commands ['wheel_0_joint', 'wheel_1_joint', 'wheel_2_joint']
[kiwi_drive_node-5]   max wheel    : unlimited
[kiwi_drive_node-5]   NOTE all-positive wheel velocity = CLOCKWISE yaw.
[gazebo-1] [Dbg] [gz.cc:166] Subscribing to [/gazebo/starting_world].
[gazebo-1] [Dbg] [gz.cc:168] Waiting for a world to be set from the GUI...
[gazebo-1] [Msg] Received world [/home/stevejidev/dev_ws/install/mmr_pkg/share/mmr_pkg/worlds/mmr_world.sdf] from the GUI.
[gazebo-1] [Dbg] [gz.cc:172] Unsubscribing from [/gazebo/starting_world].
[gazebo-1] [Msg] Gazebo Sim Server v8.11.0
[gazebo-1] [Msg] Loading SDF world file[/home/stevejidev/dev_ws/install/mmr_pkg/share/mmr_pkg/worlds/mmr_world.sdf].
[gazebo-1] [Msg] Serving entity system service on [/entity/system/add]
[gazebo-1] [Dbg] [Physics.cc:906] Loaded [gz::physics::dartsim::Plugin] from library [/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins/libgz-physics-dartsim-plugin.so]
[gazebo-1] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::Physics] for entity [1]
[gazebo-1] [Dbg] [Sensors.cc:697] Configuring Sensors system
[gazebo-1] [Dbg] [Sensors.cc:557] SensorsPrivate::Run
[gazebo-1] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::Sensors] for entity [1]
[gazebo-1] [Dbg] [Sensors.cc:532] SensorsPrivate::RenderThread started
[gazebo-1] [Dbg] [Sensors.cc:337] Waiting for init
[gazebo-1] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::SceneBroadcaster] for entity [1]
[gazebo-1] [Msg] Create service on [/world/mmr_world/create_multiple] (async)
[gazebo-1] [Msg] Create service on [/world/mmr_world/create_multiple/blocking] (blocking)
[gazebo-1] [Msg] Remove service on [/world/mmr_world/remove] (async)
[gazebo-1] [Msg] Remove service on [/world/mmr_world/remove/blocking] (blocking)
[gazebo-1] [Msg] Pose service on [/world/mmr_world/set_pose] (async)
[gazebo-1] [Msg] Pose service on [/world/mmr_world/set_pose/blocking] (blocking)
[gazebo-1] [Msg] Pose service on [/world/mmr_world/set_pose_vector] (async)
[gazebo-1] [Msg] Pose service on [/world/mmr_world/set_pose_vector/blocking] (blocking)
[gazebo-1] [Msg] Light configuration service on [/world/mmr_world/light_config] (async)
[gazebo-1] [Msg] Light configuration service on [/world/mmr_world/light_config/blocking] (blocking)
[gazebo-1] [Msg] Physics service on [/world/mmr_world/set_physics] (async)
[gazebo-1] [Msg] Physics service on [/world/mmr_world/set_physics/blocking] (blocking)
[gazebo-1] [Msg] SphericalCoordinates service on [/world/mmr_world/set_spherical_coordinates] (async)
[gazebo-1] [Msg] SphericalCoordinates service on [/world/mmr_world/set_spherical_coordinates/blocking] (blocking)
[gazebo-1] [Msg] Enable collision service on [/world/mmr_world/enable_collision] (async)
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:413:17: QML ToolButton: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:413:17: QML ToolButton: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[create-3] [INFO] [1788056110.748934498] [ros_gz_sim]: Waiting messages on topic [/robot_description].
[create-3] [INFO] [1788056110.759967656] [ros_gz_sim]: Entity creation successful.
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:413:17: QML ToolButton: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[INFO] [create-3]: process has finished cleanly [pid 6134]
[INFO] [spawner-6]: process started with pid [6244]
[INFO] [spawner-7]: process started with pid [6245]
[INFO] [spawner-8]: process started with pid [6246]
[spawner-8] [INFO] [1788056111.108963114] [spawner_arm_position_controller]: waiting for service /controller_manager/list_controllers to become available...
[gazebo-1] [Msg] Enable collision service on[Msg] Gazebo Sim GUI    v8.11.0
[gazebo-1] [Dbg] [Gui.cc:275] Waiting for subscribers to [/gazebo/starting_world]...
[gazebo-1] [Dbg] [Application.cc:96] Initializing application.
[gazebo-1] [Dbg] [Application.cc:170] Qt using OpenGL graphics interface
[gazebo-1] [GUI] [Dbg] [Application.cc:657] Create main window
[gazebo-1] [GUI] [Dbg] [PathManager.cc:68] Requesting resource paths through [/gazebo/resource_paths/get]
[gazebo-1] [GUI] [Dbg] [Gui.cc:355] GUI requesting list of world names. The server may be busy downloading resources. Please be patient.
[gazebo-1] [GUI] [Dbg] [PathManager.cc:57] Received resource paths.
[gazebo-1] [GUI] [Dbg] [Gui.cc:413] Requesting GUI from [/world/mmr_world/gui/info]...
[gazebo-1] [GUI] [Dbg] [GuiRunner.cc:149] Requesting initial state from [/world/mmr_world/state]...
[gazebo-1] [GUI] [Msg] Loading config [/home/stevejidev/.gz/sim/8/gui.config]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [MinimalScene]
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[gazebo-1] [GUI] [Msg] Added plugin [3D View] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [MinimalScene] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libMinimalScene.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [EntityContextMenuPlugin]
[gazebo-1] [GUI] [Msg] Currently tracking topic on [/gui/currently_tracked]
[gazebo-1] [GUI] [Msg] Added plugin [Entity Context Menu] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [EntityContextMenuPlugin] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libEntityContextMenuPlugin.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [GzSceneManager]
[gazebo-1] [GUI] [Msg] Added plugin [Scene Manager] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [GzSceneManager] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libGzSceneManager.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [InteractiveViewControl]
[gazebo-1] [GUI] [Msg] Camera view controller topic advertised on [/gui/camera/view_control]
[gazebo-1] [GUI] [Msg] Camera reference visual topic advertised on [/gui/camera/view_control/reference_visual]
[gazebo-1] [GUI] [Msg] Camera view control sensitivity advertised on [/gui/camera/view_control/sensitivity]
[gazebo-1] [GUI] [Msg] Added plugin [Interactive view control] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [InteractiveViewControl] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libInteractiveViewControl.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [CameraTracking]
[gazebo-1] [GUI] [Msg] Added plugin [Camera tracking] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [CameraTracking] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libCameraTracking.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [MarkerManager]
[gazebo-1] [GUI] [Msg] Listening to stats on [/world/mmr_world/stats]
[gazebo-1] [GUI] [Msg] Added plugin [Marker Manager] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [MarkerManager] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libMarkerManager.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [SelectEntities]
[gazebo-1] [GUI] [Msg] Added plugin [Select entities] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [SelectEntities] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libSelectEntities.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [Spawn]
[gazebo-1] [GUI] [Msg] Added plugin [Spawn] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [Spawn] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libSpawn.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [VisualizationCapabilities]
[gazebo-1] [GUI] [Msg] View as transparent service on [/gui/view/transparent]
[gazebo-1] [GUI] [Msg] View as wireframes service on [/gui/view/wireframes]
[gazebo-1] [GUI] [Msg] View center of mass service on [/gui/view/com]
[gazebo-1] [GUI] [Msg] View inertia service on [/gui/view/inertia]
[gazebo-1] [GUI] [Msg] View collisions service on [/gui/view/collisions]
[gazebo-1] [GUI] [Msg] View joints service on [/gui/view/joints]
[gazebo-1] [GUI] [Msg] View frames service on [/gui/view/frames]
[gazebo-1] [GUI] [Msg] Added plugin [Visualization capabilities] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [VisualizationCapabilities] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libVisualizationCapabilities.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [WorldControl]
[gazebo-1] [GUI] [Msg] Using world control service [/world/mmr_world/control]
[gazebo-1] [GUI] [Msg] Listening to stats on [/world/mmr_world/stats]
[gazebo-1] [GUI] [Dbg] [WorldControl.cc:237] Using an event to share WorldControl msgs with the server
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:413:17: QML ToolButton: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file::/ComponentInspector/ComponentInspector.qml:251:3: QML Dialog: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:413:17: QML ToolButton: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file:///usr/lib/x86_64-linux-gnu/qt5/qml/QtQuick/Dialogs/DefaultFileDialog.qml:309:21: QML Button: Binding loop detected for property "implicitHeight"
[gazebo-1] [GUI] [Msg] Added plugin [World control] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [WorldControl] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libWorldControl.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [WorldStats]
[gazebo-1] [GUI] [Msg] Listening to stats on [/world/mmr_world/stats]
[gazebo-1] [GUI] [Msg] Added plugin [World stats] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [WorldStats] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libWorldStats.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [Shapes]
[gazebo-1] [GUI] [Msg] Added plugin [Shapes] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [Shapes] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libShapes.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [Lights]
[gazebo-1] [GUI] [Msg] Added plugin [Lights] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [Lights] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libLights.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [TransformControl]
[gazebo-1] [GUI] [Msg] Added plugin [Transform control] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [TransformControl] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libTransformControl.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [Screenshot]
[gazebo-1] [GUI] [Msg] Screenshot service on [/gui/screenshot]
[gazebo-1] [GUI] [Msg] Added plugin [Screenshot] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [Screenshot] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libScreenshot.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [CopyPaste]
[gazebo-1] [GUI] [Msg] Added plugin [Copy/Paste] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [CopyPaste] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libCopyPaste.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [ComponentInspector]
[gazebo-1] [GUI] [Msg] Added plugin [Component inspector] to main window
[gazebo-1] [GUI] [Msg] Loaded plugin [ComponentInspector] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libComponentInspector.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:528] Loading plugin [EntityTree]
[gazebo-1] [GUI] [Msg] Currently tracking topic on [/gui/currently_tracked]
[gazebo-1] [GUI] [Msg] Added plugin [Entity tree] to main window
[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] file::/WorldStats/WorldStats.qml:53:3: QML RowLayout: Binding loop detected for property "x"
[gazebo-1] libEGL warning: pci id for fd 109: 10de:2d04, driver (null)
[gazebo-1] 
[gazebo-1] pci id for fd 110: 10de:2d04, driver (null)
[gazebo-1] pci id for fd 111: 10de:2d04, driver (null)
[gazebo-1] libEGL warning: egl: failed to create dri2 screen
[gazebo-1] libEGL warning: pci id for fd 109: 10de:2d04, driver (null)
[gazebo-1] 
[gazebo-1] pci id for fd 110: 10de:2d04, driver (null)
[gazebo-1] pci id for fd 111: 10de:2d04, driver (null)
[gazebo-1] libEGL warning: egl: failed to create dri2 screen
[gazebo-1] libEGL warning: pci id for fd 109: 10de:2d04, driver (null)
[gazebo-1] 
[gazebo-1] [GUI] [/world/mmr_world/enable_collision/blocking] (blocking)
[gazebo-1] [Msg] Disable collision service on [/world/mmr_world/disable_collision] (async)
[gazebo-1] [Msg] Disable collision service on [/world/mmr_world/disable_collision/blocking] (blocking)
[gazebo-1] [Msg] Material service on [/world/mmr_world/visual_config] (async)
[gazebo-1] [Msg] Material service on [/world/mmr_world/visual_config/blocking] (blocking)
[gazebo-1] [Msg] Material service on [/world/mmr_world/wheel_slip] (async)
[gazebo-1] [Msg] Material service on [/world/mmr_world/wheel_slip/blocking] (blocking)
[gazebo-1] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::UserCommands] for entity [1]
[gazebo-1] [Msg] Loaded level [default]
[gazebo-1] [Msg] Serving world controls on [/world/mmr_world/control], [/world/mmr_world/control/state] and [/world/mmr_world/playback/control]
[gazebo-1] [Msg] Serving GUI information on [/world/mmr_world/gui/info]
[gazebo-1] [Msg] World [mmr_world] initialized with [1ms] physics profile.
[gazebo-1] [Msg] Serving world SDF generation service on [/world/mmr_world/generate_world_sdf]
[gazebo-1] [Msg] Serving world names on [/gazebo/worlds]
[gazebo-1] [Msg] Resource path add service on [/gazebo/resource_paths/add].
[gazebo-1] [Msg] Resource path get service on [/gazebo/resource_paths/get].
[gazebo-1] [Msg] Resource path resolve service on [/gazebo/resource_paths/resolve].
[gazebo-1] [Msg] Resource paths published on [/gazebo/resource_paths].
[gazebo-1] [Msg] Server control service on [/server_control].
[gazebo-1] [Msg] Found no publishers on /stats, adding root stats topic
[gazebo-1] [Msg] Found no publishers on /clock, adding root clock topic
[gazebo-1] [Dbg] [SimulationRunner.cc:551] Creating PostUpdate worker threads: 3
[gazebo-1] [Dbg] [SimulationRunner.cc:562] Creating postupdate worker thread (0)
[gazebo-1] [Dbg] [SimulationRunner.cc:562] Creating postupdate worker thread (1)
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="mmr_bot"]/link[@name="base_footprint"]/sensor[@name="laser"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
[gazebo-1] [INFO] [1788056112.431728897] [gz_ros_control]: Loading controller_manager
[gazebo-1] [INFO] [1788056112.437168464] [controller_manager]: Using ROS clock for triggering controller manager cycles.
[gazebo-1] [INFO] [1788056112.439292869] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[gazebo-1] [WARN] [1788056112.441121150] [gz_ros_control]: Waiting RM to load and initialize hardware...
[gazebo-1] [INFO] [1788056112.943504220] [controller_manager]: Received robot description from topic.
[gazebo-1] [INFO] [1788056112.943546319] [controller_manager]: Enforcing command limits is disabled. Command limits from URDF will be ignored.
[gazebo-1] [INFO] [1788056112.946873032] [gz_ros_control]: The position_proportional_gain has been set to: 0.1
[gazebo-1] [INFO] [1788056112.946904200] [gz_ros_control]: Loading joint: wheel_0_joint
[gazebo-1] [INFO] [1788056112.946910337] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.946915471] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.946919382] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.946925981] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.946929661] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.946932710] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.946942909] [gz_ros_control]: Loading joint: wheel_1_joint
[gazebo-1] [INFO] [1788056112.946946348] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.946949728] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.946955635] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.946959164] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.946962504] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.946965472] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.946972673] [gz_ros_control]: Loading joint: wheel_2_joint
[gazebo-1] [INFO] [1788056112.946975922] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.946978920] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.946981959] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.946985007] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.946988497] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.946991365] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.946998746] [gz_ros_control]: Loading joint: shoulder_pan
[gazebo-1] [INFO] [1788056112.947001895] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.947004763] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947007571] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.947010369] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.947013247] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.947016305] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947022784] [gz_ros_control]: Loading joint: shoulder_lift
[gazebo-1] [INFO] [1788056112.947025802] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.947028580] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947031739] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.947034547] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.947037555] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.947040704] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947047363] [gz_ros_control]: Loading joint: elbow_flex
[gazebo-1] [INFO] [1788056112.947050632] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.947053500] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947056298] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.947060069] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.947063258] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.947066026] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947071962] [gz_ros_control]: Loading joint: wrist_flex
[gazebo-1] [INFO] [1788056112.947075021] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.947077789] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947080637] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.947085380] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.947088308] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.947091126] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947096471] [gz_ros_control]: Loading joint: wrist_roll
[gazebo-1] [INFO] [1788056112.947099600] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.947102328] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947105366] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.947108124] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.947110952] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.947113870] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947135421] [gz_ros_control]: Loading joint: gripper
[gazebo-1] [INFO] [1788056112.947138690] [gz_ros_control]: 	State:
[gazebo-1] [INFO] [1788056112.947141568] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947144416] [gz_ros_control]: 		 velocity
[gazebo-1] [INFO] [1788056112.947147305] [gz_ros_control]: 		 effort
[gazebo-1] [INFO] [1788056112.947150213] [gz_ros_control]: 	Command:
[gazebo-1] [INFO] [1788056112.947153041] [gz_ros_control]: 		 position
[gazebo-1] [INFO] [1788056112.947243325] [controller_manager]: Initialize hardware 'mmr_bot_system' 
[gazebo-1] [WARN] [1788056112.947257696] [controller_manager.hardware_component.system.mmr_bot_system]: Executor is not available during hardware component initialization for 'mmr_bot_system'. Skipping node creation!
[gazebo-1] [INFO] [1788056112.947307837] [controller_manager]: Successful initialization of hardware 'mmr_bot_system'
[gazebo-1] [INFO] [1788056112.947413926] [controller_manager]: Activating component 'mmr_bot_system'.
[gazebo-1] [INFO] [1788056112.947424576] [resource_manager]: 'configure' hardware 'mmr_bot_system' 
[gazebo-1] [INFO] [1788056112.947428577] [gz_ros_control]: System Successfully configured!
[gazebo-1] [INFO] [1788056112.947432638] [resource_manager]: Successful 'configure' of hardware 'mmr_bot_system'
[gazebo-1] [INFO] [1788056112.947439959] [resource_manager]: 'activate' hardware 'mmr_bot_system' 
[gazebo-1] [INFO] [1788056112.947452194] [resource_manager]: Successful 'activate' of hardware 'mmr_bot_system'
[gazebo-1] [WARN] [1788056112.947507991] [controller_manager]: Component 'mmr_bot_system' does not have read or write statistics initialized, skipping registration.
[gazebo-1] [INFO] [1788056112.947513757] [controller_manager]: Resource Manager has been successfully initialized. Starting Controller Manager services...
[spawner-8] [INFO] [1788056113.112888068] [spawner_arm_position_controller]: Setting controller param "params_file" to "['/tmp/launch_params_80d4p571']" for arm_position_controller
[gazebo-1] [INFO] [1788056113.113760948] [controller_manager]: Loading controller : 'arm_position_controller' of type 'position_controllers/JointGroupPositionController'
[gazebo-1] [INFO] [1788056113.113797008] [controller_manager]: Loading controller 'arm_position_controller'
[gazebo-1] [INFO] [1788056113.115507672] [controller_manager]: Controller 'arm_position_controller' node arguments: --ros-args --params-file /home/stevejidev/dev_ws/install/mmr_pkg/share/mmr_pkg/config/controllers.yaml -p use_sim_time:=true --params-file /tmp/launch_params_80d4p571 --param use_sim_time:=true 
[spawner-8] [INFO] [1788056113.122273406] [spawner_arm_position_controller]: Loaded arm_position_controller
[gazebo-1] [INFO] [1788056113.122792640] [controller_manager]: Configuring controller: 'arm_position_controller'
[gazebo-1] [INFO] [1788056113.123291107] [arm_position_controller]: configure successful
[gazebo-1] [INFO] [1788056113.123991832] [controller_manager]: Activating controllers: [ arm_position_controller ]
[gazebo-1] [Dbg] [SystemManager.cc:80] Loaded system [gz_ros2_control::GazeboSimROS2ControlPlugin] for entity [10]
[gazebo-1] [Dbg] [UserCommands.cc:1094] Created entity [10] named [mmr_bot]
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__base_link_collision_2] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__base_link_collision_3] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__base_link_collision_4] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__base_link_collision_5] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__base_link_collision_6] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__arm_base_link_collision_7] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__arm_base_link_collision_8] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__arm_base_link_collision_9] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__arm_base_link_collision_10] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [base_footprint_fixed_joint_lump__laser_frame_collision_11] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_shoulder_link_collision] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_shoulder_link_collision_1] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_shoulder_link_collision_2] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_upper_arm_link_collision] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_upper_arm_link_collision_1] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_lower_arm_link_collision] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_lower_arm_link_collision_1] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_lower_arm_link_collision_2] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_wrist_link_collision] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_wrist_link_collision_1] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_gripper_link_collision] couldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/motor_pod.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/motor_pod.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/motor_pod.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/motor_pod.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/camera_pod.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/camera_pod.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/camera_pod.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/camera_pod.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/pi_stack.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/pi_stack.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/pi_stack.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/pi_stack.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/base_motor_holder_so101_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/base_so101_v2.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/base_so101_v2.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/base_so101_v2.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/base_so101_v2.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/waveshare_mounting_plate_so101_v2.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/rplidar_c1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/rplidar_c1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/rplidar_c1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/rplidar_c1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/motor_holder_so101_base_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/rotation_pitch_so101_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/upper_arm_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/upper_arm_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/upper_arm_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/upper_arm_so101_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/under_arm_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/under_arm_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/under_arm_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/under_arm_so101_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/motor_holder_so101_wrist_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/sts3215_03a_no_horn_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/wrist_roll_pitch_so101_v2.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/wrist_roll_follower_so101_v1.stl].
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/collision/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/collision/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/collision/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/collision/arm/moving_jaw_so101_v1.stl].
[gazebo-1] [WARN] [1788056114.466108205] [gz_ros_control]:  Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.001 s).
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_gripper_link_collision_1] co [Msg] Loaded plugin [EntityTree] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libEntityTree.so]
[gazebo-1] [GUI] [Dbg] [Application.cc:398] Loading window config
[gazebo-1] [GUI] [Msg] Using server control service [/server_control]
[gazebo-1] [GUI] [Dbg] [Application.cc:671] Applying config
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[gazebo-1] [GUI] [Msg] Loading plugin [gz-rendering-ogre2]
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:749] Create scene [scene]
[gazebo-1] [GUI] [Dbg] [MinimalScene.cc:1037] Creating texture node render interface for OpenGL
[gazebo-1] [GUI] [Dbg] [TransformControl.cc:453] TransformControl plugin is using camera [scene::Camera(65527)]
[gazebo-1] [GUI] [Dbg] [Spawn.cc:308] Spawn plugin is using camera [scene::Camera(65527)]
[gazebo-1] [GUI] [Dbg] [SelectEntities.cc:452] SelectEntities plugin is using camera [scene::Camera(65527)]
[gazebo-1] [GUI] [Dbg] [MarkerManager.cc:169] Advertise /marker/list service.
[gazebo-1] [GUI] [Dbg] [MarkerManager.cc:179] Advertise /marker/list.
[gazebo-1] [GUI] [Dbg] [MarkerManager.cc:189] Advertise /marker_array.
[gazebo-1] [GUI] [Dbg] [CameraTracking.cc:205] CameraTrackingPrivate plugin is moving camera [scene::Camera(65527)]
[gazebo-1] [GUI] [Msg] Move to service on [/gui/move_to]
[gazebo-1] [GUI] [Msg] Follow service on [/gui/follow] (deprecated)
[gazebo-1] [GUI] [Msg] Tracking topic on [/gui/track]
[gazebo-1] [GUI] [Msg] Tracking status topic on [/gui/currently_tracked]
[gazebo-1] [GUI] [Msg] Move to pose service on [/gui/move_to/pose]
[gazebo-1] [GUI] [Msg] Camera pose topic advertised on [/gui/camera/pose]
[gazebo-1] [GUI] [Msg] Follow offset service on [/gui/follow/offset] (deprecated)
[gazebo-1] [GUI] [Dbg] [InteractiveViewControl.cc:176] InteractiveViewControl plugin is moving camera [scene::Camera(65527)]
[gazebo-1] [GUI] [Dbg] [EntityContextMenuPlugin.cc:79] Entity context menu plugin is using camera [scene::Camera(65527)]
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="mmr_bot"]/link[@name="base_footprint"]/sensor[@name="laser"]/gz_frame_id:<data-string>:L236]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/base_link.obj]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/base_link.obj]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/base_link.obj]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/base_link.obj].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__base_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_1
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/base_so101_v2.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/base_so101_v2.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/base_so101_v2.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/base_so101_v2.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_2
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_3
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_4
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/rplidar_c1.obj]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/rplidar_c1.obj]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/rplidar_c1.obj]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/rplidar_c1.obj].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__laser_frame_visual_5
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_shoulder_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_shoulder_link_visual_1
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_shoulder_link_visual_2
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_upper_arm_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_upper_arm_link_visual_1
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_lower_arm_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_lower_arm_link_visual_1
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_lower_arm_link_visual_2
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_wrist_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_wrist_link_visual_1
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_gripper_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_gripper_link_visual_1
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_moving_jaw_so101_v1_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/wheel.obj].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: wheel_0_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/wheel.obj].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: wheel_1_link_visual
[gazebo-1] [GUI] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [GUI] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/wheel.obj].
[gazebo-1] [GUI] [Err] [SceneManager.cc:426] Failed to load geometry for visual: wheel_2_link_visual
[gazebo-1] libEGL warning: pci id for fd 99: 10de:2d04, driver (null)
[gazebo-1] 
[gazebo-1] pci id for fd 100: 10de:2d04, driver (null)
[gazebo-1] pci id for fd 101: 10de:2d04, driver (null)
[gazebo-1] libEGL warning: egl: failed to create dri2 screen
[gazebo-1] libEGL warning: pci id for fd 99: 10de:2d04, driver (null)
[gazebo-1] 
[gazebo-1] pci id for fd 100: 10de:2d04, driver (null)
[gazebo-1] pci id for fd 101: 10de:2d04, driver (null)
[gazebo-1] libEGL warning: egl: failed to create dri2 screen
[gazebo-1] libEGL warning: pci id for fd 99: 10de:2d04, driver (null)
[gazebo-1] 
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/base_link.obj]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/base_link.obj]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/base_link.obj]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/base_link.obj].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__base_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/base_motor_holder_so101_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_1
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/base_so101_v2.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/base_so101_v2.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/base_so101_v2.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/base_so101_v2.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_2
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_3
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/waveshare_mounting_plate_so101_v2.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__arm_base_link_visual_4
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/rplidar_c1.obj]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/rplidar_c1.obj]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/rplidar_c1.obj]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/rplidar_c1.obj].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: base_footprint_fixed_joint_lump__laser_frame_visual_5
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_shoulder_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/motor_holder_so101_base_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_shoulder_link_visual_1
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/rotation_pitch_so101_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_shoulder_link_visual_2
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_upper_arm_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/upper_arm_so101_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_upper_arm_link_visual_1
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/under_arm_so101_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_lower_arm_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/motor_holder_so101_wrist_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_lower_arm_link_visual_1
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_lower_arm_link_visual_2
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_no_horn_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_wrist_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/wrist_roll_pitch_so101_v2.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_wrist_link_visual_1
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/sts3215_03a_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_gripper_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/wrist_roll_follower_so101_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_gripper_link_visual_1
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/arm/moving_jaw_so101_v1.stl].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: arm_moving_jaw_so101_v1_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/wheel.obj].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: wheel_0_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/wheel.obj].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: wheel_1_link_visual
[gazebo-1] [Err] [SystemPaths.cc:426] Unable to find file with URI [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Err] [SystemPaths.cc:526] Could not resolve file [model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Err] [MeshManager.cc:211] Unable to find file[model://mmr_pkg/meshes/visual/wheel.obj]
[gazebo-1] [Wrn] [Util.cc:859] Failed to load mesh from [model://mmr_pkg/meshes/visual/wheel.obj].
[gazebo-1] [Err] [SceneManager.cc:426] Failed to load geometry for visual: wheel_2_link_visual
[gazebo-1] [INFO] [1788056114.891989582] [arm_position_controller]: activate successful
[gazebo-1] [INFO] [1788056114.892041894] [controller_manager]: Successfully switched controllers!
[spawner-8] [INFO] [1788056114.902603584] [spawner_arm_position_controller]: Configured and activated arm_position_controller
[spawner-6] [INFO] [1788056114.983860233] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[INFO] [spawner-8]: process has finished cleanly [pid 6246]
[spawner-6] [INFO] [1788056115.237404988] [spawner_joint_state_broadcaster]: Setting controller param "params_file" to "['/tmp/launch_params_lw7qgcg9']" for joint_state_broadcaster
[gazebo-1] [INFO] [1788056115.238209411] [controller_manager]: Loading controller : 'joint_state_broadcaster' of type 'joint_state_broadcaster/JointStateBroadcaster'
[gazebo-1] [INFO] [1788056115.238236896] [controller_manager]: Loading controller 'joint_state_broadcaster'
[gazebo-1] [INFO] [1788056115.241465752] [controller_manager]: Controller 'joint_state_broadcaster' node arguments: --ros-args --params-file /home/stevejidev/dev_ws/install/mmr_pkg/share/mmr_pkg/config/controllers.yaml -p use_sim_time:=true --params-file /tmp/launch_params_lw7qgcg9 --param use_sim_time:=true 
[spawner-6] [INFO] [1788056115.252550430] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[gazebo-1] [INFO] [1788056115.253046222] [controller_manager]: Configuring controller: 'joint_state_broadcaster'
[gazebo-1] [INFO] [1788056115.253087025] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[gazebo-1] [INFO] [1788056115.262664148] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]
[gazebo-1] [INFO] [1788056115.271878105] [controller_manager]: Successfully switched controllers!
[spawner-6] [INFO] [1788056115.282561340] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner-7] [INFO] [1788056115.376243649] [spawner_wheel_velocity_controller]: waiting for service /controller_manager/list_controllers to become available...
[INFO] [spawner-6]: process has finished cleanly [pid 6244]
[spawner-7] [INFO] [1788056116.129710146] [spawner_wheel_velocity_controller]: Setting controller param "params_file" to "['/tmp/launch_params_o_0h2z19']" for wheel_velocity_controller
[gazebo-1] [INFO] [1788056116.130512177] [controller_manager]: Loading controller : 'wheel_velocity_controller' of type 'velocity_controllers/JointGroupVelocityController'
[gazebo-1] [INFO] [1788056116.130538796] [controller_manager]: Loading controller 'wheel_velocity_controller'
[gazebo-1] [INFO] [1788056116.131779128] [controller_manager]: Controller 'wheel_velocity_controller' node arguments: --ros-args --params-file /home/stevejidev/dev_ws/install/mmr_pkg/share/mmr_pkg/config/controllers.yaml -p use_sim_time:=true --params-file /tmp/launch_params_o_0h2z19 --param use_sim_time:=true 
[spawner-7] [INFO] [1788056116.142661127] [spawner_wheel_velocity_controller]: Loaded wheel_velocity_controller
[gazebo-1] [INFO] [1788056116.143232808] [controller_manager]: Configuring controller: 'wheel_velocity_controller'
[gazebo-1] [INFO] [1788056116.143578338] [wheel_velocity_controller]: configure successful
[gazebo-1] [INFO] [1788056116.152707923] [controller_manager]: Activating controllers: [ wheel_velocity_controller ]
[gazebo-1] [INFO] [1788056116.161851691] [wheel_velocity_controller]: activate successful
[gazebo-1] [INFO] [1788056116.161881279] [controller_manager]: Successfully switched controllers!
[spawner-7] [INFO] [1788056116.172642485] [spawner_wheel_velocity_controller]: Configured and activated wheel_velocity_controller
[INFO] [spawner-7]: process has finished cleanly [pid 6245]
[gazebo-1] uldn't be created
[gazebo-1] [Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[gazebo-1] [Dbg] [SDFFeatures.cc:864] The geometry element of collision [arm_moving_jaw_so101_v1_link_collision] couldn't be created
[gazebo-1] [Dbg] [Sensors.cc:953] Initialization needed
[gazebo-1] [Dbg] [Sensors.cc:349] Initializing render context
[gazebo-1] [Msg] Loading plugin [gz-rendering-ogre2]
[gazebo-1] [Msg] Serving scene information on [/world/mmr_world/scene/info]
[gazebo-1] [Msg] Serving graph information on [/world/mmr_world/scene/graph]
[gazebo-1] [Msg] Serving full state on [/world/mmr_world/state]
[gazebo-1] [Msg] Serving full state (async) on [/world/mmr_world/state_async]
[gazebo-1] [Msg] Publishing scene information on [/world/mmr_world/scene/info]
[gazebo-1] [Msg] Publishing entity deletions on [/world/mmr_world/scene/deletion]
[gazebo-1] [Msg] Publishing state changes on [/world/mmr_world/state]
[gazebo-1] [Msg] Publishing pose messages on [/world/mmr_world/pose/info]
[gazebo-1] [Msg] Publishing dynamic pose messages on [/world/mmr_world/dynamic_pose/info]
[gazebo-1] [Dbg] [EntityComponentManager.cc:1656] Updated state thread iterators: 12 threads processing around 7 entities each.
[gazebo-1] [Dbg] [SimulationRunner.cc:578] Exiting postupdate worker thread (1)
[gazebo-1] [Dbg] [SimulationRunner.cc:578] Exiting postupdate worker thread (0)
[gazebo-1] [Dbg] [SimulationRunner.cc:551] Creating PostUpdate worker threads: 4
[gazebo-1] [Dbg] [SimulationRunner.cc:562] Creating postupdate worker thread (0)
[gazebo-1] [Dbg] [SimulationRunner.cc:562] Creating postupdate worker thread (1)
[gazebo-1] [Dbg] [SimulationRunner.cc:562] Creating postupdate worker thread (2)
[gazebo-1] [Dbg] [RenderUtil.cc:2646] Create scene [scene]
[gazebo-1] [Dbg] [Sensors.cc:391] Rendering Thread initialized
[gazebo-1] Initialization needed
[gazebo-1] [Dbg] [Lidar.cc:139] Laser scans for [mmr_bot::base_footprint::laser] advertised on [scan]
[gazebo-1] [Dbg] [GpuLidarSensor.cc:164] Lidar points for [mmr_bot::base_footprint::laser] advertised on [scan/points]

# mmr_pkg — `mmr_bot` description (Phase 1)

URDF/xacro description of a three-wheeled kiwi-drive holonomic base carrying an
SO-101 arm, a Raspberry Pi 5 and an RPLIDAR C1. Geometry is derived from the
Fusion 360 STEP export (`mmr_bot.step`); the arm is vendored from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100).

**Phase 1 is RViz only.** No Gazebo, no `ros2_control`, no controllers. Phase 2
starts only once you have confirmed this looks right.

> **Read the [Verification status](#verification-status) section before trusting
> anything here.** This package was built on a Windows machine with no ROS 2
> install. `xacro`, `check_urdf`, `rviz2` and `colcon` were **never run**. The
> checks that *were* run are offline reimplementations, listed below.

---

## Build and run

```bash
cd ~/ros2_ws/src
cp -r /path/to/mmr_pkg .
cd ~/ros2_ws
colcon build --packages-select mmr_pkg
source install/setup.bash

ros2 launch mmr_pkg display.launch.py
```

Arguments: `gui:=false` swaps the slider GUI for a plain `joint_state_publisher`;
`model:=` and `rviz_config:=` override the paths.

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

1. **`laser_frame` z = 0.105180 m is approximate.** It is the *mid-height of the
   turret shell* (CAD z 137.000…163.241 mm). The CAD body is a solid shell with
   no modelled optics, so the true scan plane is not derivable from this STEP.
   x and y are solid (rotor axis located from the turret geometry: 133.781 mm
   forward, 0.004 mm off centre). Override `laser_z` in `sensors.xacro` with the
   datasheet value if you want it exact.
2. **No inertials on the chassis.** You said you would paste Fusion 360 values.
   Until then `use_inertial` in `base.xacro` is `false` and **no `<inertial>` is
   emitted for `base_link`, the wheels or `laser_frame`** — per the brief, no
   placeholder identity tensors anywhere. `urdf/inertial_macros.xacro` ships
   working macros ready to receive them, including `inertial_from_fusion`, which
   handles both unit conversions for you: Fusion reports kg·mm² (divide by 1e6)
   and uses the **opposite sign convention for products of inertia** (Ixy_urdf =
   −Ixy_fusion). The arm's inertials are real, from upstream, and are present.
3. **A camera is modelled although the brief says there is none.** An Innomaker
   U20CAM-1080P on a `BaseCamMount`, facing forward. It is currently merged into
   the `base_link` visual, with its hull in `camera_pod.stl`. Say the word and it
   can be split into a proper `camera_link` + optical frame — the mount transform
   is measurable from the same STEP.
4. **`arm_gripper_frame_link` has a zero inertia tensor** (mass 1e-9). That is
   upstream's dummy frame, kept verbatim. Harmless for RViz; may want a tiny
   non-zero diagonal in Phase 2.
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

---

## Corrections to the brief

The brief asked to be told when a supplied number disagreed with the STEP.
Three did.

| Brief said | STEP says | Resolution |
|---|---|---|
| omniwheel "60 mm — confirm RADIUS or DIAMETER" | max radius **30.000 mm** on all three; part is named `RodaOmni 60 mm` | 60 mm is a **diameter**; radius 0.030 m |
| base plate "200 mm — round or square?" | **round**, Ø **240.002 mm** (roundness ratio 1.0001; a square would give 1.414) | plate radius 0.120 m — the 200 mm figure is wrong |
| wheel axle height 27 mm | axle sits **30.000 mm** above the wheel-circle tangent, exactly the wheel radius | 27 mm is wrong; 30 mm is used, and the xacro derives it so it cannot disagree with the radius |

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

Run `python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro -o build/robot.urdf`
then `python tools/check_urdf_lite.py build/robot.urdf`. Current result —
**0 failures, 2 expected warnings**:

- All five xacro files are well-formed XML and expand cleanly (27 properties,
  7 macros, 4 includes).
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

The two warnings are the documented gaps: `arm_gripper_frame_link`'s zero
inertia, and the six links with no inertial pending your Fusion numbers.

### NOT checked here — please run these

```bash
xacro urdf/mmr_bot.urdf.xacro > /tmp/robot.urdf          # must run clean
check_urdf /tmp/robot.urdf                               # 1 tree, no orphans, 9 movable
ros2 launch mmr_pkg display.launch.py                    # 0 missing-mesh warnings, 0 TF errors
ros2 run tf2_tools view_frames                           # connected tree
```

Then, by eye in RViz:

- all three wheels touch the ground plane;
- each `joint_state_publisher_gui` slider moves the correct link about the
  correct axis;
- the part colours came through on `base_link`.

`xacro_lite.py` implements only the subset of xacro this package uses. If real
`xacro` disagrees with it, real `xacro` is right — tell me and I will fix the
description.

---

## Phase 2 — not started

Gazebo Harmonic, `ros2_control`, the `kiwi_drive_node` and the lidar bridge all
wait for your confirmation that the above looks right in RViz.

---

## Repository layout

```
mmr_pkg/
├── package.xml, CMakeLists.txt
├── urdf/       mmr_bot.urdf.xacro, base.xacro, sensors.xacro, arm.xacro,
│               inertial_macros.xacro
├── meshes/     visual/ (+ .mtl), collision/, arm/, collision/arm/
├── launch/     display.launch.py
├── rviz/       display.rviz
└── tools/      STEP parsing, measurement and mesh extraction scripts, plus the
                offline xacro/URDF checkers. Not installed by CMake — these are
                provenance for every number above, not runtime code.
```

"""Estimate CoM and inertia tensors from the real collision geometry.

WHY THIS EXISTS
---------------
Gazebo will not simulate a link sensibly without an <inertial>. Phase 1
deliberately emitted none, because the brief forbids placeholder tensors and
the Fusion 360 mass properties were never supplied. Phase 2 needs *something*,
so this computes the honest middle option:

    SHAPE  of the tensor  <- the actual geometry, exactly
    SCALE  of the tensor  <- a mass that must come from outside

WHAT IS INTEGRATED, AND WHY NOT THE VISUAL MESH
-----------------------------------------------
The visual meshes are decimated CAD exports and are NOT watertight, so volume
and inertia integrals over them are meaningless (trimesh will happily return a
number anyway -- that is the trap). Instead base_link is built as a COMPOSITE
of the collision shapes that were already justified in Phase 1:

    base plate cylinder, top plate cylinder, 3x motor pod hull,
    camera pod hull, Pi stack hull

Each piece is watertight, its inertia is computed about its own centroid, and
the pieces are combined with the parallel-axis theorem. Mass is distributed
between them in proportion to volume -- i.e. one uniform density across the
whole chassis. That is the assumption, stated plainly: it is wrong in detail
(the Pi stack is denser than the plates) but it puts the mass in roughly the
right places, which a bounding box or a lumped point mass does not.

The wheels are not integrated at all: their collision IS a cylinder primitive,
so the closed-form cylinder tensor is exact for the modelled shape and
urdf/inertial_macros.xacro already has `inertial_cylinder_x`.

This is NOT a Fusion readout and does not pretend to be.

MASS PROVENANCE
---------------
  base_link    1.0 kg    GIVEN IN THE BRIEF (section 3.1), chassis only
  laser_frame  0.110 kg  Slamtec RPLIDAR C1 datasheet rev 1.2, Fig 2-9
  wheel_*      ???       NOT SOURCED. See the note printed at the end.

    python tools/estimate_inertia.py
"""
import math

import numpy as np
import trimesh

BASE_RADIUS = 0.135500
WHEEL_ANGLES = (60.0, 180.0, 300.0)


def cylinder(radius, length, transform):
    return trimesh.creation.cylinder(radius=radius, height=length,
                                     transform=transform)


def Rz(deg):
    T = np.eye(4)
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return T


def shift(x=0.0, y=0.0, z=0.0):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def hull(path, transform=None):
    m = trimesh.load(path, force="mesh")
    m.merge_vertices()
    m = m.convex_hull if not m.is_watertight else m
    if transform is not None:
        m.apply_transform(transform)
    return m


# --- base_link, exactly the collision set from urdf/base.xacro -------------
pieces = [
    ("base plate",  cylinder(0.120, 0.012, shift(z=0.006))),
    ("top plate",   cylinder(0.120, 0.005, shift(z=0.0745))),
    ("camera pod",  hull("meshes/collision/camera_pod.stl")),
    ("pi stack",    hull("meshes/collision/pi_stack.stl")),
]
for a in WHEEL_ANGLES:
    pieces.append((f"motor pod {a:.0f}",
                   hull("meshes/collision/motor_pod.stl", Rz(a))))


def composite(pieces, total_mass):
    """Combine watertight pieces at one uniform density (parallel-axis)."""
    vols = np.array([abs(p.volume) for _, p in pieces])
    masses = total_mass * vols / vols.sum()
    coms = np.array([p.center_mass for _, p in pieces])
    com = (masses[:, None] * coms).sum(axis=0) / masses.sum()
    I = np.zeros((3, 3))
    for (nm, p), m in zip(pieces, masses):
        p.density = m / abs(p.volume)          # scale to this piece's mass
        d = p.center_mass - com
        I += p.moment_inertia
        I += m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return com, I, masses, vols


print("=== base_link composite ===")
print(f"{'piece':<16s} {'vol cm3':>9s} {'mass g':>8s}  centroid mm")
com_b, I_b, masses, vols = composite(pieces, 1.0)
for (nm, p), m, v in zip(pieces, masses, vols):
    c = p.center_mass * 1000
    print(f"{nm:<16s} {v*1e6:9.2f} {m*1000:8.1f}  "
          f"({c[0]:+7.2f} {c[1]:+7.2f} {c[2]:+7.2f})")
print(f"{'TOTAL':<16s} {vols.sum()*1e6:9.2f} {masses.sum()*1000:8.1f}  "
      f"({com_b[0]*1000:+7.2f} {com_b[1]*1000:+7.2f} {com_b[2]*1000:+7.2f})")

# --- laser_frame ----------------------------------------------------------
lidar = hull("meshes/collision/rplidar_c1.stl")
lidar.density = 0.110 / abs(lidar.volume)
com_l, I_l = lidar.center_mass, lidar.moment_inertia

# --- wheel: closed form for the modelled cylinder, spin axis = local +X ----
WR, WW, WM = 0.030, 0.019, 0.100
I_w = np.diag([0.5 * WM * WR**2,                                  # about +X
               WM * (3 * WR**2 + WW**2) / 12.0,
               WM * (3 * WR**2 + WW**2) / 12.0])

print("\n" + "=" * 74)
print("xacro, ready to paste. Tensors are about each link's own CoM,")
print("expressed in that link's frame, which is what <inertial> wants.")
print("=" * 74)


def emit(tag, mass, com, I, provenance):
    print(f"\n<!-- {tag}: {provenance} -->")
    print(f'<xacro:inertial_exact mass="{mass}"')
    print(f'    ox="{com[0]:.6f}" oy="{com[1]:.6f}" oz="{com[2]:.6f}"')
    print(f'    ixx="{I[0,0]:.9f}" iyy="{I[1,1]:.9f}" izz="{I[2,2]:.9f}"')
    print(f'    ixy="{I[0,1]:.9f}" ixz="{I[0,2]:.9f}" iyz="{I[1,2]:.9f}"/>')


emit("base_link", 1.0, com_b, I_b,
     "brief section 3.1 mass 1.0 kg; uniform density over the Phase 1 "
     "collision set")
emit("laser_frame", 0.110, com_l, I_l,
     "RPLIDAR C1 datasheet rev 1.2 Fig 2-9, 110 g typical; uniform density "
     "over the convex hull")
print(f"\n<!-- wheel_*_link: cylinder r={WR} l={WW} about local +X. "
      f"MASS NOT SOURCED. -->")
print(f'<xacro:inertial_cylinder_x mass="{WM}" radius="{WR}" length="{WW}"/>')
print(f'<!--   would give ixx={I_w[0,0]:.9f} iyy={I_w[1,1]:.9f} '
      f'izz={I_w[2,2]:.9f} -->')

# ------------------------------------------------------------------ sanity
print("\n" + "=" * 74)
print("sanity checks  (no rigid body can violate the triangle inequality)")
print("=" * 74)
for nm, mass, I in (("base_link", 1.0, I_b), ("laser_frame", 0.110, I_l),
                    ("wheel", WM, I_w)):
    w = np.linalg.eigvalsh(I)
    a, b, c = sorted(w)
    ok = (a + b >= c - 1e-12) and a > 0
    # radius of gyration is the intuitive check: is the mass spread over a
    # distance that looks like the part?
    rg = math.sqrt(max(w) / mass)
    print(f"  {nm:<12s} principal {np.round(w, 9)}  "
          f"{'OK' if ok else '*** VIOLATED ***'}   r_gyration {rg*1000:.1f} mm")

print("""
======================================================================
WHEEL MASS IS THE ONE NUMBER IN THIS PACKAGE WITH NO SOURCE.
======================================================================
The 0.100 kg above is a working value so the simulation runs. It is NOT
measured and NOT from a datasheet, and it is the only such number in the
package. Put a wheel on a kitchen scale -- 1 g resolution is plenty -- and set
wheel_mass in urdf/base.xacro.

Wheel inertia sets how hard the drive has to work to spin the wheels up, so a
wrong value makes the simulated robot accelerate wrongly in a way that is easy
to misdiagnose as a controller tuning problem.
""")

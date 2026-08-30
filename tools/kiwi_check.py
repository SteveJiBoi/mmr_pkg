"""Verify the kiwi drive sign convention against the actual URDF geometry.

Phase 1 only claims a CONVENTION; the drive node itself is Phase 2. But the
convention has to be stated correctly in the README, so it is derived here from
the joint origins and axes that are really in generated/robot.urdf, rather than
asserted from memory.

Convention under test
---------------------
Wheel i sits at p_i = L (cos a_i, sin a_i, 0) with spin axis u_i = (cos a_i,
sin a_i, 0) pointing radially OUTWARD, contact patch at -R z^.

A material point of the wheel at the contact patch moves at
    v_mat = w u_i x (-R z^) = w R (-sin a_i, cos a_i, 0)
Rolling without slip means the ground-relative velocity of that point is zero,
so the wheel drives the chassis along
    d_i = -v_mat / (w R) = (sin a_i, -cos a_i, 0)      i.e. bearing a_i - 90 deg

Body twist (vx, vy, wz) demands contact velocity v_i = v + wz z^ x p_i, whose
component along d_i must equal w_i R:
    w_i = ( vx sin a_i - vy cos a_i - L wz ) / R
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

URDF = "generated/robot.urdf"
R = 0.030
L = 0.135500

root = ET.parse(URDF).getroot()
J = {j.get("name"): j for j in root.findall("joint")}

angles, radii = [], []
print("--- read back from the URDF ---")
for i in range(3):
    j = J[f"wheel_{i}_joint"]
    o = j.find("origin")
    p = np.array([float(v) for v in o.get("xyz").split()])
    rpy = [float(v) for v in o.get("rpy").split()]
    ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
    cz, sz = math.cos(rpy[2]), math.sin(rpy[2])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    u = Rz @ ax                                   # spin axis in base_link
    a = math.degrees(math.atan2(p[1], p[0])) % 360
    angles.append(a)
    radii.append(math.hypot(p[0], p[1]))
    assert np.allclose(u, [math.cos(math.radians(a)), math.sin(math.radians(a)), 0],
                       atol=1e-9), f"wheel {i} axis is not radially outward"
    print(f"  wheel_{i}: a={a:7.3f} deg  L={radii[-1]:.6f} m  "
          f"u={np.round(u, 6)}  (radially outward OK)")

assert max(radii) - min(radii) < 1e-9, "wheel radii differ"
assert abs(radii[0] - L) < 1e-9, f"L mismatch: {radii[0]} vs {L}"

A = [math.radians(a) for a in angles]


def ik(vx, vy, wz):
    """Body twist -> wheel angular velocities (rad/s)."""
    return np.array([(vx * math.sin(a) - vy * math.cos(a) - L * wz) / R
                     for a in A])


def fk(w):
    """Wheel angular velocities -> body twist, least squares."""
    M = np.array([[math.sin(a) / R, -math.cos(a) / R, -L / R] for a in A])
    return np.linalg.lstsq(M, np.asarray(w), rcond=None)[0]


print("\n--- FK/IK round trip ---")
rng = np.random.default_rng(0)
worst = 0.0
for _ in range(2000):
    t = rng.uniform(-1, 1, 3)
    worst = max(worst, float(np.abs(fk(ik(*t)) - t).max()))
print(f"  worst round-trip error over 2000 random twists: {worst:.3e}")
assert worst < 1e-12

print("\n--- what a POSITIVE wheel velocity does ---")
for i, a in enumerate(A):
    d = np.array([math.sin(a), -math.cos(a)])
    print(f"  wheel_{i} (+1 rad/s alone) drives the contact patch toward "
          f"bearing {math.degrees(math.atan2(d[1], d[0])) % 360:7.3f} deg "
          f"= mounting angle {angles[i]:.0f} - 90")

print("\n--- named cases ---")
cases = [("forward   +X  1 m/s", (1, 0, 0)),
         ("left      +Y  1 m/s", (0, 1, 0)),
         ("yaw CCW  +Z  1 rad/s", (0, 0, 1)),
         ("yaw CW   -Z  1 rad/s", (0, 0, -1))]
for label, t in cases:
    print(f"  {label:<22s} -> w = {np.round(ik(*t), 4)}")

allpos = fk(np.array([1.0, 1.0, 1.0]))
print(f"\n  all three wheels at +1 rad/s -> twist "
      f"(vx={allpos[0]:+.6f}, vy={allpos[1]:+.6f}, wz={allpos[2]:+.6f})")
assert abs(allpos[0]) < 1e-12 and abs(allpos[1]) < 1e-12
print(f"  => pure rotation, wz is NEGATIVE, i.e. all-positive = CLOCKWISE "
      f"seen from above ({-R/L:.6f} rad/s per rad/s of wheel)")
print("\nOK  convention verified against the URDF")

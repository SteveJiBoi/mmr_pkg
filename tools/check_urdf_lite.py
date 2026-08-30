"""Offline structural checks on an expanded URDF.

Stands in for `check_urdf` / RViz on a machine with no ROS 2. Verifies what is
verifiable without a ROS install: tree shape, joint inventory, mesh resolution,
frame placement and the wheel-contact invariant. Anything needing a running
ROS graph (TF echo, RViz warnings) is out of scope and stays unverified.

    python tools/check_urdf_lite.py generated/robot.urdf
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

EXPECTED_MOVABLE = {
    "wheel_0_joint", "wheel_1_joint", "wheel_2_joint",
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
}

fails, warns = [], []


def bad(msg):
    fails.append(msg)
    print(f"  FAIL  {msg}")


def warn(msg):
    warns.append(msg)
    print(f"  WARN  {msg}")


def ok(msg):
    print(f"  ok    {msg}")


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def origin_of(j):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None
                              else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None
                              else "0 0 0").split()]
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--pkg-root", default=".")
    ap.add_argument("--pkg-name", default="mmr_pkg")
    a = ap.parse_args()

    root = ET.parse(a.urdf).getroot()
    links = root.findall("link")
    joints = root.findall("joint")
    lnames = [l.get("name") for l in links]
    jnames = [j.get("name") for j in joints]

    print(f"\n=== {a.urdf}  robot name '{root.get('name')}' ===")
    print(f"    {len(links)} links, {len(joints)} joints\n")

    print("--- names ---")
    for what, names in (("link", lnames), ("joint", jnames)):
        dupes = {n for n in names if names.count(n) > 1}
        if dupes:
            bad(f"duplicate {what} names: {sorted(dupes)}")
        else:
            ok(f"no duplicate {what} names ({len(names)})")

    # ---------------------------------------------------------------- tree
    print("\n--- tree ---")
    child_of, parent_of = {}, {}
    for j in joints:
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        if c in parent_of:
            bad(f"link '{c}' has two parents "
                f"({parent_of[c]} and {j.get('name')})")
        parent_of[c] = j.get("name")
        child_of.setdefault(p, []).append(c)
        for ln in (p, c):
            if ln not in lnames:
                bad(f"joint '{j.get('name')}' references undeclared link '{ln}'")

    roots = [l for l in lnames if l not in parent_of]
    if len(roots) == 1:
        ok(f"exactly one root link: {roots[0]}")
    else:
        bad(f"expected 1 root link, found {len(roots)}: {roots}")

    if len(joints) == len(links) - 1:
        ok(f"joint count is links-1 ({len(joints)}), consistent with a tree")
    else:
        bad(f"{len(links)} links but {len(joints)} joints - not a tree")

    seen, stack = set(), [roots[0]] if roots else []
    while stack:
        n = stack.pop()
        if n in seen:
            bad(f"cycle detected at '{n}'")
            break
        seen.add(n)
        stack.extend(child_of.get(n, []))
    orphans = set(lnames) - seen
    if orphans:
        bad(f"orphan links unreachable from root: {sorted(orphans)}")
    else:
        ok(f"all {len(lnames)} links reachable from root, no orphans")

    # -------------------------------------------------------------- joints
    print("\n--- joint inventory ---")
    movable = {j.get("name"): j.get("type") for j in joints
               if j.get("type") != "fixed"}
    if set(movable) == EXPECTED_MOVABLE:
        ok(f"exactly {len(movable)} movable joints, all as specified")
    else:
        bad(f"movable joints differ.\n         missing: "
            f"{sorted(EXPECTED_MOVABLE - set(movable))}\n         "
            f"unexpected: {sorted(set(movable) - EXPECTED_MOVABLE)}")
    for n, t in sorted(movable.items()):
        want = "continuous" if n.startswith("wheel_") else "revolute"
        (ok if t == want else bad)(f"  {n:<16s} {t}")
    for j in joints:
        if j.get("type") in ("revolute", "prismatic") and j.find("limit") is None:
            bad(f"{j.get('name')} is {j.get('type')} but has no <limit>")

    # --------------------------------------------------------------- FK
    print("\n--- forward kinematics (zero joint state) ---")
    jmap = {j.find("child").get("link"): j for j in joints}
    world = {roots[0]: np.eye(4)} if roots else {}

    def fk(link):
        if link in world:
            return world[link]
        j = jmap[link]
        world[link] = fk(j.find("parent").get("link")) @ origin_of(j)
        return world[link]

    for l in lnames:
        fk(l)

    wheel_r = 0.030
    for i in range(3):
        ln = f"wheel_{i}_link"
        if ln not in world:
            continue
        T = world[ln]
        bottom = T[2, 3] - wheel_r
        axis = [float(v) for v in jmap[ln].find("axis").get("xyz").split()]
        ax_w = T[:3, :3] @ np.array(axis)
        radial = T[:3, 3].copy()
        radial[2] = 0
        radial /= np.linalg.norm(radial)
        dot = float(ax_w @ radial)
        bearing = math.degrees(math.atan2(T[1, 3], T[0, 3])) % 360
        (ok if abs(bottom) < 1e-9 else bad)(
            f"{ln}: centre z={T[2,3]:+.6f}  bottom z={bottom:+.9f} "
            f"(must be 0)")
        (ok if abs(dot - 1) < 1e-9 else bad)(
            f"{ln}: spin axis {np.round(ax_w,4)} is radially OUTWARD "
            f"(dot={dot:.6f}), mounting bearing {bearing:.2f} deg")

    for ln in ("laser_frame", "arm_base_link"):
        if ln in world:
            t = world[ln][:3, 3]
            ok(f"{ln:<16s} at base_footprint ({t[0]:+.6f}, {t[1]:+.6f}, "
               f"{t[2]:+.6f}) m")

    # -------------------------------------------------------------- meshes
    print("\n--- meshes ---")
    n_mesh, missing, nonpkg = 0, [], []
    for m in root.iter("mesh"):
        fn = m.get("filename")
        n_mesh += 1
        if not fn.startswith("package://"):
            nonpkg.append(fn)
            continue
        rest = fn[len("package://"):]
        pkg, rel = rest.split("/", 1)
        if pkg != a.pkg_name:
            nonpkg.append(fn)
            continue
        if not os.path.isfile(os.path.join(a.pkg_root, rel)):
            missing.append(rel)
        if m.get("scale"):
            warn(f"{rel} has scale='{m.get('scale')}' - meshes should already "
                 "be in metres")
    (bad if nonpkg else ok)(
        f"all {n_mesh} mesh refs use package://{a.pkg_name}/"
        if not nonpkg else f"non-package:// or foreign refs: {set(nonpkg)}")
    if missing:
        for f in sorted(set(missing)):
            bad(f"missing mesh file: {f}")
    else:
        ok(f"all {n_mesh} referenced mesh files exist on disk")

    # .obj files need their .mtl beside them or colours are lost
    for m in root.iter("mesh"):
        fn = m.get("filename", "")
        if fn.endswith(".obj"):
            rel = fn[len("package://") + len(a.pkg_name) + 1:]
            mtl = os.path.join(a.pkg_root, rel[:-4] + ".mtl")
            if not os.path.isfile(mtl):
                warn(f"{rel} has no sibling .mtl - part colours will be lost")

    # collision must never point at a visual mesh
    vis = {m.get("filename") for l in links for v in l.findall("visual")
           for m in v.iter("mesh")}
    col = {m.get("filename") for l in links for c in l.findall("collision")
           for m in c.iter("mesh")}
    shared = vis & col
    (bad if shared else ok)(
        "no collision geometry points at a visual mesh"
        if not shared else f"collision reuses visual mesh: {sorted(shared)}")

    # ------------------------------------------------------------ inertials
    print("\n--- inertials ---")
    no_inertial = [l.get("name") for l in links if l.find("inertial") is None]
    for l in links:
        it = l.find("inertial")
        if it is None:
            continue
        I = it.find("inertia")
        d = [float(I.get(k)) for k in ("ixx", "iyy", "izz")]
        mass = float(it.find("mass").get("value"))
        if all(abs(v - 1.0) < 1e-12 for v in d):
            bad(f"{l.get('name')} has a placeholder identity inertia")
        if mass <= 0:
            bad(f"{l.get('name')} has mass {mass}")
        elif any(v <= 0 for v in d):
            warn(f"{l.get('name')} has a zero/negative inertia diagonal "
                 f"{d} (mass {mass})")
    ok(f"{len(links)-len(no_inertial)}/{len(links)} links carry an inertial; "
       "no identity placeholders")
    if no_inertial:
        warn(f"links with NO inertial (expected while chassis mass properties "
             f"are outstanding): {sorted(no_inertial)}")

    # ------------------------------------------------------------ materials
    defined = {m.get("name") for m in root.findall("material")}
    used = {m.get("name") for l in links for m in l.iter("material")
            if m.get("name") and not len(m)}
    undef = used - defined
    print("\n--- materials ---")
    (bad if undef else ok)(
        f"all {len(used)} referenced materials are defined"
        if not undef else f"undefined materials: {sorted(undef)}")

    # ---------------------------------------------------------------- tree
    print("\n--- frame tree ---")

    def show(n, pre=""):
        j = jmap.get(n)
        tag = ""
        if j is not None:
            tag = f"  [{j.get('name')}, {j.get('type')}]"
        print(f"    {pre}{n}{tag}")
        kids = child_of.get(n, [])
        for k in kids:
            show(k, pre + "  ")
    if roots:
        show(roots[0])

    print(f"\n=== {len(fails)} failures, {len(warns)} warnings ===")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

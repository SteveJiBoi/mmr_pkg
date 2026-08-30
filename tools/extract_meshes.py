"""Extract per-link visual + collision meshes from the STEP-derived GLB.

CAD frame -> base_link frame:
    p_link = Rz(+90deg) . (p_cad - (0.196, 0.001, 44.941) mm)
i.e. robot +X (forward) == CAD -Y, which is where the lidar, camera and arm face.
Ground plane is 45.000 mm below base_link; the wheel axle then sits at exactly
the 30.000 mm wheel radius.
"""
import collections
import json
import math
import os

import numpy as np
import trimesh
import fast_simplification

MM = 1000.0
ORIGIN_CAD_MM = np.array([0.196, 0.001, 44.941])
FORWARD_CAD_DEG = -90.0          # CAD bearing of robot +X
WHEEL_R = 0.030
WHEEL_ANGLES = {0: 60.0, 1: 180.0, 2: 300.0}
BASE_RADIUS = 0.135500

# Parts that still overshoot their triangle target by this factor after quadric
# decimation are replaced by their convex hull (see decimate()). Set to a huge
# number to keep every part's true shape -- base_link then lands at ~101k tris
# instead of ~44k, mostly Pi-cooler fins and fan blades.
HULL_OVERSHOOT = 3.0

OUT_V = "meshes/visual"
OUT_C = "meshes/collision"
os.makedirs(OUT_V, exist_ok=True)
os.makedirs(OUT_C, exist_ok=True)


def Rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    M[0, 0], M[0, 1] = c, -s
    M[1, 0], M[1, 1] = s, c
    return M


# CAD (mm, glb already in m) -> base_link (m)
T_CAD_TO_LINK = Rz(-FORWARD_CAD_DEG) @ trimesh.transformations.translation_matrix(
    -ORIGIN_CAD_MM / MM)

scene = trimesh.load("tools/mmr_bot.glb", process=False)
graph = scene.graph
parents = graph.transforms.parents
kids = collections.defaultdict(list)
for k, v in parents.items():
    kids[v].append(k)
asm = [n for n in graph.nodes if parents.get(n) == graph.base_frame][0]


def subtree(n):
    out, st = [], [n]
    while st:
        c = st.pop()
        out.append(c)
        st.extend(kids.get(c, []))
    return out


def parts(top_nodes, extra_T=np.eye(4)):
    """Return [(name, mesh-in-target-frame)] for the given top-level nodes."""
    out = []
    for top in top_nodes:
        for n in subtree(top):
            try:
                T, gname = graph[n]
            except Exception:
                continue
            if gname is None:
                continue
            g = scene.geometry[gname]
            if len(g.vertices) == 0:
                continue
            m = g.copy()
            m.apply_transform(extra_T @ T_CAD_TO_LINK @ T)
            out.append((n, m))
    return out


TOP = sorted(kids[asm])
wheels = [c for c in TOP if c.startswith("RodaOmni")]
arm = [c for c in TOP if c.startswith("SO101")]
lidar = [c for c in TOP if c.startswith("RPLIDAR")]
chassis = [c for c in TOP if c not in wheels + arm + lidar]
print("chassis components:", len(chassis))
for c in chassis:
    print("   ", c)


def _retexture(m2, src):
    """Carry the PBR material across; vertex-count change invalidates any uv."""
    try:
        m2.visual = trimesh.visual.TextureVisuals(material=src.visual.material)
    except Exception:
        pass
    return m2


def decimate(pairs, budget, floor=24, hull_overshoot=HULL_OVERSHOOT):
    """Decimate a set of parts to ~budget triangles total.

    Triangles are allocated in proportion to each part's SURFACE AREA, not
    uniformly: the CAD carries 18k triangles on a 4.4 mm spring and 56k on a
    heatsink fin stack, so a uniform ratio spends the whole budget on fasteners
    nobody can see.

    Quadric decimation has a hard topological floor: thin heatsink fins, coil
    springs and screw threads are high-genus and cannot be collapsed without
    changing topology. Any part still overshooting its target by more than
    `hull_overshoot` after decimation is replaced by its convex hull. Every
    such substitution is printed, because hulling a part that genuinely needs
    its holes (a plate, a bracket) would be wrong -- the list is meant to be
    eyeballed. Large plates never trigger it: their area share buys them a
    target far above their floor.
    """
    # cascadio splits vertices at every B-rep patch boundary (~39% duplicates),
    # which blocks edge collapse across patches and wrecks decimation quality.
    # Merging first is mandatory, not an optimisation.
    for _, m in pairs:
        m.merge_vertices()
    tot = sum(len(m.faces) for _, m in pairs)
    if tot <= budget:
        print(f"    no decimation needed ({tot} <= {budget})")
        return pairs, tot, tot

    areas = np.array([max(m.area, 1e-12) for _, m in pairs])
    share = areas / areas.sum()
    out, n_hulled = [], 0
    for (n, m), sh in zip(pairs, share):
        nf = len(m.faces)
        target = int(min(nf, max(floor, round(budget * sh))))
        if target >= nf:
            out.append((n, m))
            continue
        try:
            v, f = fast_simplification.simplify(
                np.asarray(m.vertices, dtype=np.float32),
                np.asarray(m.faces, dtype=np.uint32),
                target_count=target, agg=8.0)
            # hit the topological floor -> substitute the convex hull
            if len(f) > hull_overshoot * target:
                h = m.convex_hull
                if len(h.faces) > target:
                    hv, hf = fast_simplification.simplify(
                        np.asarray(h.vertices, dtype=np.float32),
                        np.asarray(h.faces, dtype=np.uint32),
                        target_count=target, agg=8.0)
                    h = trimesh.Trimesh(vertices=hv, faces=hf, process=False)
                if len(h.faces) < len(f):
                    diag = float(np.linalg.norm(
                        m.bounds[1] - m.bounds[0])) * MM
                    print(f"      hulled {n[:52]:<52s} {nf:6d} -> "
                          f"{len(h.faces):5d} tris ({diag:5.1f} mm)")
                    out.append((n, _retexture(h, m)))
                    n_hulled += 1
                    continue
            out.append((n, _retexture(
                trimesh.Trimesh(vertices=v, faces=f, process=False), m)))
        except Exception as e:
            print(f"      ! decimate failed on {n}: {e}")
            out.append((n, m))
    new = sum(len(m.faces) for _, m in out)
    print(f"    decimated {tot} -> {new} tris (budget {budget}; "
          f"{n_hulled}/{len(pairs)} small parts hulled)")
    return out, tot, new


def export_obj(pairs, path):
    """Write <path>.obj plus a sibling <stem>.mtl carrying the CAD part colours."""
    sc = trimesh.Scene()
    for i, (n, m) in enumerate(pairs):
        sc.add_geometry(m, node_name=f"p{i:04d}", geom_name=f"p{i:04d}")
    stem = os.path.splitext(os.path.basename(path))[0]
    mtl_name = f"{stem}.mtl"
    text, files = trimesh.exchange.obj.export_obj(
        sc, include_color=True, include_texture=True,
        return_texture=True, mtl_name=mtl_name)
    with open(path, "w") as f:
        f.write(text)
    nmat = 0
    for fname, blob in (files or {}).items():
        out = os.path.join(os.path.dirname(path), os.path.basename(fname))
        with open(out, "wb") as f:
            f.write(blob if isinstance(blob, bytes) else blob.encode())
        if out.endswith(".mtl"):
            nmat = blob.decode().count("newmtl") if isinstance(blob, bytes) \
                else blob.count("newmtl")
            print(f"    wrote {out} ({nmat} materials)")
    if nmat == 0:
        print(f"    ! WARNING no materials written for {path}")
    return path


report = {}

# ---------------- base_link ----------------
print("\n== base_link ==")
p = parts(chassis)
p, t0, t1 = decimate(p, 45000)
export_obj(p, f"{OUT_V}/base_link.obj")
allv = np.vstack([m.vertices for _, m in p])
report["base_link"] = dict(tris_before=t0, tris_after=t1,
                           bbox_min=allv.min(axis=0).tolist(),
                           bbox_max=allv.max(axis=0).tolist())
print("   bbox m:", np.round(allv.min(axis=0), 4), np.round(allv.max(axis=0), 4))

# ---------------- wheel (canonical link frame) ----------------
# wheel_i_link = base_link * Trans(BASE_RADIUS @ angle) * Rz(angle)
# so link frame has +X radially outward (= spin axis), matching the CAD axes.
print("\n== wheel (canonical) ==")
src = [c for c in wheels if c.endswith(":3")][0]     # the 180 deg one
ang = WHEEL_ANGLES[1]
T_link = (trimesh.transformations.translation_matrix(
    [BASE_RADIUS * math.cos(math.radians(ang)),
     BASE_RADIUS * math.sin(math.radians(ang)), -0.015]) @ Rz(ang))
p = parts([src], extra_T=np.linalg.inv(T_link))
p, t0, t1 = decimate(p, 16000)
export_obj(p, f"{OUT_V}/wheel.obj")
allv = np.vstack([m.vertices for _, m in p])
print("   bbox m:", np.round(allv.min(axis=0), 5), np.round(allv.max(axis=0), 5))
print(f"   -> spin axis is local +X; radius {max(np.hypot(allv[:,1],allv[:,2])):.5f} m"
      f"  half-width {max(abs(allv[:,0])):.5f} m")
report["wheel"] = dict(tris_before=t0, tris_after=t1,
                       bbox_min=allv.min(axis=0).tolist(),
                       bbox_max=allv.max(axis=0).tolist())

# ---------------- laser_frame ----------------
# rotor axis CAD (0.196+0.004, -133.78); turret mid-height CAD z 137.0..163.241
LIDAR_CAD = np.array([0.200, -133.78, (137.0 + 163.241) / 2])
lidar_link = (Rz(-FORWARD_CAD_DEG) @ trimesh.transformations.translation_matrix(
    -ORIGIN_CAD_MM / MM)) @ trimesh.transformations.translation_matrix(
        LIDAR_CAD / MM)
LASER_XYZ = (Rz(-FORWARD_CAD_DEG)[:3, :3] @ ((LIDAR_CAD - ORIGIN_CAD_MM) / MM))
print("\n== laser_frame ==")
print("   laser_frame xyz rel base_link (m):", np.round(LASER_XYZ, 6))
T_laser = trimesh.transformations.translation_matrix(LASER_XYZ)
p = parts(lidar, extra_T=np.linalg.inv(T_laser))
p, t0, t1 = decimate(p, 12000)
export_obj(p, f"{OUT_V}/rplidar_c1.obj")
allv = np.vstack([m.vertices for _, m in p])
print("   bbox m:", np.round(allv.min(axis=0), 5), np.round(allv.max(axis=0), 5))
report["laser_frame"] = dict(tris_before=t0, tris_after=t1,
                             xyz=LASER_XYZ.tolist(),
                             bbox_min=allv.min(axis=0).tolist(),
                             bbox_max=allv.max(axis=0).tolist())

# ---------------- collision hulls ----------------
print("\n== collision hulls ==")


def hull(nodes, path, extra_T=np.eye(4)):
    ms = [m for _, m in parts(nodes, extra_T)]
    merged = trimesh.util.concatenate(ms)
    h = merged.convex_hull
    h.export(path)
    print(f"   {path:<44s} tris={len(h.faces):5d} vol={h.volume*1e6:8.2f} cm^3")
    return h


# motor pod, rotated back to the canonical 0 deg position so one mesh serves 3
pod_src = [c for c in chassis if c.startswith(("BaseMotor v1:3",
                                               "BaseMotorMount v1:3"))]
print("   pod source:", pod_src)
hull(pod_src, f"{OUT_C}/motor_pod.stl", extra_T=Rz(-WHEEL_ANGLES[1]))
cam = [c for c in chassis if c.startswith(("BaseCamMount", "Innomaker"))]
hull(cam, f"{OUT_C}/camera_pod.stl")
pi = [c for c in chassis if c.startswith(("Pi_5_", "raspberry_pi_5", "Heatsink"))]
hull(pi, f"{OUT_C}/pi_stack.stl")
hull(lidar, f"{OUT_C}/rplidar_c1.stl", extra_T=np.linalg.inv(T_laser))

json.dump(report, open("tools/mesh_report.json", "w"), indent=1)
print("\nwrote tools/mesh_report.json")
for f in sorted(os.listdir(OUT_V)):
    print(f"  {OUT_V}/{f}  {os.path.getsize(os.path.join(OUT_V,f))/1e6:.2f} MB")
for f in sorted(os.listdir(OUT_C)):
    print(f"  {OUT_C}/{f}  {os.path.getsize(os.path.join(OUT_C,f))/1e6:.2f} MB")

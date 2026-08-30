"""Final frame report: CAD mm -> robot metres, for both candidate forward axes."""
import numpy as np
import trimesh
import collections
import math
import json

scene = trimesh.load("tools/mmr_bot.glb", process=False)
graph = scene.graph
parents = graph.transforms.parents
kids = collections.defaultdict(list)
for k, v in parents.items():
    kids[v].append(k)
geom = {}
for n in graph.nodes:
    try:
        T, g = graph[n]
    except Exception:
        continue
    if g is None:
        continue
    m = scene.geometry[g]
    if len(m.vertices):
        geom[n] = (T, m)
asm = [n for n in graph.nodes if parents.get(n) == graph.base_frame][0]


def subtree(n):
    out, st = [], [n]
    while st:
        c = st.pop()
        out.append(c)
        st.extend(kids.get(c, []))
    return out


def verts_mm(n):
    vs = []
    for c in subtree(n):
        if c in geom:
            T, m = geom[c]
            vs.append(trimesh.transform_points(np.asarray(m.vertices), T) * 1000.0)
    return np.vstack(vs) if vs else None


def merged_mesh(n):
    ms = []
    for c in subtree(n):
        if c in geom:
            T, m = geom[c]
            mm = m.copy()
            mm.apply_transform(T)
            ms.append(mm)
    if not ms:
        return None
    return trimesh.util.concatenate(ms)


print("=== RPLIDAR C1 geometry detail (CAD mm) ===")
li = [c for c in kids[asm] if c.startswith("RPLIDAR")][0]
V = verts_mm(li)
mn, mx = V.min(axis=0), V.max(axis=0)
print(f"  bbox x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}] z[{mn[2]:.3f},{mx[2]:.3f}]")
for z0 in range(int(mn[2]), int(mx[2]), 4):
    sel = (V[:, 2] >= z0) & (V[:, 2] < z0 + 4)
    if sel.sum() < 5:
        continue
    s = V[sel]
    cx = (s[:, 0].min() + s[:, 0].max()) / 2
    cy = (s[:, 1].min() + s[:, 1].max()) / 2
    r = np.hypot(s[:, 0] - cx, s[:, 1] - cy)
    print(f"   z[{z0:3d},{z0+4:3d}) n={sel.sum():5d} "
          f"ctr=({cx:7.2f},{cy:7.2f}) dx={s[:,0].max()-s[:,0].min():6.2f} "
          f"dy={s[:,1].max()-s[:,1].min():6.2f} rmax={r.max():6.2f} "
          f"round={r.max()/max((s[:,0].max()-s[:,0].min())/2,1e-9):.3f}")

print("\n=== COMPONENT VOLUMES (watertight check) ===")
vols = {}
for c in sorted(kids[asm]):
    m = merged_mesh(c)
    if m is None:
        continue
    m.merge_vertices()
    wt = m.is_watertight
    vol = abs(m.volume) * 1e9 if wt else float("nan")
    ch = m.convex_hull
    print(f"  {c:<34s} watertight={str(wt):<5s} vol={vol:12.1f} mm^3  "
          f"hull_vol={ch.volume*1e9:12.1f} mm^3  tris={len(m.faces):7d}")
    vols[c] = dict(watertight=bool(wt),
                   vol_mm3=None if not wt else float(vol),
                   hull_mm3=float(ch.volume * 1e9), tris=int(len(m.faces)))
json.dump(vols, open("tools/volumes.json", "w"), indent=1)

# ---------------- frame conversion ----------------
ORIGIN_XY = np.array([0.196, 0.001])     # true wheel-triad centre in CAD mm
BASE_Z = 44.941                          # underside of base plate, CAD mm


def to_robot(p_cad_mm, yaw_deg):
    """CAD mm -> robot metres. yaw_deg = CAD angle of robot +X (forward)."""
    p = np.asarray(p_cad_mm, dtype=float).copy()
    p[0] -= ORIGIN_XY[0]
    p[1] -= ORIGIN_XY[1]
    p[2] -= BASE_Z
    a = math.radians(-yaw_deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]]) / 1000.0


CANDIDATES = {"A: forward = CAD -Y (lidar/arm/camera face this way)": -90.0,
              "B: forward = CAD -X": 180.0}

wheels = json.load(open("tools/wheels.json"))
arm_t_cad = np.array([-0.2463, -14.0897, 124.1411])
arm_yaw_cad = -90.0
lidar_ctr_cad = np.array([0.196, -129.18, 0.0])

for label, fwd in CANDIDATES.items():
    print(f"\n\n################ {label}  (yaw={fwd} deg) ################")
    print("--- wheels ---")
    ws = []
    for k in sorted(wheels):
        w = wheels[k]
        c = np.array(w["centre"])
        rp = to_robot(c, fwd)
        ang = math.degrees(math.atan2(rp[1], rp[0])) % 360.0
        ax = np.array(w["axis"])
        a = math.radians(-fwd)
        axr = np.array([math.cos(a) * ax[0] - math.sin(a) * ax[1],
                        math.sin(a) * ax[0] + math.cos(a) * ax[1], ax[2]])
        axang = math.degrees(math.atan2(axr[1], axr[0])) % 360.0
        ws.append((k, ang, rp, axr, axang))
    for k, ang, rp, axr, axang in sorted(ws, key=lambda t: t[1]):
        print(f"  {k:<22s} mount_angle={ang:8.3f} deg  "
              f"xyz=({rp[0]:+.6f},{rp[1]:+.6f},{rp[2]:+.6f}) m  "
              f"axis=({axr[0]:+.4f},{axr[1]:+.4f},{axr[2]:+.4f}) @{axang:7.3f} deg")
    print(f"  -> mounting angle set: "
          f"{sorted(round(a,1) for _,a,_,_,_ in ws)}")
    ar = to_robot(arm_t_cad, fwd)
    print(f"--- arm base_link: xyz=({ar[0]:+.6f},{ar[1]:+.6f},{ar[2]:+.6f}) m  "
          f"rpy=(0,0,{math.radians(arm_yaw_cad-fwd):+.6f}) rad "
          f"[{arm_yaw_cad-fwd:+.1f} deg]")
    lr = to_robot(lidar_ctr_cad, fwd)
    print(f"--- lidar XY:      xyz=({lr[0]:+.6f},{lr[1]:+.6f}, z TBD) m  "
          f"bearing={math.degrees(math.atan2(lr[1],lr[0])):.3f} deg from forward")

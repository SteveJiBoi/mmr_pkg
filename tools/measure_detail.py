"""Precise measurements: wheel axes/radii, plate shape, mount frames."""
import numpy as np
import trimesh
import collections
import json
import math

scene = trimesh.load("tools/mmr_bot.glb", process=False)
graph = scene.graph
parents = graph.transforms.parents
kids = collections.defaultdict(list)
for k, v in parents.items():
    kids[v].append(k)

geom = {}
for name in graph.nodes:
    try:
        T, gname = graph[name]
    except Exception:
        continue
    if gname is None:
        continue
    g = scene.geometry[gname]
    if len(g.vertices):
        geom[name] = (T, g)


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
            T, g = geom[c]
            vs.append(trimesh.transform_points(np.asarray(g.vertices), T) * 1000.0)
    return np.vstack(vs) if vs else None


print("=== ALL NODE NAMES (depth<=2) ===")
asm = [n for n in graph.nodes if parents.get(n) == graph.base_frame][0]
for c in sorted(kids[asm]):
    print(f"  {c}")
    for gc in sorted(kids.get(c, []))[:200]:
        print(f"      {gc}")

print("\n\n=== WHEEL FITS ===")
wheels = sorted([c for c in kids[asm] if c.startswith("RodaOmni")])
res = {}
for w in wheels:
    V = verts_mm(w)
    ctr = V.mean(axis=0)
    # PCA: axis of least variance == spin axis for a disc
    X = V - ctr
    cov = X.T @ X / len(X)
    evals, evecs = np.linalg.eigh(cov)
    axis = evecs[:, 0]           # smallest eigenvalue
    if axis[:2] @ ctr[:2] < 0:   # orient radially outward
        axis = -axis
    # geometric centre = midpoint of extent along axis + radial extents
    t = X @ axis
    perp = X - np.outer(t, axis)
    rad = np.linalg.norm(perp, axis=1)
    # refine centre: midpoint of bbox along each principal dir
    c2 = ctr + axis * (t.min() + t.max()) / 2
    ang = math.degrees(math.atan2(c2[1], c2[0]))
    rxy = math.hypot(c2[0], c2[1])
    axang = math.degrees(math.atan2(axis[1], axis[0]))
    print(f"{w}")
    print(f"   centroid           = ({ctr[0]:8.3f},{ctr[1]:8.3f},{ctr[2]:8.3f})")
    print(f"   axis-centred       = ({c2[0]:8.3f},{c2[1]:8.3f},{c2[2]:8.3f})")
    print(f"   r_xy={rxy:8.3f}  pos_angle={ang:8.3f} deg")
    print(f"   spin axis          = ({axis[0]:+.5f},{axis[1]:+.5f},{axis[2]:+.5f})"
          f"  angle={axang:8.3f} deg  (radial? {abs(((axang-ang+180)%360)-180):.3f} deg off)")
    print(f"   width along axis   = {t.max()-t.min():8.3f} mm "
          f"[{t.min():.3f},{t.max():.3f}]")
    print(f"   max radius         = {rad.max():8.3f} mm   "
          f"(p99={np.percentile(rad,99):.3f})")
    print(f"   z extent           = [{V[:,2].min():.3f},{V[:,2].max():.3f}]")
    res[w] = dict(centre=c2.tolist(), axis=axis.tolist(), r_xy=rxy,
                  pos_angle=ang, width=float(t.max() - t.min()),
                  max_r=float(rad.max()),
                  zmin=float(V[:, 2].min()), zmax=float(V[:, 2].max()))

# roller-only extent (contact band width)
print("\n=== WHEEL SUB-PARTS (wheel 1) ===")
w0 = wheels[0]
for c in sorted(kids.get(w0, [])):
    V = verts_mm(c)
    if V is None:
        continue
    print(f"   {c:<40s} bbox_mm x[{V[:,0].min():8.2f},{V[:,0].max():8.2f}] "
          f"y[{V[:,1].min():8.2f},{V[:,1].max():8.2f}] "
          f"z[{V[:,2].min():8.2f},{V[:,2].max():8.2f}]")

print("\n=== BASE PLATE SHAPE ===")
bp = [c for c in kids[asm] if c.startswith("BasePlate")][0]
V = verts_mm(bp)
print(f"  bbox x[{V[:,0].min():.3f},{V[:,0].max():.3f}] "
      f"y[{V[:,1].min():.3f},{V[:,1].max():.3f}] "
      f"z[{V[:,2].min():.3f},{V[:,2].max():.3f}]")
cx = (V[:, 0].min() + V[:, 0].max()) / 2
cy = (V[:, 1].min() + V[:, 1].max()) / 2
r = np.hypot(V[:, 0] - cx, V[:, 1] - cy)
print(f"  centre=({cx:.3f},{cy:.3f})  radial dist: min={r.min():.3f} "
      f"max={r.max():.3f} mean={r.mean():.3f}")
outer = r > r.max() - 2.0
print(f"  verts within 2mm of max radius: {outer.sum()} / {len(r)}")
th = np.degrees(np.arctan2(V[outer, 1] - cy, V[outer, 0] - cx))
hist, edges = np.histogram(th, bins=36, range=(-180, 180))
print("  angular coverage of outer-radius verts (10deg bins):")
print("   ", " ".join(f"{h:3d}" for h in hist))
# square test: how many verts lie near |x-cx|=max or |y-cy|=max
print(f"  ratio max_radial/half_x = {r.max()/((V[:,0].max()-V[:,0].min())/2):.4f} "
      f"(1.000=circle, 1.414=square)")

print("\n=== TOP PLATE ===")
tp = [c for c in kids[asm] if c.startswith("TopPlate")][0]
V = verts_mm(tp)
cx2 = (V[:, 0].min() + V[:, 0].max()) / 2
cy2 = (V[:, 1].min() + V[:, 1].max()) / 2
r2 = np.hypot(V[:, 0] - cx2, V[:, 1] - cy2)
print(f"  bbox x[{V[:,0].min():.3f},{V[:,0].max():.3f}] "
      f"y[{V[:,1].min():.3f},{V[:,1].max():.3f}] "
      f"z[{V[:,2].min():.3f},{V[:,2].max():.3f}]")
print(f"  ratio max_radial/half_x = {r2.max()/((V[:,0].max()-V[:,0].min())/2):.4f}")

print("\n=== OTHER COMPONENT CENTRES (bbox centre, mm) ===")
for c in sorted(kids[asm]):
    V = verts_mm(c)
    if V is None:
        continue
    mn, mx = V.min(axis=0), V.max(axis=0)
    ctr = (mn + mx) / 2
    print(f"  {c:<34s} ctr=({ctr[0]:8.2f},{ctr[1]:8.2f},{ctr[2]:8.2f})  "
          f"size=({mx[0]-mn[0]:7.2f},{mx[1]-mn[1]:7.2f},{mx[2]-mn[2]:7.2f})")

json.dump(res, open("tools/wheels.json", "w"), indent=1)

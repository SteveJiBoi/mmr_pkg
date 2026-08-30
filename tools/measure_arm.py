import numpy as np
import trimesh
import collections
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
        T, g = graph[name]
    except Exception:
        continue
    if g is None:
        continue
    m = scene.geometry[g]
    if len(m.vertices):
        geom[name] = (T, m)


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


asm = [n for n in graph.nodes if parents.get(n) == graph.base_frame][0]
so = [c for c in kids[asm] if c.startswith("SO101")][0]
print(f"=== {so} children ===")
for c in sorted(kids[so]):
    V = verts_mm(c)
    if V is None:
        print(f"  {c:<44s} (no geom)")
        continue
    mn, mx = V.min(axis=0), V.max(axis=0)
    print(f"  {c:<44s} x[{mn[0]:8.2f},{mx[0]:8.2f}] y[{mn[1]:8.2f},{mx[1]:8.2f}]"
          f" z[{mn[2]:8.2f},{mx[2]:8.2f}]")

print("\n=== node local transforms (mm) for key frames ===")
for c in [so] + sorted(kids[so]):
    T = graph.get(c, asm)[0] if c != so else graph.get(so, asm)[0]
    t = T[:3, 3] * 1000
    R = T[:3, :3]
    print(f"  {c:<44s} t=({t[0]:9.3f},{t[1]:9.3f},{t[2]:9.3f})")
    print(f"       R=[{R[0,0]:+.4f} {R[0,1]:+.4f} {R[0,2]:+.4f}]"
          f"[{R[1,0]:+.4f} {R[1,1]:+.4f} {R[1,2]:+.4f}]"
          f"[{R[2,0]:+.4f} {R[2,1]:+.4f} {R[2,2]:+.4f}]")

print("\n=== WaveShare mounting plate / Base_SO101 detail ===")
for key in ["WaveShare", "Base_SO101", "Rotation_Pitch"]:
    for c in kids[so]:
        if key.lower() in c.lower():
            V = verts_mm(c)
            mn, mx = V.min(axis=0), V.max(axis=0)
            ctr = (mn + mx) / 2
            print(f"  {c}")
            print(f"    bbox x[{mn[0]:8.3f},{mx[0]:8.3f}] y[{mn[1]:8.3f},{mx[1]:8.3f}]"
                  f" z[{mn[2]:8.3f},{mx[2]:8.3f}]")
            print(f"    size=({mx[0]-mn[0]:.3f},{mx[1]-mn[1]:.3f},{mx[2]-mn[2]:.3f})"
                  f" ctr=({ctr[0]:.3f},{ctr[1]:.3f},{ctr[2]:.3f})")

print("\n=== RPLIDAR detail ===")
li = [c for c in kids[asm] if c.startswith("RPLIDAR")][0]
V = verts_mm(li)
mn, mx = V.min(axis=0), V.max(axis=0)
print(f"  bbox x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}] "
      f"z[{mn[2]:.3f},{mx[2]:.3f}]")
print(f"  size=({mx[0]-mn[0]:.3f},{mx[1]-mn[1]:.3f},{mx[2]-mn[2]:.3f})")
T = graph.get(li, asm)[0]
print(f"  node T t={T[:3,3]*1000}")
print(f"  R=\n{T[:3,:3]}")
# find the spinning-head cylinder axis: cluster verts by z
for zlo, zhi in [(mn[2], mn[2] + 10), (mx[2] - 20, mx[2])]:
    sel = (V[:, 2] >= zlo) & (V[:, 2] <= zhi)
    s = V[sel]
    print(f"  slab z[{zlo:.1f},{zhi:.1f}]: x[{s[:,0].min():.2f},{s[:,0].max():.2f}]"
          f" y[{s[:,1].min():.2f},{s[:,1].max():.2f}] n={sel.sum()}")

print("\n=== TopPlate / BasePlate z faces ===")
for nm in ["TopPlate", "BasePlate"]:
    c = [x for x in kids[asm] if x.startswith(nm)][0]
    V = verts_mm(c)
    print(f"  {c}: z[{V[:,2].min():.3f},{V[:,2].max():.3f}]")

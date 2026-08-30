"""Load the tessellated assembly and measure per-component geometry.

cascadio emits glTF, which is metres, so everything here is metres in
assembly-origin coordinates. Cross-check against the AP214 mm placements.
"""
import numpy as np
import trimesh
import collections
import json

scene = trimesh.load("tools/mmr_bot.glb", process=False)
graph = scene.graph
root = graph.base_frame
parents = graph.transforms.parents

# find the single child of the glTF root: the STEP root product
top_nodes = [n for n in graph.nodes if parents.get(n) == root]
print("children of", root, ":", top_nodes)
asm = top_nodes[0]
comps = [n for n in graph.nodes if parents.get(n) == asm]
print(f"depth-1 components under '{asm}': {len(comps)}")


def descendants(n):
    out = [n]
    i = 0
    kids = collections.defaultdict(list)
    for k, v in parents.items():
        kids[v].append(k)
    stack = [n]
    out = []
    while stack:
        c = stack.pop()
        out.append(c)
        stack.extend(kids.get(c, []))
    return out


geom_nodes = {}
for name in graph.nodes:
    try:
        T, gname = graph[name]
    except Exception:
        continue
    if gname is None:
        continue
    g = scene.geometry[gname]
    if len(g.vertices) == 0:
        continue
    geom_nodes[name] = (T, g)


def measure(nodelist):
    mins, maxs = [], []
    tris = 0
    nmesh = 0
    vol = 0.0
    cent_acc = np.zeros(3)
    cent_w = 0.0
    allv = []
    for n in nodelist:
        if n not in geom_nodes:
            continue
        T, g = geom_nodes[n]
        vw = trimesh.transform_points(np.asarray(g.vertices), T)
        mins.append(vw.min(axis=0))
        maxs.append(vw.max(axis=0))
        tris += len(g.faces)
        nmesh += 1
        allv.append(vw)
        try:
            m = g.copy()
            m.apply_transform(T)
            if m.is_volume:
                v = abs(m.volume)
                vol += v
                cent_acc += m.center_mass * v
                cent_w += v
        except Exception:
            pass
    if not mins:
        return None
    mn = np.min(mins, axis=0)
    mx = np.max(maxs, axis=0)
    com = (cent_acc / cent_w) if cent_w > 0 else None
    return dict(min=mn, max=mx, tris=tris, nmesh=nmesh, vol=vol, com=com,
                verts=np.vstack(allv) if allv else None)


print(f"\n{'component':<34s} {'nm':>3s} {'tris':>7s} "
      f"{'xmin':>8s} {'xmax':>8s} {'ymin':>8s} {'ymax':>8s} "
      f"{'zmin':>8s} {'zmax':>8s} {'vol_cm3':>9s}")
summary = {}
for c in comps:
    d = measure(descendants(c))
    if d is None:
        print(f"{c:<34s}  (no geometry)")
        continue
    mn, mx = d["min"], d["max"]
    print(f"{c:<34s} {d['nmesh']:3d} {d['tris']:7d} "
          f"{mn[0]*1000:8.2f} {mx[0]*1000:8.2f} {mn[1]*1000:8.2f} "
          f"{mx[1]*1000:8.2f} {mn[2]*1000:8.2f} {mx[2]*1000:8.2f} "
          f"{d['vol']*1e6:9.2f}")
    summary[c] = {
        "min_mm": (mn * 1000).tolist(), "max_mm": (mx * 1000).tolist(),
        "size_mm": ((mx - mn) * 1000).tolist(),
        "tris": d["tris"], "nmesh": d["nmesh"],
        "vol_mm3": d["vol"] * 1e9,
        "com_mm": (d["com"] * 1000).tolist() if d["com"] is not None else None,
    }

tot = measure(list(graph.nodes))
print(f"\nWHOLE ROBOT bbox mm: min={tot['min']*1000} max={tot['max']*1000}")
print(f"total tris: {tot['tris']}")

with open("tools/bbox.json", "w") as f:
    json.dump(summary, f, indent=1)
print("wrote tools/bbox.json")

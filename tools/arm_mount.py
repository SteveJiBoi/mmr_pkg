"""Locate the SO-101 URDF base_link frame inside the CAD assembly.

The CAD part 'Base_SO101' and the upstream 'base_so101_v2.stl' are the same
solid, so if their local mesh frames agree we can compose the CAD placement
with the inverse of the URDF visual origin to recover base_link.
"""
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

asm = [n for n in graph.nodes if parents.get(n) == graph.base_frame][0]
so = [c for c in kids[asm] if c.startswith("SO101")][0]
print("SO101 node:", so)
print("children:", len(kids[so]))
target = [c for c in kids[so] if c.startswith("Base_SO101")]
print("Base_SO101 nodes:", target)
node = target[0]

T_world, gname = graph[node]
mesh = scene.geometry[gname]
V = np.asarray(mesh.vertices)
print(f"\nCAD node '{node}' geometry '{gname}': {len(V)} verts {len(mesh.faces)} faces")
print("  LOCAL bbox (m):", V.min(axis=0), V.max(axis=0))
print("  LOCAL size (mm):", (V.max(axis=0) - V.min(axis=0)) * 1000)
print("  world T (t in mm):", T_world[:3, 3] * 1000)
print("  world R:\n", np.round(T_world[:3, :3], 6))

stl = trimesh.load("tools/upstream/base_so101_v2.stl", process=False)
S = np.asarray(stl.vertices)
print(f"\nupstream base_so101_v2.stl: {len(S)} verts {len(stl.faces)} faces")
print("  LOCAL bbox (m):", S.min(axis=0), S.max(axis=0))
print("  LOCAL size (mm):", (S.max(axis=0) - S.min(axis=0)) * 1000)

szc = np.sort((V.max(axis=0) - V.min(axis=0)) * 1000)
szs = np.sort((S.max(axis=0) - S.min(axis=0)) * 1000)
print("\n  sorted size CAD (mm):", np.round(szc, 3))
print("  sorted size STL (mm):", np.round(szs, 3))
print("  size match (sorted):", np.allclose(szc, szs, atol=0.3))
print("  local frames identical:",
      np.allclose(V.min(axis=0), S.min(axis=0), atol=3e-4) and
      np.allclose(V.max(axis=0), S.max(axis=0), atol=3e-4))


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    p = math.asin(sy)
    if abs(math.cos(p)) > 1e-9:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r = math.atan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


# URDF visual origin for the base_so101_v2 part inside base_link
vis_xyz = np.array([-0.00636471, -8.97657e-09, -0.0024])
vis_rpy = (1.5708, -2.78073e-29, 1.5708)
T_vis = np.eye(4)
T_vis[:3, :3] = rpy_to_R(*vis_rpy)
T_vis[:3, 3] = vis_xyz
print("\nURDF visual origin T_vis (base_link <- mesh local):")
print(np.round(T_vis, 6))

T_arm = T_world @ np.linalg.inv(T_vis)
print("\n=== arm base_link in CAD assembly frame ===")
print("  t (mm):", np.round(T_arm[:3, 3] * 1000, 4))
print("  R:\n", np.round(T_arm[:3, :3], 6))
r, p, y = R_to_rpy(T_arm[:3, :3])
print(f"  rpy (rad): {r:.6f} {p:.6f} {y:.6f}")
print(f"  rpy (deg): {math.degrees(r):.4f} {math.degrees(p):.4f} {math.degrees(y):.4f}")

# sanity: transform the STL through T_arm and compare with the CAD world bbox
Sw = trimesh.transform_points(S, T_arm @ T_vis) * 1000
Vw = trimesh.transform_points(V, T_world) * 1000
print("\n  STL through T_arm.T_vis  bbox mm:",
      np.round(Sw.min(axis=0), 3), np.round(Sw.max(axis=0), 3))
print("  CAD node world           bbox mm:",
      np.round(Vw.min(axis=0), 3), np.round(Vw.max(axis=0), 3))
print("  agreement:", np.allclose(Sw.min(axis=0), Vw.min(axis=0), atol=0.5) and
      np.allclose(Sw.max(axis=0), Vw.max(axis=0), atol=0.5))

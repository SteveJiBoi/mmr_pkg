"""Build collision meshes for the vendored SO-101 arm parts.

Upstream so101_new_calib.urdf points every <collision> at the same full-detail
STL used for <visual> (322k tris total), which the brief forbids in section 4.

A single convex hull per part is NOT usable here: the moving jaw, the
wrist_roll_pitch fork and the motor holders are deeply concave, and hulling
them fills the gripper opening solid. So each part is convex-DECOMPOSED with
CoACD into a handful of convex pieces, which keeps the concavity, collapses the
triangle count, and is the representation physics engines actually want.

The pieces for one part are concatenated into one STL so the URDF keeps
upstream's one-<collision>-per-part structure. For Phase 2 they can be split
into one <collision> element per piece if the solver needs guaranteed convexity.

NOTE: trimesh.load(process=False) leaves STL vertices unmerged (a triangle
soup), which silently destroys both CoACD and quadric decimation. merge_vertices()
before either is mandatory.
"""
import os

import numpy as np
import trimesh
import coacd

SRC = "meshes/arm"
DST = "meshes/collision/arm"
MAX_HULLS = 8
THRESHOLD = 0.05

os.makedirs(DST, exist_ok=True)
coacd.set_log_level("error")

print(f"{'part':<40s} {'vis':>7s} {'hulls':>6s} {'coll':>6s} "
      f"{'p50':>7s} {'p90':>7s} {'max mm':>7s}")
rows, tot_v, tot_c = [], 0, 0
for fn in sorted(os.listdir(SRC)):
    if not fn.endswith(".stl"):
        continue
    m = trimesh.load(os.path.join(SRC, fn), process=False)
    m.merge_vertices()                       # mandatory - see module docstring
    nf = len(m.faces)
    tot_v += nf

    pieces = coacd.run_coacd(coacd.Mesh(m.vertices, m.faces),
                             threshold=THRESHOLD, max_convex_hull=MAX_HULLS,
                             preprocess_mode="auto")
    # re-hull each piece so it is minimal and provably convex
    hulls = [trimesh.Trimesh(np.asarray(v), np.asarray(f),
                             process=False).convex_hull for v, f in pieces]
    out = trimesh.util.concatenate(hulls)
    out.export(os.path.join(DST, fn))
    tot_c += len(out.faces)

    pts = m.sample(4000)
    _, d, _ = trimesh.proximity.closest_point(out, pts)
    p50, p90, mx = (np.percentile(d, 50) * 1000, np.percentile(d, 90) * 1000,
                    d.max() * 1000)
    print(f"{fn:<40s} {nf:7d} {len(hulls):6d} {len(out.faces):6d} "
          f"{p50:7.3f} {p90:7.3f} {mx:7.3f}", flush=True)
    rows.append((fn, nf, len(hulls), len(out.faces), p50, p90, mx))

print(f"{'TOTAL':<40s} {tot_v:7d} {'':6s} {tot_c:6d}")
print(f"\ncollision is {100*tot_c/tot_v:.1f}% of visual triangle count")

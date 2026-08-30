import json
import math
from collections import defaultdict

rows = json.load(open("tools/tree.json"))

print("=== rep_type histogram ===")
h = defaultdict(int)
for r in rows:
    h[r["rep_type"]] += 1
for k, v in sorted(h.items(), key=lambda x: -x[1]):
    print(f"  {k:<50s} {v}")

print("\n=== DEPTH-1 COMPONENTS (direct children of root 'Assembly') ===")
print(f"{'name':<38s} {'x':>10s} {'y':>10s} {'z':>10s} {'r_xy':>9s} "
      f"{'ang_deg':>9s}  rot")
for r in rows:
    if r["depth"] != 1:
        continue
    x, y, z = r["xyz"]
    rad = math.hypot(x, y)
    ang = math.degrees(math.atan2(y, x))
    R = r["R"]
    ident = all(abs(R[i][j] - (1.0 if i == j else 0.0)) < 1e-9
                for i in range(3) for j in range(3))
    rs = "I" if ident else " ".join(f"{R[i][j]:+.3f}" for i in range(3)
                                    for j in range(3))
    print(f"{r['name']:<38s} {x:10.3f} {y:10.3f} {z:10.3f} {rad:9.3f} "
          f"{ang:9.3f}  {rs}")

print("\n=== depth-1 subtree sizes ===")
top = [r for r in rows if r["depth"] == 1]
for t in top:
    n = sum(1 for r in rows if r["path"].startswith(t["path"] + "/"))
    print(f"  {t['name']:<40s} descendants={n}")

print("\n=== unique names containing keywords ===")
kw = ["lidar", "rplidar", "c1", "batt", "pi_", "pi5", "arm", "so101", "so-101",
      "wheel", "omni", "roller", "motor", "servo", "sts", "plate", "stand"]
seen = set()
for r in rows:
    ln = r["name"].lower()
    for k in kw:
        if k in ln and r["name"] not in seen:
            seen.add(r["name"])
            print(f"  d{r['depth']} {r['name']:<44s} path={r['path'][:110]}")
            break

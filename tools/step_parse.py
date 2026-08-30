"""Parse the STEP AP214 assembly: product tree, placement transforms, units.

Pure-python; no CAD kernel needed. Reads the entity graph directly so the
assembly structure is taken from NEXT_ASSEMBLY_USAGE_OCCURRENCE rather than
inferred from a tessellated round-trip.
"""
import re
import sys
import math
import json
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else "mmr_bot.step"


def split_statements(text):
    """Split DATA section into statements on top-level semicolons (quote-aware)."""
    out = []
    buf = []
    in_str = False
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if in_str:
            if c == "'":
                if i + 1 < n and text[i + 1] == "'":
                    buf.append("''")
                    i += 2
                    continue
                in_str = False
            buf.append(c)
        else:
            if c == "'":
                in_str = True
                buf.append(c)
            elif c == ";":
                out.append("".join(buf))
                buf = []
            else:
                buf.append(c)
        i += 1
    if buf:
        out.append("".join(buf))
    return out


def strip_comments(text):
    return re.sub(r"/\*.*?\*/", " ", text, flags=re.DOTALL)


ENT_RE = re.compile(r"^\s*#(\d+)\s*=\s*(.*)$", re.DOTALL)
TYPE_RE = re.compile(r"^\s*([A-Z_0-9]+)\s*\((.*)\)\s*$", re.DOTALL)


def parse_args(s):
    """Split an argument list on top-level commas."""
    args = []
    depth = 0
    in_str = False
    buf = []
    i = 0
    n = len(s)
    while i < n:
        c = s[i]
        if in_str:
            if c == "'":
                if i + 1 < n and s[i + 1] == "'":
                    buf.append("''")
                    i += 2
                    continue
                in_str = False
            buf.append(c)
        else:
            if c == "'":
                in_str = True
                buf.append(c)
            elif c == "(":
                depth += 1
                buf.append(c)
            elif c == ")":
                depth -= 1
                buf.append(c)
            elif c == "," and depth == 0:
                args.append("".join(buf).strip())
                buf = []
            else:
                buf.append(c)
        i += 1
    args.append("".join(buf).strip())
    return args


def load(path):
    with open(path, "r", errors="replace") as f:
        raw = f.read()
    di = raw.index("DATA;")
    body = raw[di + 5:]
    ei = body.rfind("ENDSEC;")
    body = body[:ei]
    body = strip_comments(body)

    ents = {}          # id -> (TYPE, [args])  for simple entities
    complex_ents = {}  # id -> raw string      for  ( A() B() ) style
    for st in split_statements(body):
        m = ENT_RE.match(st)
        if not m:
            continue
        eid = int(m.group(1))
        rhs = m.group(2).strip()
        if rhs.startswith("("):
            complex_ents[eid] = rhs
            continue
        tm = TYPE_RE.match(rhs)
        if not tm:
            continue
        ents[eid] = (tm.group(1), parse_args(tm.group(2)))
    return ents, complex_ents, raw[:raw.index("DATA;")]


def ref(a):
    a = a.strip()
    return int(a[1:]) if a.startswith("#") else None


def refs(a):
    return [int(x) for x in re.findall(r"#(\d+)", a)]


def unquote(a):
    a = a.strip()
    if a.startswith("'") and a.endswith("'"):
        return a[1:-1].replace("''", "'")
    return a


def fnum(a):
    return float(a.strip())


def main():
    ents, cplx, header = load(PATH)
    print(f"parsed {len(ents)} simple + {len(cplx)} complex entities")

    by_type = defaultdict(list)
    for eid, (t, args) in ents.items():
        by_type[t].append(eid)

    # ---------------- units ----------------
    print("\n=== UNITS ===")
    unit_ids = set()
    for eid, raw in cplx.items():
        if "SI_UNIT" in raw and "LENGTH_UNIT" in raw:
            unit_ids.add(eid)
            print(f"  #{eid} = {' '.join(raw.split())[:160]}")
    for eid, raw in cplx.items():
        if "PLANE_ANGLE_UNIT" in raw and "SI_UNIT" in raw:
            print(f"  #{eid} = {' '.join(raw.split())[:160]}")
    for eid in by_type.get("CONVERSION_BASED_UNIT", []):
        print(f"  CONVERSION_BASED_UNIT #{eid} = {ents[eid][1]}")
    for eid in by_type.get("UNCERTAINTY_MEASURE_WITH_UNIT", [])[:3]:
        print(f"  UNCERTAINTY #{eid} = {ents[eid][1]}")

    # ---------------- products ----------------
    # PRODUCT( id, name, description, frame_of_reference )
    products = {}
    for eid in by_type.get("PRODUCT", []):
        a = ents[eid][1]
        products[eid] = unquote(a[0])

    # PRODUCT_DEFINITION_FORMATION -> PRODUCT
    pdf_to_product = {}
    for t in ("PRODUCT_DEFINITION_FORMATION",
              "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE"):
        for eid in by_type.get(t, []):
            a = ents[eid][1]
            pdf_to_product[eid] = ref(a[2])

    # PRODUCT_DEFINITION( id, desc, formation, frame )
    pd_to_product = {}
    for eid in by_type.get("PRODUCT_DEFINITION", []):
        a = ents[eid][1]
        f = ref(a[2])
        pd_to_product[eid] = pdf_to_product.get(f)

    def pd_name(pd):
        p = pd_to_product.get(pd)
        return products.get(p, f"<pd#{pd}>")

    print(f"\n=== PRODUCTS ({len(products)}) ===")
    counts = defaultdict(int)
    for pd in pd_to_product:
        counts[pd_name(pd)] += 1
    for nm in sorted(counts):
        print(f"  {nm}   (product_definitions: {counts[nm]})")

    # ---------------- assembly structure ----------------
    # NEXT_ASSEMBLY_USAGE_OCCURRENCE( id, name, desc, relating_pd, related_pd, ref )
    nauo = {}
    for eid in by_type.get("NEXT_ASSEMBLY_USAGE_OCCURRENCE", []):
        a = ents[eid][1]
        nauo[eid] = {
            "id": unquote(a[0]),
            "name": unquote(a[1]),
            "parent": ref(a[3]),
            "child": ref(a[4]),
        }
    print(f"\n=== NEXT_ASSEMBLY_USAGE_OCCURRENCE: {len(nauo)} ===")

    # PRODUCT_DEFINITION_SHAPE( name, desc, definition ) -> pd or nauo
    pds_to_def = {}
    for eid in by_type.get("PRODUCT_DEFINITION_SHAPE", []):
        a = ents[eid][1]
        pds_to_def[eid] = ref(a[2])

    # AXIS2_PLACEMENT_3D
    def cart(eid):
        a = ents[eid][1]
        vals = re.findall(r"[-+0-9.eE]+", a[1])
        return [float(v) for v in vals[:3]]

    def direc(eid):
        a = ents[eid][1]
        vals = re.findall(r"[-+0-9.eE]+", a[1])
        return [float(v) for v in vals[:3]]

    def axis_placement(eid):
        t, a = ents[eid]
        origin = cart(ref(a[1]))
        z = direc(ref(a[2])) if ref(a[2]) else [0, 0, 1]
        x = direc(ref(a[3])) if len(a) > 3 and ref(a[3]) else None
        return origin, z, x

    def norm(v):
        n = math.sqrt(sum(c * c for c in v))
        return [c / n for c in v] if n else v

    def cross(a, b):
        return [a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]]

    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    def mat_from_placement(eid):
        o, z, x = axis_placement(eid)
        z = norm(z)
        if x is None:
            x = [1, 0, 0] if abs(z[0]) < 0.9 else [0, 1, 0]
        x = [x[i] - dot(x, z) * z[i] for i in range(3)]
        x = norm(x)
        y = cross(z, x)
        # column-major: R columns are x, y, z
        return [[x[0], y[0], z[0], o[0]],
                [x[1], y[1], z[1], o[1]],
                [x[2], y[2], z[2], o[2]],
                [0, 0, 0, 1]]

    def matmul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
                for i in range(4)]

    def matinv(A):
        R = [[A[i][j] for j in range(3)] for i in range(3)]
        t = [A[i][3] for i in range(3)]
        Rt = [[R[j][i] for j in range(3)] for i in range(3)]
        ti = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
        return [[Rt[0][0], Rt[0][1], Rt[0][2], ti[0]],
                [Rt[1][0], Rt[1][1], Rt[1][2], ti[1]],
                [Rt[2][0], Rt[2][1], Rt[2][2], ti[2]],
                [0, 0, 0, 1]]

    # ITEM_DEFINED_TRANSFORMATION( name, desc, axis1, axis2 )
    idt = {}
    for eid in by_type.get("ITEM_DEFINED_TRANSFORMATION", []):
        a = ents[eid][1]
        idt[eid] = (ref(a[2]), ref(a[3]))

    # complex: ( REPRESENTATION_RELATIONSHIP(...)
    #            REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#idt)
    #            SHAPE_REPRESENTATION_RELATIONSHIP() )
    rrwt = {}
    for eid, raw in cplx.items():
        if "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION" in raw:
            m = re.search(
                r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)", raw)
            allrefs = refs(raw)
            rrwt[eid] = {"idt": int(m.group(1)) if m else None,
                         "refs": allrefs}

    # CONTEXT_DEPENDENT_SHAPE_REPRESENTATION( rep_rel, represented_product_relation )
    cdsr = []
    for eid in by_type.get("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION", []):
        a = ents[eid][1]
        cdsr.append((eid, ref(a[0]), ref(a[1])))
    print(f"=== CONTEXT_DEPENDENT_SHAPE_REPRESENTATION: {len(cdsr)} ===")

    # nauo id -> transform (child frame expressed in parent frame)
    nauo_tf = {}
    for eid, rel, rps in cdsr:
        target = pds_to_def.get(rps)
        if target not in nauo:
            continue
        info = rrwt.get(rel)
        if not info or info["idt"] is None:
            continue
        a1, a2 = idt[info["idt"]]
        # transform maps rep1(child/parent per rr order) into rep2
        M1 = mat_from_placement(a1)
        M2 = mat_from_placement(a2)
        nauo_tf[target] = {"M1": M1, "M2": M2,
                           "T": matmul(M2, matinv(M1)),
                           "T_alt": matmul(M1, matinv(M2)),
                           "raw_refs": info["refs"]}

    print(f"=== NAUO with transform: {len(nauo_tf)} / {len(nauo)} ===")

    # ---------------- build tree ----------------
    children = defaultdict(list)
    all_children = set()
    for eid, d in nauo.items():
        children[d["parent"]].append(eid)
        all_children.add(d["child"])

    roots = [pd for pd in pd_to_product if pd not in all_children]
    print(f"\n=== ROOT product_definitions: {len(roots)} ===")
    for r in roots:
        print(f"  #{r}  {pd_name(r)}   children={len(children.get(r, []))}")

    # shape representation attached to each pd (to know if it has own geometry)
    # SHAPE_DEFINITION_REPRESENTATION( definition(pds), used_representation )
    pd_has_shape = {}
    for eid in by_type.get("SHAPE_DEFINITION_REPRESENTATION", []):
        a = ents[eid][1]
        pds = ref(a[0])
        rep = ref(a[1])
        d = pds_to_def.get(pds)
        if d in pd_to_product:
            pd_has_shape[d] = rep

    rep_type = {}
    for t in ("ADVANCED_BREP_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION",
              "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
              "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION"):
        for eid in by_type.get(t, []):
            rep_type[eid] = t

    out_rows = []

    def walk(pd, M, path, depth, nauo_id=None):
        name = pd_name(pd)
        rep = pd_has_shape.get(pd)
        rt = rep_type.get(rep, "-")
        out_rows.append({
            "depth": depth, "name": name, "path": path, "pd": pd,
            "nauo": nauo_id,
            "rep": rep, "rep_type": rt,
            "xyz": [M[0][3], M[1][3], M[2][3]],
            "R": [[M[i][j] for j in range(3)] for i in range(3)],
        })
        for ch in children.get(pd, []):
            d = nauo[ch]
            T = nauo_tf.get(ch, {}).get("T")
            if T is None:
                T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            walk(d["child"], matmul(M, T), path + "/" + pd_name(d["child"]),
                 depth + 1, ch)

    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for r in roots:
        walk(r, I, pd_name(r), 0)

    print(f"\n=== ASSEMBLY TREE ({len(out_rows)} nodes) ===")
    for row in out_rows:
        ind = "  " * row["depth"]
        x, y, z = row["xyz"]
        geo = "GEOM" if row["rep"] and "BREP" in row["rep_type"] else (
            "asm " if row["rep"] else "  -  ")
        print(f"{ind}{row['name']:<44s} [{geo}] "
              f"xyz=({x:10.3f},{y:10.3f},{z:10.3f})")

    with open("tools/tree.json", "w") as f:
        json.dump(out_rows, f, indent=1)
    print("\nwrote tools/tree.json")


main()

"""A small standalone xacro expander, so the package can be checked offline.

This machine has no ROS 2, so `xacro` cannot be run here. This implements the
subset of xacro that mmr_pkg actually uses -- includes, properties, ${}
expressions, macros with default and block parameters, and xacro:if -- purely
so the description can be expanded and inspected before it reaches a ROS box.

It is a DEVELOPMENT AID, not a substitute for the real thing. The real
`xacro` remains the authority; run it on the ROS machine.

    python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro -o /tmp/robot.urdf
"""
import argparse
import math
import os
import re
import xml.etree.ElementTree as ET

XNS = "http://www.ros.org/wiki/xacro"
X = "{%s}" % XNS
EXPR = re.compile(r"\$\{([^}]*)\}")
FIND = re.compile(r"\$\(find\s+([^)]+)\)")

FUNCS = {k: getattr(math, k) for k in
         ("sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sqrt",
          "radians", "degrees", "floor", "ceil", "fabs", "exp", "log")}
FUNCS.update(pi=math.pi, e=math.e, abs=abs, min=min, max=max, round=round,
             int=int, float=float)
FUNCS.update({"True": True, "False": False, "true": True, "false": False})


class XacroError(Exception):
    pass


def coerce(v):
    if isinstance(v, (int, float, bool)):
        return v
    s = str(v).strip()
    for cast in (int, float):
        try:
            return cast(s)
        except (TypeError, ValueError):
            pass
    return v


def truthy(v):
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("false", "0", ""):
            return False
        if s in ("true", "1"):
            return True
    return bool(v)


def evaluate(text, scope):
    """Expand every ${...} in `text`. Returns a str, or a scalar if the whole
    string was a single expression (so numbers stay numbers through macros)."""
    if text is None:
        return None
    m = EXPR.fullmatch(text.strip())
    if m:
        return _eval_one(m.group(1), scope)

    def sub(mo):
        return str(_eval_one(mo.group(1), scope))
    out = EXPR.sub(sub, text)
    return re.sub(r"\s+", " ", out).strip() if "\n" in out else out


def _eval_one(expr, scope):
    expr = expr.strip()
    if expr in scope:                      # bare property, no arithmetic
        return scope[expr]
    env = dict(FUNCS)
    env.update({k: v for k, v in scope.items() if k.isidentifier()})
    try:
        return eval(expr, {"__builtins__": {}}, env)      # noqa: S307
    except Exception as exc:
        raise XacroError(f"cannot evaluate ${{{expr}}}: {exc}") from exc


def resolve_find(path, pkg_root, pkg_name):
    def sub(mo):
        if mo.group(1).strip() != pkg_name:
            raise XacroError(f"$(find {mo.group(1)}) - only {pkg_name} is "
                             "resolvable offline")
        return pkg_root
    return FIND.sub(sub, path)


class Xacro:
    def __init__(self, pkg_root, pkg_name):
        self.pkg_root = os.path.abspath(pkg_root)
        self.pkg_name = pkg_name
        self.props = {}
        self.macros = {}
        self.includes = []

    # ---------------------------------------------------------------- load
    def load(self, path):
        root = ET.parse(path).getroot()
        self._inline_includes(root, os.path.dirname(os.path.abspath(path)))
        return root

    def _inline_includes(self, elem, base):
        out = []
        for child in list(elem):
            if child.tag == X + "include":
                fn = resolve_find(child.get("filename"), self.pkg_root,
                                  self.pkg_name)
                fn = fn if os.path.isabs(fn) else os.path.join(base, fn)
                self.includes.append(os.path.relpath(fn, self.pkg_root))
                sub = ET.parse(fn).getroot()
                self._inline_includes(sub, os.path.dirname(fn))
                out.extend(list(sub))
            else:
                self._inline_includes(child, base)
                out.append(child)
        elem[:] = out

    # -------------------------------------------------------------- expand
    def expand(self, root):
        out = ET.Element(root.tag, dict(root.attrib))
        self._walk(root, out, dict(self.props))
        return out

    def _walk(self, src, dst, scope):
        for e in list(src):
            t = e.tag

            if t == X + "property":
                name = e.get("name")
                if "value" in e.attrib:
                    scope[name] = coerce(evaluate(e.get("value"), scope))
                else:
                    scope[name] = list(e)
                self.props[name] = scope[name]

            elif t == X + "macro":
                self.macros[e.get("name")] = (e.get("params", ""), e)

            elif t == X + "if" or t == X + "unless":
                cond = truthy(evaluate(e.get("value"), scope))
                if (t == X + "if") == cond:
                    self._walk(e, dst, scope)

            elif t == X + "insert_block":
                for b in scope.get("*" + e.get("name"), []):
                    self._emit(b, dst, scope)

            elif t.startswith(X):
                name = t[len(X):]
                if name not in self.macros:
                    raise XacroError(f"unknown macro or tag: xacro:{name}")
                self._call(name, e, dst, scope)

            else:
                self._emit(e, dst, scope)

    def _emit(self, e, dst, scope):
        node = ET.SubElement(dst, e.tag, {
            k: str(evaluate(v, scope)) for k, v in e.attrib.items()})
        if e.text and e.text.strip():
            node.text = str(evaluate(e.text, scope))
        self._walk(e, node, scope)

    def _call(self, name, call, dst, scope):
        spec, body = self.macros[name]
        inner = dict(scope)
        blocks = [p for p in spec.split() if p.startswith("*")]
        for p in spec.split():
            if p.startswith("*"):
                continue
            if ":=" in p:
                key, default = p.split(":=", 1)
                inner[key] = coerce(evaluate(call.get(key, default), scope))
            else:
                if call.get(p) is None:
                    raise XacroError(f"macro {name}: missing parameter '{p}'")
                inner[p] = coerce(evaluate(call.get(p), scope))
        kids = list(call)
        if blocks:
            if len(kids) < len(blocks):
                raise XacroError(f"macro {name}: expected {len(blocks)} block "
                                 f"parameter(s), got {len(kids)}")
            for i, b in enumerate(blocks):
                inner["*" + b[1:]] = [kids[i]]
        self._walk(body, dst, inner)


def indent(e, level=0):
    pad = "\n" + "  " * level
    if len(e):
        if not (e.text or "").strip():
            e.text = pad + "  "
        for c in e:
            indent(c, level + 1)
        if not (c.tail or "").strip():
            c.tail = pad
    if level and not (e.tail or "").strip():
        e.tail = pad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xacro")
    ap.add_argument("-o", "--out", default="-")
    ap.add_argument("--pkg-root", default=".")
    ap.add_argument("--pkg-name", default="mmr_pkg")
    a = ap.parse_args()

    ET.register_namespace("xacro", XNS)
    x = Xacro(a.pkg_root, a.pkg_name)
    root = x.load(a.xacro)
    out = x.expand(root)
    for k in list(out.attrib):
        if k.startswith("xmlns"):
            del out.attrib[k]
    indent(out)
    text = '<?xml version="1.0"?>\n' + ET.tostring(out, encoding="unicode")

    if a.out == "-":
        print(text)
    else:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print(f"expanded {a.xacro} -> {a.out}")
        print(f"  includes:   {', '.join(x.includes)}")
        print(f"  properties: {len(x.props)}   macros: {len(x.macros)}")


if __name__ == "__main__":
    main()

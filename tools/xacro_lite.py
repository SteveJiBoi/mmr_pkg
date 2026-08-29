"""A small standalone xacro expander, so the package can be checked offline.

This machine has no ROS 2, so `xacro` cannot be run here. This implements the
subset of xacro that mmr_pkg actually uses -- includes, properties, ${}
expressions, macros with default and block parameters, xacro:if/unless, and
xacro:arg with $(arg ...) -- purely so the description can be expanded and
inspected before it reaches a ROS box.

It is a DEVELOPMENT AID, not a substitute for the real thing. The real
`xacro` remains the authority; run it on the ROS machine.

    python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro -o /tmp/robot.urdf
    python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro gazebo:=false -o -
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
ARG = re.compile(r"\$\(arg\s+([^)]+)\)")

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
    def __init__(self, pkg_root, pkg_name, cli_args=None):
        self.pkg_root = os.path.abspath(pkg_root)
        self.pkg_name = pkg_name
        self.props = {}
        self.macros = {}
        self.includes = []
        self.cli_args = dict(cli_args or {})
        self.args = {}

    # ---------------------------------------------------------------- load
    def load(self, path):
        root = ET.parse(path).getroot()
        self._inline_includes(root, os.path.dirname(os.path.abspath(path)))
        self._resolve_args(root)
        return root

    def _resolve_args(self, root):
        """Handle <xacro:arg> and $(arg name).

        Args are GLOBAL in xacro, not lexically scoped like properties, so a
        single flat pass over the already-inlined tree is faithful. Order does
        not matter: every declaration is collected before any substitution.

        Known gap vs real xacro: $(arg ...) is resolved AFTER includes have
        been inlined, so it cannot be used inside an <xacro:include filename>.
        Nothing in this package does that; if that ever changes, this is where
        it would need fixing.
        """
        for parent in root.iter():
            for child in list(parent):
                if child.tag == X + "arg":
                    name = child.get("name")
                    if name is None:
                        raise XacroError("<xacro:arg> without a name")
                    if name in self.cli_args:
                        self.args[name] = self.cli_args[name]
                    elif "default" in child.attrib:
                        self.args[name] = child.get("default")
                    else:
                        raise XacroError(
                            f"$(arg {name}) has no default and was not passed "
                            f"on the command line as {name}:=value")
                    parent.remove(child)

        unknown = set(self.cli_args) - set(self.args)
        if unknown:
            raise XacroError(f"passed {sorted(unknown)} on the command line "
                             f"but no <xacro:arg> declares them")

        def sub(mo):
            name = mo.group(1).strip()
            if name not in self.args:
                raise XacroError(f"$(arg {name}) is not declared by any "
                                 f"<xacro:arg>")
            return self.args[name]

        # $(find pkg) is resolved here too, not just in <xacro:include
        # filename>. Real xacro substitutes it in element TEXT as well as in
        # attributes - gazebo.xacro relies on that for
        # <parameters>$(find mmr_pkg)/config/controllers.yaml</parameters>.
        # Leaving it unresolved here would make this expander quietly disagree
        # with the real one, which is the one thing it must not do.
        def find(v):
            return resolve_find(v, self.pkg_root, self.pkg_name)

        for e in root.iter():
            for k, v in list(e.attrib.items()):
                if "$(arg" in v:
                    v = ARG.sub(sub, v)
                if "$(find" in v:
                    v = find(v)
                e.attrib[k] = v
            if e.text:
                if "$(arg" in e.text:
                    e.text = ARG.sub(sub, e.text)
                if "$(find" in e.text:
                    e.text = find(e.text)

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
    ap.add_argument("mappings", nargs="*", metavar="name:=value",
                    help="override an <xacro:arg>, same syntax as real xacro")
    a = ap.parse_args()

    cli = {}
    for m in a.mappings:
        if ":=" not in m:
            ap.error(f"mapping {m!r} is not of the form name:=value")
        k, v = m.split(":=", 1)
        cli[k] = v

    ET.register_namespace("xacro", XNS)
    # Keep the `gz:` prefix literally `gz:`. ElementTree otherwise invents
    # ns0:, ns1:, ... which is semantically identical XML but NOT equivalent
    # here: sdformat parses with TinyXML2, which does no namespace resolution
    # at all and matches the attribute name as a raw string. `ns0:expressed_in`
    # would be silently ignored, and the omniwheel friction would quietly
    # revert to whatever the default fdir1 interpretation is. Real xacro
    # preserves prefixes; this keeps the offline output faithful to it.
    ET.register_namespace("gz", "http://gazebosim.org/schema")
    x = Xacro(a.pkg_root, a.pkg_name, cli)
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
        print(f"  args:       {x.args or '(none)'}")
        print(f"  properties: {len(x.props)}   macros: {len(x.macros)}")


if __name__ == "__main__":
    main()

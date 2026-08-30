#!/usr/bin/env python3
"""Static checks that need no ROS, no hardware and no build.

    python tools/check_repo.py

Every check here exists because the corresponding mistake was actually made in
this repository, not because it seemed like a good idea. This runs on a laptop
in under a second, which is the point: the things it catches are things that
otherwise surface as a node that will not start on the robot.

Exits non-zero if anything fails, so it can go in front of a commit.

It does NOT replace colcon, rosdep, or driving the robot. Nothing here proves
that a single packet reaches the ESP32.
"""
from __future__ import annotations

import ast
import builtins
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent.parent
SKIP_DIRS = {".git", "__pycache__", ".pytest_cache", "build", "install", "log"}

failures: list[str] = []
checks_run = 0


def fail(where: str, message: str) -> None:
    failures.append(f"{where}: {message}")


def walk(pattern: str):
    for path in sorted(ROOT.rglob(pattern)):
        if SKIP_DIRS & set(path.parts):
            continue
        yield path


def rel(path: pathlib.Path) -> str:
    return path.relative_to(ROOT).as_posix()


# ---------------------------------------------------------------------- XML
def check_xml() -> None:
    """Well-formedness, and the one XML rule that keeps catching me out.

    "--" is illegal INSIDE an XML comment. Not discouraged: illegal. It is a
    natural thing to type as a visual separator or an em dash, the parser
    rejects the whole file, and the error points at a line number rather than
    saying what the problem is. This has broken ros2_control.xacro and
    package.xml in this repo on separate occasions.
    """
    global checks_run
    for path in walk("*.xml"):
        checks_run += 1
        text = path.read_text(encoding="utf-8")
        for match in re.finditer(r"<!--(.*?)-->", text, re.DOTALL):
            if "--" in match.group(1):
                line = text[:match.start()].count("\n") + 1
                fail(f"{rel(path)}:{line}",
                     "'--' inside an XML comment is illegal; use ';' or '---'")
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            fail(rel(path), f"not well-formed: {exc}")

    for path in walk("*.xacro"):
        checks_run += 1
        text = path.read_text(encoding="utf-8")
        for match in re.finditer(r"<!--(.*?)-->", text, re.DOTALL):
            if "--" in match.group(1):
                line = text[:match.start()].count("\n") + 1
                fail(f"{rel(path)}:{line}",
                     "'--' inside an XML comment is illegal")


# ------------------------------------------------------------------- Python
def check_python_syntax() -> None:
    global checks_run
    for path in walk("*.py"):
        checks_run += 1
        try:
            ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError as exc:
            fail(rel(path), f"syntax error: {exc}")


#: Attribute names that come from rclpy.node.Node rather than from our code.
_NODE_INHERITED = {
    "get_logger", "get_clock", "declare_parameter", "declare_parameters",
    "get_parameter", "get_parameters", "create_subscription",
    "create_publisher", "create_timer", "create_service", "create_client",
    "destroy_node", "get_name", "get_namespace", "add_on_set_parameters_callback",
    "context", "executor", "handle",
}


def check_self_attributes() -> None:
    """Every self.x that is READ must be assigned somewhere in the class.

    THIS IS THE CHECK THAT MATTERS MOST HERE. esp32_bridge.py once read
    self.max_v on the last line of __init__ while the value lived on a
    collaborator object, so the constructor raised AttributeError and the node
    could not start -- and no test noticed, because importing the node needs
    rclpy and the tests run without it.

    A crude approximation of a type checker, but it needs no dependency and it
    catches the exact failure that shipped.
    """
    global checks_run
    for path in walk("*.py"):
        if "test" in path.parts:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
            checks_run += 1
            stores: set[str] = set()
            loads: dict[str, int] = {}
            for node in ast.walk(cls):
                if (isinstance(node, ast.Attribute)
                        and isinstance(node.value, ast.Name)
                        and node.value.id == "self"):
                    if isinstance(node.ctx, ast.Store):
                        stores.add(node.attr)
                    else:
                        loads.setdefault(node.attr, node.lineno)

            known = set(stores) | _NODE_INHERITED
            for item in cls.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    known.add(item.name)
                elif isinstance(item, ast.Assign):
                    known |= {t.id for t in item.targets
                              if isinstance(t, ast.Name)}
                elif isinstance(item, ast.AnnAssign) and isinstance(
                        item.target, ast.Name):
                    known.add(item.target.id)
            # Inherited members of any base we did not parse.
            for base in cls.bases:
                if isinstance(base, ast.Name) and base.id not in ("object",):
                    pass

            for attr, line in sorted(loads.items(), key=lambda kv: kv[1]):
                if attr not in known:
                    fail(f"{rel(path)}:{line}",
                         f"self.{attr} is read but never assigned in "
                         f"{cls.name}")


# --------------------------------------------------------------------- YAML
def check_yaml() -> None:
    """.rviz files are YAML. They use '#' for comments, never '//'."""
    global checks_run
    try:
        import yaml
    except ImportError:
        print("  (skipped YAML checks: PyYAML not installed)")
        return
    for pattern in ("*.yaml", "*.rviz"):
        for path in walk(pattern):
            checks_run += 1
            try:
                yaml.safe_load(path.read_text(encoding="utf-8"))
            except yaml.YAMLError as exc:
                fail(rel(path), f"invalid YAML: {exc}")


# ------------------------------------------------------- packaging regressions
def check_packaging() -> None:
    """Guards for the three build failures recorded in the brief."""
    global checks_run

    # S17 bug 4. sllidar_ros2 is a workspace checkout, not a rosdep key, and
    # naming it aborts `rosdep install` before it resolves anything else.
    checks_run += 1
    pkg = ET.parse(ROOT / "package.xml").getroot()
    depends = {e.text for e in pkg.iter() if e.tag.endswith("depend")}
    if "sllidar_ros2" in depends:
        fail("package.xml",
             "sllidar_ros2 must not be a depend; it is a workspace package, "
             "not a rosdep key, and rosdep aborts on it")

    # S17 bug 2. Every script must be installed, or `ros2 run` cannot find it.
    checks_run += 1
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    for script in sorted((ROOT / "scripts").iterdir()):
        if script.is_dir() or script.name.startswith("."):
            continue
        if f"scripts/{script.name}" not in cmake:
            fail("CMakeLists.txt",
                 f"scripts/{script.name} exists but is not in "
                 f"install(PROGRAMS ...); `ros2 run` will not find it")

    # ...and every script must point at a module that exists.
    for script in sorted((ROOT / "scripts").iterdir()):
        if script.is_dir() or script.name.startswith("."):
            continue
        checks_run += 1
        text = script.read_text(encoding="utf-8")
        match = re.search(r"from\s+(mmr_pkg\.[\w.]+)\s+import\s+main", text)
        if not match:
            fail(f"scripts/{script.name}",
                 "shim does not import main from an mmr_pkg module")
            continue
        module = ROOT / (match.group(1).replace(".", "/") + ".py")
        if not module.exists():
            fail(f"scripts/{script.name}",
                 f"imports {match.group(1)}, which does not exist")

    # S17 bug 3. A source tree containing build/ collides with colcon's own
    # directory and produces the symlink-vs-directory failure.
    checks_run += 1
    if (ROOT / "build").exists():
        fail("build/",
             "a build/ directory in the source tree collides with colcon; "
             "the generated URDF lives in generated/")

    # Every test file named in CMakeLists must exist, and vice versa.
    checks_run += 1
    declared = set(re.findall(r"ament_add_pytest_test\(\S+\s+(test/\S+?)\)",
                              cmake))
    present = {rel(p) for p in walk("test_*.py")}
    for missing in sorted(declared - present):
        fail("CMakeLists.txt", f"{missing} is registered but does not exist")
    for unregistered in sorted(present - declared):
        fail("CMakeLists.txt",
             f"{unregistered} exists but is not registered with "
             f"ament_add_pytest_test, so `colcon test` will skip it")


# ------------------------------------------------------------------- launch
def check_launch_names() -> None:
    """Catches a name used in a launch file but never imported.

    A launch file is only executed when you launch it, so a missing import is
    an error you discover on the robot rather than at build time.
    """
    global checks_run
    for path in walk("*.launch.py"):
        checks_run += 1
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                imported |= {a.asname or a.name for a in node.names}
            elif isinstance(node, ast.Import):
                imported |= {(a.asname or a.name).split(".")[0]
                             for a in node.names}

        # `builtins`, not `__builtins__`: the latter is the module when this
        # file is run as a script but a plain dict when it is imported, and
        # dir() of that dict returns dict methods -- so every use of print or
        # int would be reported as undefined. Only shows up when imported.
        bound = set(imported) | set(dir(builtins)) | {"__name__", "__file__"}
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                bound.add(node.name)
            elif isinstance(node, ast.arg):
                bound.add(node.arg)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                bound.add(node.id)
            elif isinstance(node, ast.ExceptHandler) and node.name:
                bound.add(node.name)

        for node in ast.walk(tree):
            if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load):
                if node.id not in bound:
                    fail(f"{rel(path)}:{node.lineno}",
                         f"{node.id} is used but never imported or defined")


def main() -> int:
    print("static checks (no ROS, no hardware)\n")
    for name, check in [
        ("xml well-formed and comment-legal", check_xml),
        ("python syntax", check_python_syntax),
        ("self attributes assigned", check_self_attributes),
        ("yaml and rviz parse", check_yaml),
        ("packaging regressions", check_packaging),
        ("launch file names resolve", check_launch_names),
    ]:
        before = len(failures)
        check()
        status = "FAIL" if len(failures) > before else "ok"
        print(f"  {status:4}  {name}")

    print()
    if failures:
        for line in failures:
            print(f"  FAIL  {line}")
        print(f"\n=== {len(failures)} failures over {checks_run} checks ===")
        return 1
    print(f"=== 0 failures over {checks_run} checks ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())

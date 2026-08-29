#!/usr/bin/env python3
"""Offline consistency check for the Phase 2 control wiring.

    python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro -o build/robot.urdf
    python tools/check_phase2.py build/robot.urdf config/controllers.yaml

WHY THIS EXISTS
---------------
Phase 2 spreads one fact - "which joint is driven by which number" - across
four files that no compiler ever compares:

    urdf/ros2_control.xacro     declares the interfaces
    config/controllers.yaml     declares the joint ORDER inside a
                                Float64MultiArray that carries no names
    mmr_pkg/kiwi_kinematics.py  pairs wheel i with a mounting angle
    mmr_pkg/kiwi_drive_node.py  emits the array

Every disagreement between them is silent at every stage: ros2_control loads,
the controller loads, the node publishes, and the robot drives off at the
wrong heading. That is the whole reason for this script.

It runs entirely offline. It does NOT prove the robot works - only Gazebo can
do that - it proves the four files agree with each other.
"""
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

# Strings verified against upstream on the branches that pair with
# Jazzy/Harmonic. If ros2_control ever renames these, this check is where the
# rename should be noticed.
EXPECTED_TYPES = {
    "joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
    "wheel_velocity_controller": "velocity_controllers/JointGroupVelocityController",
    "arm_position_controller": "position_controllers/JointGroupPositionController",
}
EXPECTED_GZ_SYSTEM_PLUGIN = "gz_ros2_control/GazeboSimSystem"
EXPECTED_GZ_MODEL_PLUGIN = ("gz_ros2_control-system",
                            "gz_ros2_control::GazeboSimROS2ControlPlugin")

fails, warns = [], []


def bad(msg):
    fails.append(msg)
    print(f"  FAIL  {msg}")


def warn(msg):
    warns.append(msg)
    print(f"  WARN  {msg}")


def ok(msg):
    print(f"  ok    {msg}")


def main(urdf_path, yaml_path):
    root = ET.parse(urdf_path).getroot()

    urdf_joints = {j.get("name"): j for j in root.findall("joint")}
    urdf_links = {l.get("name") for l in root.findall("link")}

    try:
        import yaml
    except ImportError:
        print("PyYAML not installed:  pip install pyyaml")
        return 2
    with open(yaml_path) as fh:
        cfg = yaml.safe_load(fh)

    # ---------------------------------------------------------------- 1
    print("--- ros2_control block ---")
    r2c = root.findall("ros2_control")
    if len(r2c) != 1:
        bad(f"expected exactly 1 <ros2_control>, found {len(r2c)}")
        return report()
    block = r2c[0]

    plugins = [p.text.strip() for p in block.findall("hardware/plugin")]
    if plugins == [EXPECTED_GZ_SYSTEM_PLUGIN]:
        ok(f"hardware plugin is {EXPECTED_GZ_SYSTEM_PLUGIN}")
    else:
        bad(f"hardware plugin is {plugins}, expected "
            f"['{EXPECTED_GZ_SYSTEM_PLUGIN}']")

    iface = {}
    for j in block.findall("joint"):
        name = j.get("name")
        cmds = sorted(c.get("name") for c in j.findall("command_interface"))
        states = sorted(s.get("name") for s in j.findall("state_interface"))
        iface[name] = (cmds, states)
        if name not in urdf_joints:
            bad(f"ros2_control drives '{name}' which is not a joint in the URDF")
        elif urdf_joints[name].get("type") == "fixed":
            bad(f"ros2_control drives '{name}' which is a FIXED joint")
    ok(f"{len(iface)} joints declared in <ros2_control>, all present in URDF")

    movable = {n for n, j in urdf_joints.items()
               if j.get("type") in ("revolute", "continuous", "prismatic")}
    missing = movable - set(iface)
    if missing:
        warn(f"movable joints with no ros2_control interface: {sorted(missing)}")
    else:
        ok(f"every one of the {len(movable)} movable joints is claimed")

    # ---------------------------------------------------------------- 2
    print("\n--- controllers.yaml <-> ros2_control ---")
    cm = cfg.get("controller_manager", {}).get("ros__parameters", {})
    declared = {k: v["type"] for k, v in cm.items()
                if isinstance(v, dict) and "type" in v}

    for name, want in EXPECTED_TYPES.items():
        got = declared.get(name)
        if got is None:
            bad(f"controller '{name}' is not declared in controllers.yaml")
        elif got != want:
            bad(f"controller '{name}' type is '{got}', expected '{want}'")
        else:
            ok(f"{name:26s} {got}")

    rate = cm.get("update_rate")
    if not isinstance(rate, int) or rate <= 0:
        bad(f"controller_manager update_rate is {rate!r}")
    elif 1000 % rate:
        warn(f"update_rate {rate} Hz does not divide the default 1000 Hz "
             f"physics rate evenly")
    else:
        ok(f"update_rate {rate} Hz divides 1000 Hz physics evenly")

    # Interface kind each controller requires.
    needs = {"wheel_velocity_controller": "velocity",
             "arm_position_controller": "position"}
    for ctrl, kind in needs.items():
        joints = cfg.get(ctrl, {}).get("ros__parameters", {}).get("joints", [])
        if not joints:
            bad(f"{ctrl} has an empty 'joints' list")
            continue
        for j in joints:
            if j not in iface:
                bad(f"{ctrl} claims '{j}', which has no <ros2_control> entry")
            elif kind not in iface[j][0]:
                bad(f"{ctrl} needs a '{kind}' command_interface on '{j}', "
                    f"which only offers {iface[j][0]}")
        else:
            ok(f"{ctrl}: all {len(joints)} joints expose a "
               f"'{kind}' command_interface")

    # ---------------------------------------------------------------- 3
    print("\n--- wheel ORDER (the silent one) ---")
    from mmr_pkg.kiwi_kinematics import (DEFAULT_BASE_RADIUS,
                                         DEFAULT_WHEEL_ANGLES_DEG,
                                         DEFAULT_WHEEL_RADIUS)
    yaml_wheels = cfg["wheel_velocity_controller"]["ros__parameters"]["joints"]

    if len(yaml_wheels) != len(DEFAULT_WHEEL_ANGLES_DEG):
        bad(f"controllers.yaml lists {len(yaml_wheels)} wheels but "
            f"kiwi_kinematics has {len(DEFAULT_WHEEL_ANGLES_DEG)} angles")
    else:
        import math
        for i, (jname, angle) in enumerate(
                zip(yaml_wheels, DEFAULT_WHEEL_ANGLES_DEG)):
            j = urdf_joints.get(jname)
            if j is None:
                bad(f"index {i}: '{jname}' is not a URDF joint")
                continue
            xyz = [float(v) for v in j.find("origin").get("xyz").split()]
            actual = math.degrees(math.atan2(xyz[1], xyz[0])) % 360.0
            if abs((actual - angle + 180) % 360 - 180) > 0.01:
                bad(f"index {i}: controllers.yaml/kiwi say '{jname}' is at "
                    f"{angle} deg, but the URDF mounts it at {actual:.3f} deg")
            else:
                ok(f"index {i}: {jname:15s} URDF bearing {actual:7.3f} deg "
                   f"== kiwi angle {angle} deg")

        r = [float(v) for v in
             urdf_joints[yaml_wheels[0]].find("origin").get("xyz").split()]
        L = math.hypot(r[0], r[1])
        if abs(L - DEFAULT_BASE_RADIUS) > 1e-6:
            bad(f"base_radius: URDF {L:.6f} m vs kiwi "
                f"{DEFAULT_BASE_RADIUS:.6f} m")
        else:
            ok(f"base_radius {L:.6f} m matches kiwi_kinematics")

        wl = urdf_joints[yaml_wheels[0]].get("name").replace("_joint", "_link")
        cyl = None
        for link in root.findall("link"):
            if link.get("name") == wl:
                cyl = link.find("collision/geometry/cylinder")
        if cyl is None:
            warn(f"{wl} has no cylinder collision to read a radius from")
        elif abs(float(cyl.get("radius")) - DEFAULT_WHEEL_RADIUS) > 1e-9:
            bad(f"wheel_radius: URDF {cyl.get('radius')} vs kiwi "
                f"{DEFAULT_WHEEL_RADIUS}")
        else:
            ok(f"wheel_radius {DEFAULT_WHEEL_RADIUS} m matches kiwi_kinematics")

    # ---------------------------------------------------------------- 4
    print("\n--- <gazebo> blocks ---")
    gz = root.findall("gazebo")
    if not gz:
        warn("no <gazebo> blocks; was the URDF expanded with gazebo:=true?")
    for g in gz:
        ref = g.get("reference")
        if ref is None:
            continue
        if ref not in urdf_links and ref not in urdf_joints:
            bad(f"<gazebo reference=\"{ref}\"> names no known link or joint")
    refs = [g.get("reference") for g in gz if g.get("reference")]
    if refs:
        ok(f"all {len(refs)} <gazebo reference=...> targets exist: "
           f"{sorted(set(refs))}")

    model_plugins = [(p.get("filename"), p.get("name"))
                     for g in gz if g.get("reference") is None
                     for p in g.findall("plugin")]
    if EXPECTED_GZ_MODEL_PLUGIN in model_plugins:
        ok("gz_ros2_control model plugin present with the verified "
           "filename/name pair")
    elif gz:
        bad(f"model-level gz_ros2_control plugin not found; saw "
            f"{model_plugins}")

    # The <parameters> path the plugin loads must be the file we just checked.
    for g in gz:
        for p in g.findall("plugin"):
            for par in p.findall("parameters"):
                if not par.text or "controllers.yaml" not in par.text:
                    warn(f"<parameters> points at {par.text!r}, which is not "
                         f"config/controllers.yaml")

    # ---------------------------------------------------------------- 5
    # Raw-text check, deliberately not done through ElementTree.
    #
    # sdformat parses with TinyXML2, which performs NO namespace resolution -
    # it compares attribute names as raw strings. So `gz:expressed_in` and
    # `ns0:expressed_in` are equivalent to an XML parser but NOT to sdformat:
    # the second is silently ignored and the omniwheel friction direction
    # quietly falls back to a default. Any XML library that rewrites prefixes
    # on round-trip will introduce this bug invisibly, which is exactly why
    # this check reads bytes instead of a parsed tree.
    print("\n--- gz: namespace prefix (raw text) ---")
    with open(urdf_path, encoding="utf-8") as fh:
        raw = fh.read()
    if "expressed_in" not in raw:
        warn("no expressed_in attribute found; omniwheel friction may be "
             "relying on the default fdir1 frame")
    elif "gz:expressed_in" in raw and 'xmlns:gz="http://gazebosim.org/schema"' in raw:
        ok("fdir1 uses a literal gz:expressed_in with xmlns:gz declared")
    else:
        bad("expressed_in is present but not as a literal `gz:` prefix with a "
            "matching xmlns:gz - TinyXML2 will ignore it")

    return report()


def report():
    print(f"\n=== {len(fails)} failures, {len(warns)} warnings ===")
    return 1 if fails else 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))

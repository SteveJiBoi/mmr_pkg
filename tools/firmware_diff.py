"""Diff the ESP32 firmware's wheel mixing against mmr_pkg/kiwi_kinematics.py.

    python tools/firmware_diff.py

Section 9 of the brief asked for the drive inverse kinematics to live in one
clearly commented function "so it can be diffed against the ESP32 firmware".
The firmware has since arrived (esp32/MotionTestOriginal/MotionTestOriginal.ino)
and this is that diff, automated so it stays true as either side changes.

It reads three things and cross-checks them:

  * the mounting bearings in build/robot.urdf  (the measured ground truth)
  * `wheelDeg[]` and `drive()` in the .ino     (what the robot actually runs)
  * inverse_kinematics() in kiwi_kinematics.py (what the simulation runs)

WHAT IT CANNOT DO
-----------------
It cannot tell you which physical wheel is wired to which ESP32 pin group, or
which way round each motor's leads are soldered. Neither fact is in the source
code. So when the two conventions disagree, this script reports every wiring
hypothesis that would reconcile them rather than asserting one -- and if
exactly one survives, that is a strong hint, not a measurement. Confirm it on
the robot with the procedure printed at the end.

Exit status is 0 if the two agree outright, 1 if they disagree.
"""
from __future__ import annotations

import itertools
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(ROOT, "build", "robot.urdf")
INO = os.path.join(ROOT, "esp32", "MotionTestOriginal",
                   "MotionTestOriginal.ino")

sys.path.insert(0, ROOT)
from mmr_pkg.kiwi_kinematics import KiwiKinematics    # noqa: E402


def fail(msg):
    print(f"ERROR: {msg}")
    sys.exit(2)


# --------------------------------------------------------------------- URDF
def urdf_bearings(path):
    """Mounting bearing of each wheel_N_joint, degrees CCW from +X."""
    if not os.path.exists(path):
        fail(f"{path} not found. Regenerate it with\n"
             f"  python tools/xacro_lite.py urdf/mmr_bot.urdf.xacro "
             f"-o build/robot.urdf")
    root = ET.parse(path).getroot()
    out = {}
    # findall, not iter: <ros2_control> contains <joint name="wheel_0_joint">
    # elements too, and those carry interfaces rather than an <origin>.
    for j in root.findall("joint"):
        m = re.fullmatch(r"wheel_(\d+)_joint", j.get("name", ""))
        if not m:
            continue
        origin = j.find("origin")
        x, y, _ = (float(v) for v in origin.get("xyz").split())
        out[int(m.group(1))] = math.degrees(math.atan2(y, x)) % 360
    if not out:
        fail("no wheel_N_joint found in the URDF")
    return [out[k] for k in sorted(out)]


# ----------------------------------------------------------------- firmware
def ino_facts(path):
    """Pull the load-bearing constants out of the sketch."""
    if not os.path.exists(path):
        fail(f"{path} not found")
    src = open(path, encoding="utf-8", errors="replace").read()

    m = re.search(r"wheelDeg\s*\[\s*3\s*\]\s*=\s*\{([^}]*)\}", src)
    if not m:
        fail("could not find `wheelDeg[3] = {...}` in the sketch. If the "
             "firmware was restructured, this script needs updating too.")
    deg = [float(v) for v in m.group(1).split(",")]

    # The mixing line. We are checking the SHAPE of the formula, not just the
    # numbers: if someone changes `cos` to `sin` or drops the minus on rot,
    # every table below silently becomes a lie.
    drive = re.search(r"int\s+power\s*=\s*int\s*\(\s*spd\s*\*\s*cos\s*\(\s*"
                      r"theta\s*-\s*wa\s*\)\s*\)\s*-\s*rot", src)
    theta = re.search(r"theta\s*=\s*\(\s*180\s*-\s*angleDeg\s*\)", src)

    dead = re.search(r"DEADMAN_MS\s*=\s*(\d+)", src)
    port = re.search(r"PORT\s*=\s*(\d+)", src)

    def pins(name):
        m = re.search(name + r"\s*\[\s*3\s*\]\s*=\s*\{([^}]*)\}", src)
        return [int(v) for v in m.group(1).split(",")] if m else [None] * 3

    return {
        "wheel_deg": deg,
        "mix_ok": bool(drive),
        "mirror": bool(theta),
        "deadman_ms": int(dead.group(1)) if dead else None,
        "port": int(port.group(1)) if port else None,
        "dir_pin": pins("dirPin"),
        "speed_pin": pins("speedPin"),
    }


def firmware_power(x_op, y_op, wheel_deg, mirror):
    """Reproduce drive() exactly, for a unit command in the operator frame."""
    if x_op == 0 and y_op == 0:
        return [0.0, 0.0, 0.0]
    ang = math.degrees(math.atan2(y_op, x_op)) % 360
    theta = (180.0 - ang) if mirror else ang
    return [math.cos(math.radians(theta - w)) for w in wheel_deg]


def norm(v):
    m = max(abs(t) for t in v)
    return [round(t / m, 6) if m else 0.0 for t in v]


# --------------------------------------------------------------------- main
def main():
    bearings = urdf_bearings(URDF)
    fw = ino_facts(INO)
    kin = KiwiKinematics(wheel_angles_deg=bearings)

    print("=" * 72)
    print("ESP32 firmware  vs  mmr_pkg/kiwi_kinematics.py")
    print("=" * 72)
    print(f"URDF mounting bearings   {[round(b, 3) for b in bearings]} deg "
          f"(measured, build/robot.urdf)")
    print(f"firmware wheelDeg[]      {fw['wheel_deg']} deg "
          f"(esp32 sketch)")
    print(f"firmware deadman         {fw['deadman_ms']} ms   udp port "
          f"{fw['port']}")
    print()

    if not fw["mix_ok"]:
        fail("the `power = int(spd*cos(theta - wa)) - rot` line in drive() no "
             "longer matches the expected shape. Re-read the sketch before "
             "trusting anything below.")
    if not fw["mirror"]:
        print("NOTE: the `theta = (180 - angleDeg)` mirror is gone from the "
              "sketch. Results below assume it is absent.")
        print()

    # Those two arrays are numerically identical but mean DIFFERENT THINGS:
    # the URDF number is where the wheel is BOLTED, the firmware number is the
    # direction it is assumed to PUSH. For a wheel whose axle points radially
    # outward those differ by 90 degrees, so identical arrays are already a
    # red flag rather than a reassurance.
    if [round(b) for b in bearings] == [round(d) for d in fw["wheel_deg"]]:
        print("Both arrays read 60/180/300, but they are not the same "
              "quantity:")
        print("  URDF     = where the wheel is MOUNTED (axle points radially "
              "outward)")
        print("  firmware = the direction that wheel is assumed to PUSH")
        print("  For this robot those differ by 90 deg, so equal arrays are a "
              "warning sign,")
        print("  not a match. The table below is the check that matters.")
        print()

    # ------------------------------------------------------------- the table
    # Operator frame: x = right (D), y = forward (W).
    # ROS/REP-103:    vx = forward,   vy = left.   So vx = y_op, vy = -x_op.
    keys = [("W  forward", 0, 1), ("S  back", 0, -1),
            ("D  right", 1, 0), ("A  left", -1, 0)]

    print(f"{'key':12s} {'firmware power[]':28s} {'kiwi_kinematics w[]':28s}")
    print("-" * 72)
    agree = True
    for name, x, y in keys:
        f = norm(firmware_power(x, y, fw["wheel_deg"], fw["mirror"]))
        s = norm(kin.inverse_kinematics(y, -x, 0.0))
        same = f == s
        agree &= same
        mark = "" if same else "   <-- DISAGREE"
        print(f"{name:12s} {str(f):28s} {str(s):28s}{mark}")
    print()

    if agree:
        print("RESULT: the firmware and the simulation agree on every axis.")
        return 0

    print("RESULT: they DISAGREE.")
    print()
    print("They agree on forward/back and are exactly negated on strafe. That")
    print("is a REFLECTION, and no relabelling of the robot frame can undo a")
    print("reflection -- so this is a genuine handedness difference, not a")
    print("bookkeeping one.")
    print()

    # --------------------------------------------------- wiring hypotheses
    tests = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (1, -1)]
    found = []
    for perm in itertools.permutations(range(3)):
        for sgn in itertools.product((1, -1), repeat=3):
            if all(norm([sgn[k] * firmware_power(x, y, fw["wheel_deg"],
                                                 fw["mirror"])[perm[k]]
                         for k in range(3)]) == norm(
                             kin.inverse_kinematics(y, -x, 0.0))
                   for x, y in tests):
                found.append((perm, sgn))

    print(f"Of the {6 * 8} possible wirings (3! index orders x 2^3 motor "
          f"polarities),")
    print(f"{len(found)} reconcile the firmware with the measured URDF:")
    print()
    for perm, sgn in found:
        for k in range(3):
            pol = "reversed" if sgn[k] < 0 else "normal"
            print(f"    ESP32 index {k} (dir pin {fw['dir_pin'][k]}, pwm pin "
                  f"{fw['speed_pin'][k]})  ->  URDF wheel_{perm[k]}_link, "
                  f"motor leads {pol}")
        print()

    print("CONFIRM THIS ON THE ROBOT -- it is a hypothesis, not a measurement.")
    print("The wiring is not in the source code, so it cannot be derived here.")
    print()
    print("  1. Put the robot on blocks so the wheels spin free.")
    print("  2. Send one wheel at a time. The firmware has no per-wheel")
    print("     command, so temporarily flash a sketch that calls")
    print("     setMotor(i, 120) for i = 0, 1, 2 in turn, or drive a pure")
    print("     translation along each wheel's own axis and watch which")
    print("     wheel stays still.")
    print("  3. For each i, note WHICH physical wheel turns and WHICH WAY.")
    print("     URDF wheel_0 is at bearing 60 deg, wheel_1 at 180, wheel_2 at")
    print("     300, measured CCW from the direction the lidar faces.")
    print("  4. Positive w on URDF wheel_N should push the chassis toward")
    print("     bearing (N's mounting angle - 90 deg).")
    print()
    print("One more thing worth ruling out first: 'left/right is correct' in")
    print("the firmware comment is an operator judgement. If that judgement")
    print("was made standing IN FRONT of the robot facing it, left and right")
    print("are mirrored and the firmware is the thing that is wrong.")
    return 1


if __name__ == "__main__":
    sys.exit(main())

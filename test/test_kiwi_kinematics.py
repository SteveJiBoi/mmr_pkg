"""Tests for the kiwi kinematics.

These are the guard rails on the one piece of maths in this package that can
be wrong in a way that still *looks* like it works: a sign error or a 30 degree
angle error both produce smooth, plausible holonomic motion in the wrong
direction.

Run offline with plain pytest (no ROS needed):

    python -m pytest test/test_kiwi_kinematics.py -v
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mmr_pkg.kiwi_kinematics import (  # noqa: E402
    DEFAULT_BASE_RADIUS, DEFAULT_WHEEL_ANGLES_DEG, DEFAULT_WHEEL_RADIUS,
    KiwiKinematics)

R = DEFAULT_WHEEL_RADIUS
L = DEFAULT_BASE_RADIUS


@pytest.fixture
def k():
    return KiwiKinematics()


# --------------------------------------------------------------- round trip
def test_fk_inverts_ik_exactly(k):
    """FK(IK(t)) == t for random twists. The core consistency property."""
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(2000):
        t = rng.uniform(-2.0, 2.0, 3)
        back = k.forward_kinematics(k.inverse_kinematics(*t))
        worst = max(worst, float(np.abs(np.array(back) - t).max()))
    assert worst < 1e-12, f"round-trip error {worst:.3e}"


def test_closed_form_matches_pseudoinverse(k):
    """The 120-degree closed form documented in the docstring is correct.

    If someone later hard-codes the closed form for speed, this is the test
    that catches it drifting from the general solution.
    """
    rng = np.random.default_rng(1)
    for _ in range(500):
        w = rng.uniform(-40, 40, 3)
        vx = (2 * R / 3) * sum(w[i] * math.sin(a) for i, a in enumerate(k.angles))
        vy = -(2 * R / 3) * sum(w[i] * math.cos(a) for i, a in enumerate(k.angles))
        wz = -(R / (3 * L)) * sum(w)
        assert np.allclose(k.forward_kinematics(w), (vx, vy, wz), atol=1e-12)


# ------------------------------------------------------------ sign convention
def test_all_positive_is_clockwise(k):
    """+1 rad/s on every wheel = pure CLOCKWISE rotation (negative yaw).

    The single most commonly inverted convention on an omni base.
    """
    vx, vy, wz = k.forward_kinematics([1.0, 1.0, 1.0])
    assert abs(vx) < 1e-12 and abs(vy) < 1e-12, "should be pure rotation"
    assert wz < 0, "all-positive must be CLOCKWISE (negative yaw)"
    assert wz == pytest.approx(-R / L, abs=1e-12)


def test_single_wheel_drives_at_angle_minus_90(k):
    """+1 rad/s on wheel i alone pushes the robot toward bearing a_i - 90."""
    for i, a in enumerate(k.angles):
        w = [0.0, 0.0, 0.0]
        w[i] = 1.0
        vx, vy, _ = k.forward_kinematics(w)
        got = math.degrees(math.atan2(vy, vx)) % 360
        want = (math.degrees(a) - 90.0) % 360
        assert got == pytest.approx(want, abs=1e-9), f"wheel {i}"


def test_forward_leaves_the_rear_wheel_stationary(k):
    """Driving straight forward must not turn the 180-degree wheel.

    Its spin axis lies along X, so it free-rolls sideways. This is a strong
    check on the 60/180/300 mounting angles: with the brief's 30/150/270 no
    wheel would be stationary here, so this test fails loudly if the angles
    are ever changed back without also changing the firmware.
    """
    w = k.inverse_kinematics(1.0, 0.0, 0.0)
    rear = [i for i, a in enumerate(k.angles)
            if abs((math.degrees(a) % 360) - 180.0) < 1e-9]
    assert rear, "expected a wheel at 180 degrees"
    assert w[rear[0]] == pytest.approx(0.0, abs=1e-12)
    assert w[0] == pytest.approx(1.0 / R * math.sin(k.angles[0]), abs=1e-12)


def test_pure_yaw_spins_all_wheels_equally(k):
    """A pure yaw command must give all three wheels the same speed."""
    w = k.inverse_kinematics(0.0, 0.0, 1.0)
    assert w[0] == pytest.approx(w[1], abs=1e-12)
    assert w[1] == pytest.approx(w[2], abs=1e-12)
    assert w[0] == pytest.approx(-L / R, abs=1e-12)


def test_named_cases_match_kiwi_check(k):
    """Values quoted in the README and produced by tools/kiwi_check.py."""
    assert np.allclose(k.inverse_kinematics(1, 0, 0),
                       [28.8675, 0.0, -28.8675], atol=1e-4)
    assert np.allclose(k.inverse_kinematics(0, 1, 0),
                       [-16.6667, 33.3333, -16.6667], atol=1e-4)
    assert np.allclose(k.inverse_kinematics(0, 0, 1),
                       [-4.5167, -4.5167, -4.5167], atol=1e-4)


def test_zero_twist_is_zero_wheels(k):
    assert k.inverse_kinematics(0.0, 0.0, 0.0) == [0.0, 0.0, 0.0]


def test_linearity(k):
    """IK is linear -- relied on by scale_to_limit."""
    a = np.array(k.inverse_kinematics(0.3, -0.2, 0.5))
    b = np.array(k.inverse_kinematics(0.6, -0.4, 1.0))
    assert np.allclose(2 * a, b, atol=1e-12)


# ------------------------------------------------------------------ limiting
def test_scale_to_limit_respects_the_cap(k):
    vx, vy, wz, s = k.scale_to_limit(5.0, 3.0, 2.0, max_wheel_speed=10.0)
    assert s < 1.0
    assert max(abs(w) for w in k.inverse_kinematics(vx, vy, wz)) \
        == pytest.approx(10.0, abs=1e-9)


def test_scale_to_limit_preserves_direction(k):
    """Uniform scaling, not per-wheel clamping: the heading must not change."""
    vx, vy, wz, s = k.scale_to_limit(5.0, 3.0, 2.0, max_wheel_speed=10.0)
    assert math.atan2(vy, vx) == pytest.approx(math.atan2(3.0, 5.0), abs=1e-12)
    assert (vx, vy, wz) == pytest.approx((5.0 * s, 3.0 * s, 2.0 * s), abs=1e-12)


def test_scale_to_limit_is_a_noop_when_under(k):
    assert k.scale_to_limit(0.01, 0.0, 0.0, 100.0) == (0.01, 0.0, 0.0, 1.0)


def test_scale_to_limit_disabled(k):
    assert k.scale_to_limit(99.0, 0.0, 0.0, 0.0)[3] == 1.0


# --------------------------------------------------------------- robustness
def test_angles_are_a_parameter_not_a_constant():
    """The brief's 30/150/270 must be selectable without editing code."""
    alt = KiwiKinematics(wheel_angles_deg=(30.0, 150.0, 270.0))
    assert not np.allclose(alt.inverse_kinematics(1, 0, 0),
                           KiwiKinematics().inverse_kinematics(1, 0, 0))
    # still self-consistent, just a different robot
    assert np.allclose(alt.forward_kinematics(alt.inverse_kinematics(1, 0, 0)),
                       (1, 0, 0), atol=1e-12)


def test_rejects_bad_geometry():
    with pytest.raises(ValueError):
        KiwiKinematics(wheel_angles_deg=(0.0, 120.0))
    with pytest.raises(ValueError):
        KiwiKinematics(wheel_radius=0.0)
    with pytest.raises(ValueError):
        KiwiKinematics(base_radius=-1.0)
    with pytest.raises(ValueError):
        KiwiKinematics().forward_kinematics([1.0, 2.0])


def test_defaults_match_the_urdf():
    """Guards against base.xacro and this module drifting apart."""
    import xml.etree.ElementTree as ET
    urdf = os.path.join(os.path.dirname(__file__), "..", "generated", "robot.urdf")
    if not os.path.isfile(urdf):
        pytest.skip("generated/robot.urdf not generated; run tools/xacro_lite.py")
    root = ET.parse(urdf).getroot()
    J = {j.get("name"): j for j in root.findall("joint")}
    for i, want in enumerate(DEFAULT_WHEEL_ANGLES_DEG):
        o = J[f"wheel_{i}_joint"].find("origin")
        x, y, _ = (float(v) for v in o.get("xyz").split())
        assert math.degrees(math.atan2(y, x)) % 360 == pytest.approx(want, abs=1e-6)
        assert math.hypot(x, y) == pytest.approx(DEFAULT_BASE_RADIUS, abs=1e-9)

"""Check the ROS->UDP encoder against a transcription of esp32/controller.py.

The point of these tests is NOT that the arithmetic is self-consistent -- it is
that a ROS /cmd_vel produces byte-for-byte the same packet the pygame
controller would have sent for the equivalent keypress. The firmware is the
fixed point; controller.py is the known-good client; this is a second client
that must be indistinguishable from it.

Pure Python, no ROS graph, so it runs under plain pytest as well as under
`colcon test`.
"""
import math
import re

import pytest

from mmr_pkg.esp32_protocol import PacketEncoder


# --------------------------------------------------------------------------
# A faithful transcription of the packet-building half of esp32/controller.py.
# Kept literal, including the int() truncation and the 0..360 wrap, so it can
# be eyeballed against the original side by side.
# --------------------------------------------------------------------------
def controller_py_packet(keys, speed=180, rot_magnitude=80):
    x = y = 0
    if "w" in keys:
        y += 1
    if "s" in keys:
        y -= 1
    if "d" in keys:
        x += 1
    if "a" in keys:
        x -= 1

    if "q" in keys:
        rotation = -rot_magnitude
    elif "e" in keys:
        rotation = rot_magnitude
    else:
        rotation = 0

    if x == 0 and y == 0:
        return f"A,0,0,{rotation}"
    angle = math.degrees(math.atan2(y, x))
    if angle < 0:
        angle += 360
    return f"A,{int(angle)},{speed},{rotation}"


# Same keypresses expressed as a ROS twist. REP-103: +x forward, +y LEFT,
# +z yaw CCW. Magnitudes of 1.0 map to full scale under the encoder defaults.
KEYS_TO_TWIST = {
    ("w",):      (1.0, 0.0, 0.0),
    ("s",):      (-1.0, 0.0, 0.0),
    ("d",):      (0.0, -1.0, 0.0),      # right  = negative y
    ("a",):      (0.0, 1.0, 0.0),       # left   = positive y
    ("q",):      (0.0, 0.0, 1.0),       # turn left = CCW = positive yaw
    ("e",):      (0.0, 0.0, -1.0),
    (): (0.0, 0.0, 0.0),
}


@pytest.fixture
def enc():
    # Defaults are chosen to reproduce controller.py exactly: 1.0 m/s -> pwm
    # 180 (its cruise speed) and 1.0 rad/s -> pwm 80 (its Q/E value).
    return PacketEncoder()


@pytest.mark.parametrize("keys", list(KEYS_TO_TWIST))
def test_matches_controller_py(enc, keys):
    vx, vy, wz = KEYS_TO_TWIST[keys]
    assert enc.packet(vx, vy, wz).decode() == controller_py_packet(keys)


def test_diagonals_match_controller_py(enc):
    """The 45 degree cases, where the int() truncation could disagree."""
    for keys, (vx, vy) in {
        ("w", "d"): (1.0, -1.0),
        ("w", "a"): (1.0, 1.0),
        ("s", "d"): (-1.0, -1.0),
        ("s", "a"): (-1.0, 1.0),
    }.items():
        # Unit diagonal, so the magnitude is 1.0 and speed stays at cruise.
        n = math.hypot(vx, vy)
        got = enc.packet(vx / n, vy / n, 0.0).decode()
        assert got == controller_py_packet(keys), keys


def test_stationary_still_sends_rotation(enc):
    """Spinning on the spot is speed 0 with a non-zero rot, not silence."""
    assert enc.packet(0.0, 0.0, 1.0).decode() == "A,0,0,-80"


def test_zero_twist_is_a_full_stop(enc):
    assert enc.packet(0.0, 0.0, 0.0).decode() == "A,0,0,0"


# ------------------------------------------------------------------ protocol
@pytest.mark.parametrize("vx,vy,wz", [
    (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (-3.0, 2.5, -9.0),
    (1e6, -1e6, 1e6), (0.001, 0.0, 0.0),
])
def test_packet_is_always_parseable_by_the_firmware(enc, vx, vy, wz):
    """The firmware does sscanf("%c,%d,%d,%d") into a 64-byte buffer.

    Anything that does not match that shape gets "ERR" and is dropped, so the
    encoder must never emit a float, an exponent, or an over-long line -- not
    even for absurd input.
    """
    pkt = enc.packet(vx, vy, wz).decode()
    assert len(pkt) < 63, pkt
    m = re.fullmatch(r"A,(\d+),(\d+),(-?\d+)", pkt)
    assert m, pkt
    angle, speed, rot = (int(g) for g in m.groups())
    assert 0 <= angle <= 359
    assert 0 <= speed <= 255           # firmware constrains, but do not rely
    assert -255 <= rot <= 255


def test_saturation_clamps_rather_than_wraps(enc):
    """A silly-fast command must pin at full scale, not overflow into reverse."""
    _, speed, rot = enc.encode(1000.0, 0.0, 1000.0)
    assert speed == 255
    assert rot == -255


# -------------------------------------------------------------------- signs
def test_strafe_sign_mirrors_left_and_right():
    a = PacketEncoder(strafe_sign=1)
    b = PacketEncoder(strafe_sign=-1)
    left = (0.0, 1.0, 0.0)
    assert a.encode(*left)[0] == 180
    assert b.encode(*left)[0] == 0     # mirrored, i.e. drives right instead


def test_yaw_sign_flips_rotation_only():
    a = PacketEncoder(yaw_sign=-1)
    b = PacketEncoder(yaw_sign=1)
    assert a.encode(0.0, 0.0, 1.0)[2] == -80
    assert b.encode(0.0, 0.0, 1.0)[2] == 80
    # ...and leaves translation untouched
    assert a.encode(1.0, 0.0, 0.0)[:2] == b.encode(1.0, 0.0, 0.0)[:2]


# ---------------------------------------------------------------- clipping
def test_prevent_clipping_preserves_the_translation_rotation_ratio():
    """The firmware clamps each wheel AFTER subtracting rot.

    Worst case a wheel sees speed + |rot|; above 255 it clips and the robot
    curves away from the commanded heading. prevent_clipping scales both terms
    by one factor so the ratio, and therefore the path, is unchanged.
    """
    off = PacketEncoder(prevent_clipping=False)
    on = PacketEncoder(prevent_clipping=True)

    # Full translation plus full rotation: 180 + 80 = 260, over the limit.
    _, s_off, r_off = off.encode(1.0, 0.0, 1.0)
    assert s_off + abs(r_off) > 255           # the problem exists

    _, s_on, r_on = on.encode(1.0, 0.0, 1.0)
    assert s_on + abs(r_on) <= 255            # and is fixed

    # Ratio preserved to within integer rounding.
    assert abs(s_on / abs(r_on) - s_off / abs(r_off)) < 0.05


def test_prevent_clipping_is_a_no_op_when_within_range():
    on = PacketEncoder(prevent_clipping=True)
    off = PacketEncoder(prevent_clipping=False)
    slow = (0.2, 0.0, 0.2)
    assert on.encode(*slow) == off.encode(*slow)


# ------------------------------------------------------------ normalisation
def test_max_speed_is_the_denominator_of_the_pwm_map():
    """Doubling max_linear_speed halves the PWM for the same cmd_vel."""
    a = PacketEncoder(max_linear_speed=1.0)
    b = PacketEncoder(max_linear_speed=2.0)
    assert a.encode(1.0, 0.0, 0.0)[1] == 180
    assert b.encode(1.0, 0.0, 0.0)[1] == 90


def test_rejects_a_zero_denominator():
    with pytest.raises(ValueError):
        PacketEncoder(max_linear_speed=0.0)
    with pytest.raises(ValueError):
        PacketEncoder(max_angular_speed=0.0)

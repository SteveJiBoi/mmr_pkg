"""Tests for the keyboard teleop key map (S16: key -> Twist mapping).

No ROS, no terminal, no clock: TeleopState takes time as an argument, so a held
key, a released key and a clock that steps backwards are all just numbers here.
"""
import pytest

from mmr_pkg.teleop_keys import (MIN_SPEED, MOVE_BINDINGS, QUIT_KEYS,
                                 SPEED_BINDINGS, STOP_KEYS, TeleopState)

LIN = 0.5
ANG = 1.0


@pytest.fixture
def kb():
    return TeleopState(linear_speed=LIN, angular_speed=ANG, key_timeout=0.6,
                       max_linear_speed=1.0, max_angular_speed=2.0)


# ============================================================ the six drive keys
class TestKeyToTwist:
    """REP-103 body axes: +x forward, +y LEFT, +z counter-clockwise."""

    @pytest.mark.parametrize("key,expected", [
        ("w", (+LIN, 0.0, 0.0)),        # forward
        ("x", (-LIN, 0.0, 0.0)),        # backward -- X, deliberately not S
        ("a", (0.0, +LIN, 0.0)),        # strafe left  = +y
        ("d", (0.0, -LIN, 0.0)),        # strafe right = -y
        ("q", (0.0, 0.0, +ANG)),        # turn left    = CCW = +z
        ("e", (0.0, 0.0, -ANG)),        # turn right   = CW  = -z
    ])
    def test_each_key_maps_to_its_axis(self, kb, key, expected):
        kb.handle(key, now=0.0)
        assert kb.twist(0.0) == pytest.approx(expected)

    def test_s_is_not_bound(self):
        """The brief asks for X as reverse; S must do nothing at all."""
        assert "s" not in MOVE_BINDINGS

    def test_uppercase_works_the_same(self, kb):
        """Caps lock or a held shift must not silently stop the robot."""
        kb.handle("W", now=0.0)
        assert kb.twist(0.0) == pytest.approx((+LIN, 0.0, 0.0))

    def test_an_unknown_key_is_ignored_not_treated_as_stop(self, kb):
        kb.handle("w", now=0.0)
        kb.handle("j", now=0.0)
        assert kb.twist(0.0) == pytest.approx((+LIN, 0.0, 0.0))

    def test_nothing_pressed_is_a_zero_twist(self, kb):
        assert kb.twist(0.0) == (0.0, 0.0, 0.0)

    def test_two_keys_combine_into_a_diagonal(self, kb):
        kb.handle("w", now=0.0)
        kb.handle("a", now=0.0)
        assert kb.twist(0.0) == pytest.approx((+LIN, +LIN, 0.0))

    def test_translation_and_rotation_combine(self, kb):
        kb.handle("w", now=0.0)
        kb.handle("e", now=0.0)
        assert kb.twist(0.0) == pytest.approx((+LIN, 0.0, -ANG))

    def test_opposing_keys_cancel(self, kb):
        kb.handle("w", now=0.0)
        kb.handle("x", now=0.0)
        assert kb.twist(0.0) == pytest.approx((0.0, 0.0, 0.0))


# ================================================================ key release
class TestRelease:
    """A terminal has no key-up event; release is inferred from auto-repeat."""

    def test_a_key_expires_after_key_timeout(self, kb):
        kb.handle("w", now=0.0)
        assert kb.twist(0.5) == pytest.approx((+LIN, 0.0, 0.0))
        assert kb.twist(0.7) == (0.0, 0.0, 0.0)

    def test_auto_repeat_keeps_the_key_alive(self, kb):
        for t in (0.0, 0.5, 1.0, 1.5):
            kb.handle("w", now=t)
            assert kb.twist(t) == pytest.approx((+LIN, 0.0, 0.0))
        assert kb.twist(2.2) == (0.0, 0.0, 0.0)

    def test_keys_expire_independently(self, kb):
        kb.handle("w", now=0.0)
        kb.handle("a", now=0.5)
        assert kb.twist(0.5) == pytest.approx((+LIN, +LIN, 0.0))
        assert kb.twist(0.9) == pytest.approx((0.0, +LIN, 0.0))   # w has gone

    def test_a_backwards_clock_step_releases_everything(self, kb):
        """Same rule as the bridge: untrustworthy time reads as stop."""
        kb.handle("w", now=1000.0)
        assert kb.twist(900.0) == (0.0, 0.0, 0.0)


# ======================================================================= stop
class TestStop:

    def test_space_stops_immediately(self, kb):
        kb.handle("w", now=0.0)
        kb.handle(" ", now=0.0)
        assert kb.twist(0.0) == (0.0, 0.0, 0.0)

    def test_space_is_not_undone_by_a_key_still_repeating(self, kb):
        """Clearing the held set, not just the output.

        If SPACE only zeroed the output, the next auto-repeat of a key that is
        still physically down would start the robot again a few ms later.
        """
        kb.handle("w", now=0.0)
        kb.handle(" ", now=0.1)
        assert kb.twist(0.2) == (0.0, 0.0, 0.0)
        assert kb.active_keys(0.2) == ()

    def test_space_reports_itself_to_the_operator(self, kb):
        assert kb.handle(" ", now=0.0) == "STOP"

    def test_stop_does_not_change_the_speed_setting(self, kb):
        kb.handle("=", now=0.0)
        fast = kb.linear_speed
        kb.handle(" ", now=0.0)
        assert kb.linear_speed == fast

    def test_explicit_stop_zeroes_everything(self, kb):
        kb.handle("w", now=0.0)
        assert kb.stop() == (0.0, 0.0, 0.0)
        assert kb.twist(0.0) == (0.0, 0.0, 0.0)


# ============================================================== speed control
class TestSpeedControl:

    def test_plus_and_minus_change_the_speed(self, kb):
        kb.handle("=", now=0.0)
        assert kb.linear_speed == pytest.approx(LIN * 1.1)
        kb.handle("-", now=0.0)
        assert kb.linear_speed == pytest.approx(LIN * 1.1 * 0.9)

    def test_brackets_change_only_the_turn_speed(self, kb):
        kb.handle("]", now=0.0)
        assert kb.linear_speed == pytest.approx(LIN)
        assert kb.angular_speed == pytest.approx(ANG * 1.1)

    def test_the_new_speed_applies_to_the_next_twist(self, kb):
        kb.handle("w", now=0.0)
        kb.handle("=", now=0.0)
        assert kb.twist(0.0)[0] == pytest.approx(LIN * 1.1)

    def test_speed_cannot_exceed_the_ceiling(self, kb):
        for _ in range(50):
            kb.handle("=", now=0.0)
        assert kb.linear_speed == pytest.approx(1.0)
        assert kb.angular_speed == pytest.approx(2.0)

    def test_speed_cannot_be_wound_down_to_a_standstill(self, kb):
        """Below MIN_SPEED the robot will not overcome its own stiction, and a
        teleop that commands an unmovable speed just looks broken."""
        for _ in range(200):
            kb.handle("-", now=0.0)
        assert kb.linear_speed == pytest.approx(MIN_SPEED)

    def test_a_speed_change_is_shown_to_the_operator(self, kb):
        note = kb.handle("=", now=0.0)
        assert note is not None and "speed" in note

    def test_a_starting_speed_above_the_ceiling_is_clamped(self):
        kb = TeleopState(linear_speed=99.0, angular_speed=99.0,
                         max_linear_speed=1.0, max_angular_speed=2.0)
        assert kb.linear_speed == pytest.approx(1.0)
        assert kb.angular_speed == pytest.approx(2.0)


# ====================================================================== quit
class TestQuit:

    @pytest.mark.parametrize("key", sorted(QUIT_KEYS))
    def test_quit_keys_set_the_flag_and_stop(self, kb, key):
        kb.handle("w", now=0.0)
        kb.handle(key, now=0.0)
        assert kb.quit
        assert kb.twist(0.0) == (0.0, 0.0, 0.0)

    def test_ctrl_c_is_a_quit_key(self):
        """cbreak leaves ISIG on so Ctrl-C normally raises, but a terminal with
        ISIG cleared would deliver it as a character; handle both."""
        assert "\x03" in QUIT_KEYS


# ================================================================== bindings
class TestBindingTable:
    """The map is meant to be edited in one place; keep it coherent."""

    def test_no_key_has_two_jobs(self):
        groups = [set(MOVE_BINDINGS), set(SPEED_BINDINGS), set(STOP_KEYS),
                  set(QUIT_KEYS)]
        for i, a in enumerate(groups):
            for b in groups[i + 1:]:
                assert not (a & b), f"key bound twice: {a & b}"

    def test_every_direction_is_a_unit_vector_on_one_axis(self):
        """Bindings are DIRECTIONS scaled at use time, never speeds in m/s."""
        for key, vec in MOVE_BINDINGS.items():
            nonzero = [v for v in vec if v != 0.0]
            assert len(nonzero) == 1, key
            assert abs(nonzero[0]) == 1.0, key

    def test_every_axis_has_both_directions_bound(self):
        assert set(MOVE_BINDINGS.values()) == {
            (+1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
            (0.0, +1.0, 0.0), (0.0, -1.0, 0.0),
            (0.0, 0.0, +1.0), (0.0, 0.0, -1.0)}

    def test_bindings_are_lowercase_so_handle_can_normalise(self):
        for key in MOVE_BINDINGS:
            assert key == key.lower()

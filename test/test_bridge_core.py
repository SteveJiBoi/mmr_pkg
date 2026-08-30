"""Tests for the bridge's safety and link policy.

These are the tests that could not exist while this logic lived inside the ROS
node: nothing could import it without rclpy, so nothing ran it, and an
AttributeError on the last line of __init__ shipped under a green suite. Every
test here exercises the code the robot actually runs.

Covers the S16 list: speed limiting, angular limiting, command timeout, zero
command generation, and invalid ESP32 IP handling.

Pure Python, no ROS graph, no network.
"""
import math

import pytest

from mmr_pkg.bridge_core import (CommandWatchdog, LinkMonitor, VelocityShaper,
                                 resolve, resolve_endpoint)


# ============================================================ command timeout
class TestCommandWatchdog:

    def test_a_fresh_command_is_passed_through(self):
        w = CommandWatchdog(timeout=0.5)
        w.update(1.0, 2.0, 3.0, now=100.0)
        assert w.command(100.2) == (1.0, 2.0, 3.0)
        assert not w.is_stale(100.2)

    def test_an_expired_command_becomes_zero(self):
        w = CommandWatchdog(timeout=0.5)
        w.update(1.0, 2.0, 3.0, now=100.0)
        assert w.is_stale(100.6)
        assert w.command(100.6) == (0.0, 0.0, 0.0)

    def test_the_boundary_is_not_stale_but_just_past_it_is(self):
        w = CommandWatchdog(timeout=0.5)
        w.update(1.0, 0.0, 0.0, now=0.0)
        assert not w.is_stale(0.5)
        assert w.is_stale(0.5 + 1e-9)

    def test_before_any_command_the_robot_does_not_move(self):
        """Start-up must be a full stop, not an undefined command."""
        w = CommandWatchdog(timeout=0.5)
        assert w.is_stale(0.0)
        assert w.command(0.0) == (0.0, 0.0, 0.0)
        assert w.age(0.0) is None

    def test_a_backwards_clock_step_reads_as_stale(self):
        """NTP stepping the Pi's clock back must not revive an old command.

        Negative elapsed time means the clock is untrustworthy; the safe
        reading of "I don't know how old this is" is "too old".
        """
        w = CommandWatchdog(timeout=0.5)
        w.update(1.0, 0.0, 0.0, now=1000.0)
        assert w.is_stale(900.0)
        assert w.command(900.0) == (0.0, 0.0, 0.0)

    def test_rejects_a_nonsense_timeout(self):
        with pytest.raises(ValueError):
            CommandWatchdog(timeout=0.0)
        with pytest.raises(ValueError):
            CommandWatchdog(timeout=-1.0)


# ============================================================ speed limiting
class TestVelocityLimiting:

    def test_linear_speed_is_capped(self):
        s = VelocityShaper(max_linear_speed=1.0)
        vx, vy, _ = s.limit(5.0, 0.0, 0.0)
        assert vx == pytest.approx(1.0)
        assert vy == pytest.approx(0.0)

    def test_angular_speed_is_capped_both_ways(self):
        s = VelocityShaper(max_angular_speed=2.0)
        assert s.limit(0.0, 0.0, 9.0)[2] == pytest.approx(2.0)
        assert s.limit(0.0, 0.0, -9.0)[2] == pytest.approx(-2.0)

    def test_limiting_preserves_the_direction_of_travel(self):
        """The whole reason the clamp is on magnitude and not per axis.

        Per-axis clamping of (2, 1) to a ceiling of 1 gives (1, 1) -- a 45
        degree error in a robot that was asked to go mostly forward.
        """
        s = VelocityShaper(max_linear_speed=1.0)
        vx, vy, _ = s.limit(2.0, 1.0, 0.0)
        assert math.hypot(vx, vy) == pytest.approx(1.0)
        assert math.atan2(vy, vx) == pytest.approx(math.atan2(1.0, 2.0))

    def test_a_twist_inside_the_limits_is_untouched(self):
        s = VelocityShaper(max_linear_speed=1.0, max_angular_speed=1.0)
        assert s.limit(0.3, -0.2, 0.5) == pytest.approx((0.3, -0.2, 0.5))

    def test_linear_and_angular_limits_are_independent(self):
        """Saturating the spin must not slow the translation, or vice versa."""
        s = VelocityShaper(max_linear_speed=1.0, max_angular_speed=1.0)
        vx, vy, wz = s.limit(0.5, 0.0, 99.0)
        assert (vx, vy) == pytest.approx((0.5, 0.0))
        assert wz == pytest.approx(1.0)

    def test_rejects_a_zero_ceiling(self):
        with pytest.raises(ValueError):
            VelocityShaper(max_linear_speed=0.0)
        with pytest.raises(ValueError):
            VelocityShaper(max_angular_speed=0.0)

    def test_rejects_a_negative_acceleration_limit(self):
        with pytest.raises(ValueError):
            VelocityShaper(max_linear_accel=-1.0)


# ================================================================= smoothing
class TestVelocitySmoothing:

    def test_smoothing_is_off_by_default(self):
        """There is no measured acceleration for this robot, so no default."""
        s = VelocityShaper()
        assert s.step(1.0, 0.0, 0.0, dt=0.05) == pytest.approx((1.0, 0.0, 0.0))

    def test_acceleration_is_rate_limited_when_configured(self):
        s = VelocityShaper(max_linear_accel=1.0)          # 1 m/s^2
        assert s.step(1.0, 0.0, 0.0, dt=0.1)[0] == pytest.approx(0.1)
        assert s.step(1.0, 0.0, 0.0, dt=0.1)[0] == pytest.approx(0.2)

    def test_the_ramp_reaches_the_target_and_stops_there(self):
        s = VelocityShaper(max_linear_accel=10.0)
        for _ in range(20):
            s.step(0.5, 0.0, 0.0, dt=0.1)
        assert s.current[0] == pytest.approx(0.5)

    def test_braking_is_unlimited_by_default(self):
        """A slow ramp down is a robot that will not stop when told."""
        s = VelocityShaper(max_linear_accel=0.5)
        for _ in range(10):
            s.step(1.0, 0.0, 0.0, dt=0.1)
        assert s.current[0] > 0.0
        assert s.step(0.0, 0.0, 0.0, dt=0.1)[0] == pytest.approx(0.0)

    def test_deceleration_is_rate_limited_when_asked_for(self):
        s = VelocityShaper(max_linear_accel=10.0, max_linear_decel=1.0)
        s.step(1.0, 0.0, 0.0, dt=1.0)
        assert s.current[0] == pytest.approx(1.0)
        assert s.step(0.0, 0.0, 0.0, dt=0.1)[0] == pytest.approx(0.9)

    def test_angular_smoothing_is_separate_from_linear(self):
        s = VelocityShaper(max_angular_accel=1.0)
        vx, _, wz = s.step(1.0, 0.0, 1.0, dt=0.1)
        assert vx == pytest.approx(1.0)          # linear unlimited
        assert wz == pytest.approx(0.1)          # angular ramped

    def test_the_ramp_holds_the_direction_of_travel(self):
        """Ramping each axis separately would bend the path during the ramp."""
        s = VelocityShaper(max_linear_accel=1.0)
        vx, vy, _ = s.step(0.6, 0.8, 0.0, dt=0.1)      # unit vector target
        assert math.hypot(vx, vy) == pytest.approx(0.1)
        assert math.atan2(vy, vx) == pytest.approx(math.atan2(0.8, 0.6))

    def test_stop_ignores_every_acceleration_limit(self):
        s = VelocityShaper(max_linear_accel=0.1, max_linear_decel=0.1)
        for _ in range(10):
            s.step(1.0, 0.0, 0.0, dt=0.1)
        assert s.current[0] > 0.0
        assert s.stop() == (0.0, 0.0, 0.0)
        assert s.current == (0.0, 0.0, 0.0)

    def test_a_nonsense_dt_jumps_to_the_target_rather_than_guessing(self):
        s = VelocityShaper(max_linear_accel=1.0)
        assert s.step(1.0, 0.0, 0.0, dt=0.0)[0] == pytest.approx(1.0)


# ======================================================== the safety coupling
class TestResolve:

    def test_a_fresh_command_drives(self):
        w, s = CommandWatchdog(0.5), VelocityShaper()
        w.update(0.4, 0.0, 0.0, now=0.0)
        assert resolve(w, s, now=0.1, dt=0.05) == pytest.approx((0.4, 0.0, 0.0))

    def test_a_stale_command_stops_HARD_and_does_not_ramp_down(self):
        """The bug this function exists to make unwritable.

        watchdog.command() returns zero when stale, so feeding it to step()
        looks like a stop -- but it decelerates to zero at the configured rate.
        Losing the operator is exactly when smoothing must not apply.
        """
        w = CommandWatchdog(timeout=0.5)
        s = VelocityShaper(max_linear_accel=10.0, max_linear_decel=0.01)
        w.update(1.0, 0.0, 0.0, now=0.0)
        resolve(w, s, now=0.0, dt=1.0)
        assert s.current[0] == pytest.approx(1.0)

        # The wrong way would still be moving; resolve() is already stopped.
        assert s.step(*w.command(99.0), dt=0.1)[0] > 0.9      # the trap
        s.stop()
        resolve(w, s, now=0.0, dt=1.0)                        # back up to speed
        assert resolve(w, s, now=99.0, dt=0.1) == (0.0, 0.0, 0.0)

    def test_nothing_moves_before_the_first_command(self):
        w, s = CommandWatchdog(0.5), VelocityShaper()
        assert resolve(w, s, now=0.0, dt=0.05) == (0.0, 0.0, 0.0)


# ============================================================== link monitor
class TestLinkMonitor:

    def test_a_link_that_never_answers_is_eventually_reported(self):
        """The defect in the previous revision: a wrong IP was silent forever.

        It only ever checked for a link that had already been up, so "never
        worked" produced no message at all.
        """
        m = LinkMonitor(reply_timeout=1.0)
        m.on_send(0.0)
        assert m.state(0.5) == LinkMonitor.NEVER_SEEN
        assert m.poll(0.5) is None
        assert m.state(1.5) == LinkMonitor.DOWN
        assert m.poll(1.5) == LinkMonitor.DOWN
        assert m.received == 0          # how the node words it differently

    def test_a_reply_brings_the_link_up(self):
        m = LinkMonitor(reply_timeout=1.0)
        m.on_send(0.0)
        m.on_reply(0.1)
        assert m.state(0.2) == LinkMonitor.UP
        assert m.poll(0.2) == LinkMonitor.UP

    def test_silence_after_working_is_a_loss_not_a_never(self):
        m = LinkMonitor(reply_timeout=1.0)
        m.on_send(0.0)
        m.on_reply(0.1)
        m.poll(0.2)
        assert m.poll(2.0) == LinkMonitor.DOWN
        assert m.received == 1          # distinguishes it from a wrong address

    def test_poll_reports_only_transitions(self):
        """S2: do not spam the terminal."""
        m = LinkMonitor(reply_timeout=1.0)
        m.on_send(0.0)
        m.on_reply(0.1)
        assert m.poll(0.2) == LinkMonitor.UP
        assert m.poll(0.3) is None
        assert m.poll(0.4) is None

    def test_recovery_is_reported_after_a_loss(self):
        m = LinkMonitor(reply_timeout=1.0)
        m.on_send(0.0)
        m.on_reply(0.1)
        m.poll(0.2)
        assert m.poll(2.0) == LinkMonitor.DOWN
        m.on_reply(2.1)
        assert m.poll(2.2) == LinkMonitor.UP

    def test_rtt_pairs_a_reply_with_the_oldest_unmatched_send(self):
        m = LinkMonitor()
        m.on_send(0.0)
        m.on_send(0.05)
        m.on_reply(0.14)
        assert m.rtt == pytest.approx(0.14)
        m.on_reply(0.19)
        assert m.rtt == pytest.approx(0.14)

    def test_statistics_count_sends_replies_and_errors(self):
        m = LinkMonitor()
        for t in range(10):
            m.on_send(float(t))
        for t in range(8):
            m.on_reply(float(t))
        m.on_send_error()
        assert (m.sent, m.received, m.send_errors) == (10, 8, 1)
        assert m.reply_ratio == pytest.approx(0.8)

    def test_reply_ratio_is_zero_rather_than_a_division_error(self):
        assert LinkMonitor().reply_ratio == 0.0

    def test_the_unmatched_queue_cannot_grow_without_bound(self):
        """A dead far end must not leak memory for as long as the node runs."""
        m = LinkMonitor(window=8)
        for t in range(1000):
            m.on_send(float(t))
        assert len(m._unmatched) == 8

    def test_last_reply_age_is_exact_and_needs_no_pairing(self):
        m = LinkMonitor()
        assert m.last_reply_age(1.0) is None
        m.on_reply(1.0)
        assert m.last_reply_age(1.4) == pytest.approx(0.4)


# ====================================================== invalid ESP32 address
class TestResolveEndpoint:

    def test_a_numeric_address_passes_straight_through(self):
        assert resolve_endpoint("10.229.5.249", 1234) == ("10.229.5.249", 1234)

    def test_an_empty_address_is_refused_with_instructions(self):
        with pytest.raises(ValueError, match="esp32_ip is not set"):
            resolve_endpoint("", 1234)

    @pytest.mark.parametrize("bad", ["10.229.5", "999.1.1.1", "not an ip",
                                     "10.229.5.249.7"])
    def test_a_malformed_address_is_refused_at_startup(self, bad):
        """UDP would accept the mistake silently and the robot would sit still.

        There is no connection to fail, so a packet to a plausible-but-wrong
        address just leaves and is never heard of again.
        """
        with pytest.raises(ValueError):
            resolve_endpoint(bad, 1234)

    @pytest.mark.parametrize("port", [0, -1, 65536, 99999])
    def test_an_out_of_range_port_is_refused(self, port):
        with pytest.raises(ValueError, match="esp32_port"):
            resolve_endpoint("10.229.5.249", port)

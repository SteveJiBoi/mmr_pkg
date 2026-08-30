"""Safety and link policy for the ESP32 bridge, with no ROS dependency.

Third file in the same pattern as kiwi_kinematics.py and esp32_protocol.py: the
decisions live here, the ROS plumbing lives in esp32_bridge.py. That split is
not tidiness. A previous revision kept this logic inside the node, and because
the node cannot be imported without rclpy, nothing about it was ever executed by
a test -- an `AttributeError` on the last line of __init__ shipped with a green
suite and the node could not start at all. Everything in this file runs under
plain pytest on a laptop.

Nothing here knows what a Node, a Twist or a socket is. Time arrives as float
seconds from the caller's clock; the caller decides which clock.


THE PIPELINE
------------
    /cmd_vel -> CommandWatchdog -> VelocityShaper -> PacketEncoder -> UDP
                (is it fresh?)     (SI limits and    (SI -> PWM,
                                    accel limits)     esp32_protocol.py)

Each stage has exactly one job and one owner for each number. max_linear_speed
is declared once and is used by BOTH the shaper (as the ceiling) and the encoder
(as the denominator of the PWM map), so the two cannot drift apart.


CLOCKS AND THE BACKWARDS-JUMP RULE
----------------------------------
A Pi that has just come up will step its clock when NTP first syncs. If that
step is backwards, a naive `now - last_command > timeout` goes negative, reads
as "fresh", and the robot keeps executing a command that may be arbitrarily
old. So elapsed time that comes back negative is treated as STALE, not fresh:
if the clock is not trustworthy then neither is the command, and the safe
interpretation of "I don't know" is "stop".
"""
from __future__ import annotations

import ipaddress
import math
import socket
from collections import deque
from typing import Deque, Optional, Tuple

Twist3 = Tuple[float, float, float]

ZERO: Twist3 = (0.0, 0.0, 0.0)


def _elapsed(now: float, then: Optional[float]) -> Optional[float]:
    """Seconds from `then` to `now`, or None if that cannot be trusted.

    None means "no reading" and every caller must treat it as the unsafe case.
    See the backwards-jump rule in the module docstring.
    """
    if then is None:
        return None
    dt = now - then
    return None if dt < 0.0 else dt


def resolve_endpoint(ip: str, port: int) -> Tuple[str, int]:
    """Validate the ESP32 address and resolve it to numeric form ONCE.

    Two separate reasons this is not left to sendto():

    LATENCY. sendto() given a HOSTNAME resolves it on every single call. That is
    a blocking network round trip inside the control loop, at the control rate,
    on a Pi -- exactly what S15 rules out, and it would sit in the same 20 Hz
    timer that has to beat a 400 ms firmware deadman. Resolving once here means
    the hot path only ever sees four numbers.

    DIAGNOSIS. UDP is connectionless, so a packet sent to a plausible but wrong
    address is not an error: it leaves and is never heard of again, and the
    robot just sits there looking broken. Catching a malformed address at
    start-up converts an unexplained dead robot into a message at the moment of
    the mistake.

    Raises ValueError with something actionable. Returns (numeric_ip, port).
    """
    if not ip:
        raise ValueError(
            "esp32_ip is not set. Start with e.g.\n"
            "  ros2 run mmr_pkg esp32_bridge --ros-args "
            "-p esp32_ip:=10.229.5.249\n"
            "The ESP32 prints its address on its serial console at boot "
            '("Ready. IP: ...").')
    if not 0 < int(port) < 65536:
        raise ValueError(f"esp32_port must be in 1..65535, got {port!r}")

    # Anything made only of digits and dots is judged as an address and is
    # NEVER sent to a resolver. Two reasons. It makes a typo like "10.229.5"
    # fail the same way on every machine, instead of depending on whether the
    # local resolver happens to be permissive or the network has a wildcard
    # record. And it means a mistyped address fails instantly rather than after
    # a DNS timeout, which on the measured network is seconds of a robot
    # apparently hanging at start-up.
    if ip and all(c.isdigit() or c == "." for c in ip):
        try:
            return str(ipaddress.IPv4Address(ip)), int(port)
        except ValueError as exc:
            raise ValueError(
                f"esp32_ip {ip!r} looks like an IPv4 address but is not a "
                f"valid one ({exc}). The ESP32 prints its address on its "
                f'serial console at boot ("Ready. IP: ...").') from exc

    try:
        # AF_INET only: the firmware's WiFiUDP is IPv4, so an IPv6 result would
        # be undeliverable rather than merely unusual.
        info = socket.getaddrinfo(ip, int(port), socket.AF_INET,
                                  socket.SOCK_DGRAM)
    except socket.gaierror as exc:
        raise ValueError(
            f"esp32_ip {ip!r} is neither a valid IPv4 address nor a "
            f"resolvable name ({exc}). The ESP32 firmware prints its address "
            f"on the serial console at boot.") from exc

    return info[0][4][0], int(port)


class CommandWatchdog:
    """Holds the latest commanded twist and decides whether it is still fresh.

    The firmware has its own 400 ms deadman and that is the mechanism that
    actually matters, because it survives this process being killed, the Pi
    sleeping and WiFi dropping. This watchdog is the earlier, softer one: it
    turns "the publisher went quiet" into an explicit zero command roughly a
    control cycle later, rather than letting the robot coast for the remainder
    of the firmware's window.
    """

    def __init__(self, timeout: float = 0.5) -> None:
        if timeout <= 0.0:
            raise ValueError("cmd_timeout must be > 0 seconds")
        self.timeout = float(timeout)
        self._cmd: Twist3 = ZERO
        self._stamp: Optional[float] = None

    def update(self, vx: float, vy: float, wz: float, now: float) -> None:
        self._cmd = (float(vx), float(vy), float(wz))
        self._stamp = now

    def is_stale(self, now: float) -> bool:
        age = _elapsed(now, self._stamp)
        # Never commanded, or an untrustworthy clock: both are stale. Refusing
        # to move until told to is the only safe start-up state.
        return age is None or age > self.timeout

    def command(self, now: float) -> Twist3:
        """The twist to act on: the latest one, or zero once it has expired."""
        return ZERO if self.is_stale(now) else self._cmd

    def age(self, now: float) -> Optional[float]:
        return _elapsed(now, self._stamp)


class VelocityShaper:
    """Clamps a twist to configured limits (S4), then optionally slew-limits it (S5).

    LIMITING. The linear clamp is on the MAGNITUDE of (vx, vy), not on each axis
    separately. Clamping axes independently would shorten one component more
    than the other and quietly rotate the direction of travel -- ask for
    (2.0, 1.0) with a ceiling of 1.0 and per-axis clamping gives (1.0, 1.0),
    a 45 degree error. Scaling both by one factor keeps the heading and only
    reduces the speed. Same reasoning as kiwi_kinematics.scale_to_limit and as
    prevent_clipping in esp32_protocol.py; three stages, one principle.

    SMOOTHING is OFF by default, and the defaults are 0.0 meaning "no limit"
    rather than some plausible-looking number. There is no measured
    acceleration figure for this robot -- there are no encoders -- so any
    default here would be invented. Set them from observation if you want them.

    Acceleration and deceleration are separate because they are not equally
    risky. A slow ramp UP is comfortable; a slow ramp DOWN is a robot that will
    not stop when you tell it to. Leaving max_*_decel at 0.0 means braking is
    never rate limited, which is the safe default.

    None of this applies to an emergency stop: call stop() and the state snaps
    to zero with no ramp at all.
    """

    def __init__(self,
                 max_linear_speed: float = 1.0,
                 max_angular_speed: float = 1.0,
                 max_linear_accel: float = 0.0,
                 max_angular_accel: float = 0.0,
                 max_linear_decel: float = 0.0,
                 max_angular_decel: float = 0.0) -> None:
        if max_linear_speed <= 0.0 or max_angular_speed <= 0.0:
            raise ValueError("max_linear_speed and max_angular_speed must be "
                             "> 0; they are both a ceiling and, in "
                             "PacketEncoder, the denominator of the PWM map")
        if min(max_linear_accel, max_angular_accel,
               max_linear_decel, max_angular_decel) < 0.0:
            raise ValueError("acceleration limits cannot be negative; "
                             "use 0.0 to mean unlimited")

        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)
        self.max_linear_accel = float(max_linear_accel)
        self.max_angular_accel = float(max_angular_accel)
        self.max_linear_decel = float(max_linear_decel)
        self.max_angular_decel = float(max_angular_decel)

        self._current: Twist3 = ZERO

    # ------------------------------------------------------------------ pure
    def limit(self, vx: float, vy: float, wz: float) -> Twist3:
        """Clamp to the configured ceilings. No state, no time."""
        speed = math.hypot(vx, vy)
        if speed > self.max_linear_speed:
            scale = self.max_linear_speed / speed
            vx *= scale
            vy *= scale
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))
        return vx, vy, wz

    # ----------------------------------------------------------------- state
    @property
    def current(self) -> Twist3:
        return self._current

    def stop(self) -> Twist3:
        """Emergency stop: snap to zero, ignoring every acceleration limit."""
        self._current = ZERO
        return self._current

    def step(self, vx: float, vy: float, wz: float, dt: float) -> Twist3:
        """Advance one control cycle towards the (limited) target twist."""
        target = self.limit(vx, vy, wz)

        # A nonsensical dt -- first cycle, a clock step, a stalled timer --
        # cannot be used to compute a rate. Jumping straight to the target is
        # the same behaviour as smoothing being disabled, which is the
        # documented default, so it degrades to something already expected.
        if dt <= 0.0 or not self._limits_active():
            self._current = target
            return self._current

        cx, cy, cw = self._current
        tx, ty, tw = target

        # Ramp the linear VECTOR, not each axis: an axis-wise ramp would bend
        # the path during the ramp for the same reason axis-wise clamping does.
        self._current = (
            *self._ramp_vector((cx, cy), (tx, ty), dt),
            self._ramp_scalar(cw, tw, dt,
                              self.max_angular_accel, self.max_angular_decel),
        )
        return self._current

    # -------------------------------------------------------------- internals
    def _limits_active(self) -> bool:
        return any((self.max_linear_accel, self.max_angular_accel,
                    self.max_linear_decel, self.max_angular_decel))

    def _ramp_vector(self, cur: Tuple[float, float],
                     tgt: Tuple[float, float], dt: float) -> Tuple[float, float]:
        dx, dy = tgt[0] - cur[0], tgt[1] - cur[1]
        need = math.hypot(dx, dy)
        if need < 1e-12:
            return tgt

        # Speeding up or slowing down, judged by distance from standstill, so
        # the decel limit governs any NET reduction in speed. A direction change
        # that holds speed -- including a full reversal, where |target| equals
        # |current| -- counts as acceleration. That is deliberate: what is being
        # limited is a rate of change of velocity, and how hard the robot may
        # push is the accel limit regardless of which way it is pushing.
        speeding_up = math.hypot(*tgt) >= math.hypot(*cur)
        rate = self.max_linear_accel if speeding_up else self.max_linear_decel
        if rate <= 0.0:                      # 0.0 means unlimited
            return tgt

        allowed = rate * dt
        if need <= allowed:
            return tgt
        f = allowed / need
        return cur[0] + dx * f, cur[1] + dy * f

    @staticmethod
    def _ramp_scalar(cur: float, tgt: float, dt: float,
                     accel: float, decel: float) -> float:
        rate = accel if abs(tgt) >= abs(cur) else decel
        if rate <= 0.0:
            return tgt
        allowed = rate * dt
        delta = tgt - cur
        if abs(delta) <= allowed:
            return tgt
        return cur + math.copysign(allowed, delta)


def resolve(watchdog: CommandWatchdog, shaper: VelocityShaper,
            now: float, dt: float) -> Twist3:
    """The one supported way to combine the watchdog with the shaper.

    Exists because there is a wrong way that looks right:

        shaper.step(*watchdog.command(now), dt)      # WRONG on a stale command

    watchdog.command() already returns zero once the command has expired, so
    that line reads as though it stops -- but it feeds zero through the shaper,
    which RAMPS DOWN to it at the deceleration limit. A robot whose operator has
    gone silent would then keep rolling for as long as that ramp takes. Loss of
    the commander is exactly when smoothing must not apply.

    S3, S5: stale means stop, and stop means now.
    """
    if watchdog.is_stale(now):
        return shaper.stop()
    return shaper.step(*watchdog.command(now), dt=dt)


class LinkMonitor:
    """Tracks whether the ESP32 is answering, and how well (S2, S13).

    WHY THE RTT IS ONLY EVER AN ESTIMATE. The protocol has no sequence number:
    an "ACK <a> <s> <r>" cannot be matched to the packet that caused it. While
    the robot drives steadily every packet is identical, so the content does not
    disambiguate either. This class pairs each reply with the oldest unmatched
    send, which is right only if delivery is in order and nothing is lost.
    On the measured network -- roughly 138 ms average RTT against a 50 ms send
    period, so two or three packets in flight, with 7.7% loss -- neither holds
    reliably, and a lost packet makes the next estimate read HIGH.

    So treat rtt as an upper-ish bound useful for spotting a change, not as a
    measurement. `last_reply_age` below IS exact and needs no pairing, and it is
    the number that actually relates to the firmware's 400 ms deadman. Prefer
    it. A real RTT would need a sequence number in the packet, which means a
    firmware change.

    NO LOGGING HAPPENS HERE. This records; the node decides what is worth
    saying. Everything is O(1) per cycle with a bounded deque -- this runs at
    the control rate on a Pi.
    """

    NEVER_SEEN = "never_seen"
    UP = "up"
    DOWN = "down"

    def __init__(self, reply_timeout: float = 1.0, window: int = 64) -> None:
        if reply_timeout <= 0.0:
            raise ValueError("reply_timeout must be > 0 seconds")
        self.reply_timeout = float(reply_timeout)

        self._unmatched: Deque[float] = deque(maxlen=max(1, window))
        self._first_send: Optional[float] = None
        self._last_reply: Optional[float] = None
        self._rtt: Optional[float] = None
        self.sent = 0
        self.received = 0
        self.send_errors = 0
        self._state = self.NEVER_SEEN

    # ------------------------------------------------------------------ events
    def on_send(self, now: float) -> None:
        self.sent += 1
        if self._first_send is None:
            self._first_send = now
        # Bounded: if the far end is dead this would otherwise grow forever.
        # Dropping the oldest is also what makes the estimate self-correcting
        # once replies resume.
        self._unmatched.append(now)

    def on_send_error(self) -> None:
        """A sendto() that raised -- WiFi down, host unreachable, no route."""
        self.send_errors += 1

    def on_reply(self, now: float) -> None:
        self.received += 1
        self._last_reply = now
        if self._unmatched:
            rtt = _elapsed(now, self._unmatched.popleft())
            if rtt is not None:
                self._rtt = rtt

    # ------------------------------------------------------------------ state
    def last_reply_age(self, now: float) -> Optional[float]:
        """Exact seconds since the last reply, or None if there has never been one."""
        return _elapsed(now, self._last_reply)

    @property
    def rtt(self) -> Optional[float]:
        """Approximate round trip, seconds. Read the class docstring first."""
        return self._rtt

    @property
    def reply_ratio(self) -> float:
        """Replies per packet sent, over the life of the node. 1.0 is perfect."""
        return (self.received / self.sent) if self.sent else 0.0

    def state(self, now: float) -> str:
        if self._last_reply is None:
            # NEVER_SEEN is only the STARTING state, and it decays into DOWN
            # once we have been transmitting for longer than a reply could
            # plausibly take. It has to decay, or a wrong esp32_ip would sit in
            # NEVER_SEEN forever, poll() would never report a change, and the
            # node would stay silent about a link that has never once worked --
            # which is precisely the failure this class replaced. The node
            # tells the two DOWNs apart with `received == 0` and says something
            # more useful for each.
            since_first = _elapsed(now, self._first_send)
            if since_first is None or since_first <= self.reply_timeout:
                return self.NEVER_SEEN
            return self.DOWN
        age = self.last_reply_age(now)
        return self.UP if age is not None and age <= self.reply_timeout else self.DOWN

    def poll(self, now: float) -> Optional[str]:
        """Returns the new state ONLY when it changes, else None.

        Lets the node log transitions and stay quiet in between, which is what
        S2 means by not spamming the terminal.
        """
        new = self.state(now)
        if new == self._state:
            return None
        self._state = new
        return new

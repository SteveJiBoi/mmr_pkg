"""Key bindings and key-to-Twist logic for the keyboard teleop node.

No ROS import, no termios import. That is what lets the mapping be tested on any
machine, and it is why the node in kb_teleop.py contains almost nothing but
terminal handling.


WHAT A TERMINAL CAN AND CANNOT TELL YOU
---------------------------------------
It cannot tell you a key was RELEASED. stdin delivers characters, and a
character means "this key went down", or "the auto-repeat fired". There is no
key-up event to read. Anyone who claims otherwise is either using a windowing
toolkit or is wrong.

So release is INFERRED: hold a key and the OS repeats it, let go and the repeats
stop, so a key that has not been seen for key_timeout is treated as released.
That works, with two consequences worth knowing before you drive:

  * There is up to key_timeout of lag between letting go and stopping. It is
    bounded, it is not zero, and on a real robot it is distance travelled.
  * Auto-repeat does not start instantly. A typical console waits ~500 ms
    before the first repeat, so key_timeout must be LONGER than that delay or
    a held key will stutter. That is the whole reason the default is 0.6 s and
    not something snappier.

    Shorten both together if you want a tighter feel. On X11:
        xset r rate 200 30      # 200 ms to first repeat, 30 ms between
    then run with -p key_timeout:=0.3

SPACE does not depend on any of this. It is an explicit, immediate, unambiguous
stop, and it is the control to reach for. The firmware's 400 ms deadman remains
the backstop underneath everything.

Holding two keys for a diagonal works only as well as the terminal allows: most
terminals repeat only the MOST RECENTLY pressed key, so the other one coasts on
its timeout and drops out after key_timeout. Tap-and-hold diagonals are fine;
long diagonals drift into a single axis.
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

Twist3 = Tuple[float, float, float]

# ===========================================================================
# THE KEY MAP. THIS IS THE ONE PLACE TO CHANGE KEY BINDINGS.
#
# Values are unit DIRECTIONS in REP-103 body axes, scaled at use time by the
# current linear/angular speed. They are not speeds; do not put m/s here.
#
#     +x = forward     +y = LEFT     +z = counter-clockwise seen from above
#
# Note X, not S, is reverse: W/X keeps forward and back on one column with the
# strafe keys either side of the middle row, so the six driving keys form a
# block under the left hand.
# ===========================================================================
MOVE_BINDINGS: Dict[str, Twist3] = {
    "w": (+1.0, 0.0, 0.0),      # forward
    "x": (-1.0, 0.0, 0.0),      # backward
    "a": (0.0, +1.0, 0.0),      # strafe left
    "d": (0.0, -1.0, 0.0),      # strafe right
    "q": (0.0, 0.0, +1.0),      # rotate left  (counter-clockwise)
    "e": (0.0, 0.0, -1.0),      # rotate right (clockwise)
}

#: Immediate, unconditional stop. Not subject to key_timeout.
STOP_KEYS = frozenset({" "})

#: Quit cleanly. Ctrl-C also works and is handled the same way.
QUIT_KEYS = frozenset({"\x03", "k"})       # \x03 is Ctrl-C

#: (linear multiplier, angular multiplier) applied on each press.
SPEED_BINDINGS: Dict[str, Tuple[float, float]] = {
    "=": (1.1, 1.1), "+": (1.1, 1.1),      # both faster
    "-": (0.9, 0.9), "_": (0.9, 0.9),      # both slower
    "]": (1.0, 1.1),                       # turn faster only
    "[": (1.0, 0.9),                       # turn slower only
}

#: Below this the robot almost certainly will not overcome its own stiction,
#: and a teleop that silently commands an unmovable speed is confusing.
MIN_SPEED = 0.05

BANNER = """\
mmr_pkg keyboard teleop  --  hold a key to drive, release to stop

      q   w   e        w / x   forward / back
        a   d          a / d   strafe left / right
          x            q / e   turn left / right

    SPACE  stop now          + / -    faster / slower
    k      quit              [ / ]    turn slower / faster

Release is inferred from auto-repeat, so stopping lags by up to key_timeout.
SPACE is immediate. This window must keep focus for any of it to work.
"""


class TeleopState:
    """Tracks which keys are live and turns that into a Twist.

    Time arrives as float seconds from the caller; this class never reads a
    clock, so tests can drive it through any timeline they like.
    """

    def __init__(self,
                 linear_speed: float = 0.5,
                 angular_speed: float = 1.0,
                 key_timeout: float = 0.6,
                 max_linear_speed: float = 1.0,
                 max_angular_speed: float = 1.0) -> None:
        if key_timeout <= 0.0:
            raise ValueError("key_timeout must be > 0 seconds")
        if max_linear_speed <= 0.0 or max_angular_speed <= 0.0:
            raise ValueError("max speeds must be > 0")

        self.key_timeout = float(key_timeout)
        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)
        # Clamped at construction too: a launch file that asks for more than
        # the ceiling should be corrected, not silently obeyed.
        self.linear_speed = self._clamp(linear_speed, max_linear_speed)
        self.angular_speed = self._clamp(angular_speed, max_angular_speed)

        self._held: Dict[str, float] = {}
        self.quit = False

    @staticmethod
    def _clamp(value: float, ceiling: float) -> float:
        return max(MIN_SPEED, min(float(ceiling), float(value)))

    # ------------------------------------------------------------------ input
    def handle(self, key: str, now: float) -> Optional[str]:
        """Process one character. Returns a line to show the user, or None.

        Unknown keys are ignored rather than treated as a stop: a stray
        keystroke should not be a control input, and SPACE is right there.
        """
        key = key.lower()

        if key in QUIT_KEYS:
            self.quit = True
            self._held.clear()
            return "quit"

        if key in STOP_KEYS:
            # Clearing the held set, not just zeroing the output, so that a key
            # still auto-repeating does not resurrect the motion a moment later.
            self._held.clear()
            return "STOP"

        if key in SPEED_BINDINGS:
            lin, ang = SPEED_BINDINGS[key]
            self.linear_speed = self._clamp(self.linear_speed * lin,
                                            self.max_linear_speed)
            self.angular_speed = self._clamp(self.angular_speed * ang,
                                             self.max_angular_speed)
            return self.speed_line()

        if key in MOVE_BINDINGS:
            self._held[key] = now
            return None

        return None

    # ----------------------------------------------------------------- output
    def twist(self, now: float) -> Twist3:
        """Sum of the still-live keys, scaled by the current speeds."""
        self._expire(now)
        vx = vy = wz = 0.0
        for key in self._held:
            dx, dy, dw = MOVE_BINDINGS[key]
            vx += dx * self.linear_speed
            vy += dy * self.linear_speed
            wz += dw * self.angular_speed
        return vx, vy, wz

    def stop(self) -> Twist3:
        self._held.clear()
        return 0.0, 0.0, 0.0

    def active_keys(self, now: float) -> Tuple[str, ...]:
        self._expire(now)
        return tuple(sorted(self._held))

    def speed_line(self) -> str:
        return (f"speed {self.linear_speed:.2f} m/s   "
                f"turn {self.angular_speed:.2f} rad/s")

    # -------------------------------------------------------------- internals
    def _expire(self, now: float) -> None:
        # A negative age means the clock stepped backwards; drop the key rather
        # than trust it. Same rule as bridge_core: if time is not trustworthy,
        # neither is the command, and "stop" is the safe reading of "unsure".
        self._held = {k: t for k, t in self._held.items()
                      if 0.0 <= now - t <= self.key_timeout}

"""ROS 2 node: keyboard -> /cmd_vel. Run it on the DESKTOP, not the Pi.

    ros2 run mmr_pkg kb_teleop
    ros2 run mmr_pkg kb_teleop --ros-args -p linear_speed:=0.3 -p key_timeout:=0.3

W/X forward and back, A/D strafe, Q/E turn, SPACE stop, k quit.

MUST BE RUN FROM A TERMINAL YOU ARE TYPING INTO. It reads stdin in cbreak mode,
so it cannot be started from a launch file: launch gives its children no
controlling terminal, and the node would sit there receiving nothing while
looking perfectly healthy. It refuses to start in that case rather than pretend.
That is also why launch/teleop.launch.py starts RViz only and leaves this to a
second terminal.

The bindings and all the key logic are in mmr_pkg/teleop_keys.py, which imports
neither ROS nor termios so the mapping can be tested anywhere. Change key
bindings THERE, in MOVE_BINDINGS -- one dict, one place.

This file is the terminal handling and the ROS publisher, and nothing else.
"""
from __future__ import annotations

import os
import sys
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

from mmr_pkg.teleop_keys import BANNER, TeleopState

# POSIX-only, and this node is POSIX-only, but importing at module scope would
# make the file unimportable on Windows where much of this repo is developed.
# Failing at construction with an explanation beats failing at import with an
# ImportError that names a module nobody asked for.
try:
    import select
    import termios
    import tty
    _TERMINAL_OK = True
except ImportError:                                  # pragma: no cover
    _TERMINAL_OK = False


class RawTerminal:
    """stdin in cbreak mode, restored on the way out no matter what.

    cbreak rather than full raw: it leaves output post-processing alone, so a
    logger writing to the same terminal still produces readable lines instead of
    a staircase. Ctrl-C still arrives as \\x03 for us to handle.
    """

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self._saved: Optional[list] = None

    def __enter__(self) -> "RawTerminal":
        self._saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc) -> None:
        if self._saved is not None:
            # TCSADRAIN, not TCSANOW: let queued output finish first, otherwise
            # the last status line can be truncated mid-write.
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._saved)

    def read_pending(self) -> List[str]:
        """Every character waiting right now. Never blocks.

        Blocking here would stall the publish timer, and the whole point of the
        fixed rate is that the ESP32 deadman is fed on a clock.
        """
        keys: List[str] = []
        while select.select([self.fd], [], [], 0.0)[0]:
            ch = os.read(self.fd, 1).decode("utf-8", errors="ignore")
            if not ch:
                break
            if ch == "\x1b":
                # An arrow key is ESC [ A. Read as individual characters that
                # '[' would land on the "turn slower" binding, so the tail of
                # any escape sequence is consumed and thrown away here.
                while select.select([self.fd], [], [], 0.0)[0]:
                    if os.read(self.fd, 1).decode("utf-8", "ignore") not in "[;0123456789":
                        break
                continue
            keys.append(ch)
        return keys


class KeyboardTeleop(Node):

    def __init__(self) -> None:
        super().__init__("kb_teleop")

        p = self.declare_parameter
        p("linear_speed", 0.5)          # m/s   at full deflection
        p("angular_speed", 1.0)         # rad/s at full deflection

        # Ceilings, so + cannot wind the speed past what the bridge accepts.
        # Match these to the bridge's own max_linear_speed / max_angular_speed;
        # both default to 1.0, which is the normalisation described in
        # esp32_protocol.py, NOT a measured speed for this robot.
        p("max_linear_speed", 1.0)
        p("max_angular_speed", 1.0)

        # Longer than the console's initial auto-repeat delay (~500 ms on a
        # typical setup) or a held key stutters. See teleop_keys.py.
        p("key_timeout", 0.6)

        p("publish_rate", 25.0)         # Hz, inside the 20-30 the brief asks for
        p("cmd_vel_topic", "cmd_vel")
        p("use_stamped_cmd_vel", False)

        g = self.get_parameter
        rate = float(g("publish_rate").value)
        if rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0 Hz, got {rate}")

        self.state = TeleopState(
            linear_speed=float(g("linear_speed").value),
            angular_speed=float(g("angular_speed").value),
            key_timeout=float(g("key_timeout").value),
            max_linear_speed=float(g("max_linear_speed").value),
            max_angular_speed=float(g("max_angular_speed").value))

        self.stamped = bool(g("use_stamped_cmd_vel").value)
        topic = str(g("cmd_vel_topic").value)
        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, topic, 10)

        self.term = RawTerminal()
        self._last_status = ""
        self.create_timer(1.0 / rate, self._tick)

        sys.stdout.write(BANNER + "\n")
        self._status(self.state.speed_line())

    # ------------------------------------------------------------------ time
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------- loop
    def _tick(self) -> None:
        now = self._now()

        for key in self.term.read_pending():
            note = self.state.handle(key, now)
            if note is not None:
                self._status(note if note != "quit" else "quit")

        if self.state.quit:
            # Just a flag. main() owns the loop and checks it, rather than this
            # raising through a timer callback and depending on how the
            # executor chooses to propagate exceptions out of one.
            return

        vx, vy, wz = self.state.twist(now)
        self._publish(vx, vy, wz)

        # Rewritten only when the text actually changes. A 25 Hz terminal
        # repaint is pure waste and makes the log unreadable.
        held = "".join(self.state.active_keys(now)) or "-"
        self._status(f"{self.state.speed_line()}   keys [{held}]"
                     f"   vx {vx:+.2f}  vy {vy:+.2f}  wz {wz:+.2f}")

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = Twist()
            twist = msg
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.pub.publish(msg)

    def _status(self, line: str) -> None:
        if line == self._last_status:
            return
        self._last_status = line
        # \r and pad to the previous width so a shorter line does not leave
        # fragments of the longer one behind.
        sys.stdout.write("\r" + line.ljust(78))
        sys.stdout.flush()

    # --------------------------------------------------------------- shutdown
    def send_stop(self, attempts: int = 3, spacing: float = 0.02) -> None:
        """Publish zeros on the way out so nothing downstream is left driving.

        Three, spaced, for the same reason the bridge does: this is the message
        that matters and the transport below it loses packets. Belt and braces
        over the bridge's own cmd_timeout and the firmware's 400 ms deadman --
        neither of which this node can rely on being present.
        """
        self.state.stop()
        for i in range(attempts):
            self._publish(0.0, 0.0, 0.0)
            if i + 1 < attempts:
                time.sleep(spacing)


def main(args=None) -> int:
    if not _TERMINAL_OK:
        print("kb_teleop needs a POSIX terminal (termios); this platform has none.")
        return 2
    if not sys.stdin.isatty():
        # The failure this prevents is nasty: started from a launch file the
        # node runs happily, publishes a steady stream of zeros, and every
        # diagnostic looks fine while no key ever arrives.
        print("kb_teleop: stdin is not a terminal.\n"
              "Run it directly in a terminal you are typing into:\n"
              "    ros2 run mmr_pkg kb_teleop\n"
              "It cannot be started from a launch file, which gives its "
              "children no controlling terminal.")
        return 2

    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        with node.term:
            # An explicit loop rather than rclpy.spin(), because quitting on
            # the 'k' key has to leave the loop from inside a timer callback,
            # and a flag checked here is unambiguous where an exception thrown
            # through the executor is not. timeout_sec bounds how long Ctrl-C
            # takes to be noticed.
            while rclpy.ok() and not node.state.quit:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f"kb_teleop: {exc}")
        return 2
    finally:
        if node is not None:
            # After the terminal is restored, so the summary is readable.
            sys.stdout.write("\n")
            node.send_stop()
            node.get_logger().info("stopped; sent zero /cmd_vel")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

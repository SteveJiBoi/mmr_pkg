"""ROS 2 node: /cmd_vel -> UDP packets for the ESP32 omni firmware.

Bridges the ROS side (SI units, REP-103 frames) to the ESP32 sketch in
esp32/MotionTestOriginal/MotionTestOriginal.ino, which speaks a small ASCII UDP
protocol and thinks in raw PWM.

    ros2 run mmr_pkg esp32_bridge --ros-args -p esp32_ip:=10.229.5.249

It requires NO firmware change. The sketch already accepts a polar command and
does its own wheel mixing, so this node reproduces exactly what
esp32/controller.py sends and nothing more.

THIS FILE IS ONLY PLUMBING -- subscribe, time, send, publish. Every decision
lives in a module with no ROS import, so it can be tested on a machine with no
ROS install:

    mmr_pkg/esp32_protocol.py   the wire format and the SI -> PWM arithmetic
    mmr_pkg/bridge_core.py      the watchdog, the limits, the link statistics

Read those before changing any number here. That split is load bearing: an
earlier revision kept the policy in this class, nothing could import it without
rclpy, so nothing tested it, and a stale attribute reference on the last line of
__init__ shipped under a green test suite and stopped the node from starting at
all.

If you would rather move the kinematics into ROS and send three wheel PWMs --
the better long-term design, because kiwi_kinematics.py then becomes the single
source of truth for both simulation and hardware -- see README "Option B" and
run tools/firmware_diff.py first.

Topics
------
  subscribe   /cmd_vel              geometry_msgs/Twist (or TwistStamped)
  publish     /diagnostics          diagnostic_msgs/DiagnosticArray, at 1 Hz
  publish     ~/esp32_reply         std_msgs/String, raw ACK text, OFF by default

Safety
------
Three layers, in order of how much they can be relied on:

  1. this node zeroes the command if /cmd_vel goes quiet for cmd_timeout;
  2. it sends a burst of explicit stops on the way out;
  3. the firmware brakes on its own if no packet arrives for 400 ms.

Only the third survives this process being killed, the Pi sleeping or WiFi
dropping, so it is the one that actually matters. The first two exist to stop
the robot a control cycle after the operator lets go rather than 400 ms after.
"""
from __future__ import annotations

import socket
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from mmr_pkg.bridge_core import (CommandWatchdog, LinkMonitor, VelocityShaper,
                                 resolve, resolve_endpoint)
from mmr_pkg.esp32_protocol import (DEFAULT_ANGULAR_PWM, DEFAULT_LINEAR_PWM,
                                    FIRMWARE_DEADMAN_MS, FIRMWARE_UDP_PORT,
                                    PacketEncoder)

#: Sent on the way out. Spaced rather than bursted -- see _emergency_stop.
STOP_PACKET = b"A,0,0,0"


class Esp32Bridge(Node):

    def __init__(self) -> None:
        super().__init__("esp32_bridge")

        p = self.declare_parameter

        # ------------------------------------------------------------ link
        # No default IP. The address is per-robot and a wrong one fails
        # silently -- UDP has no connection, so packets to a dead host just
        # vanish and the robot sits there looking broken. Better to refuse to
        # start.
        p("esp32_ip", "")
        p("esp32_port", FIRMWARE_UDP_PORT)

        # ------------------------------------------------------- normalisation
        # See esp32_protocol.py, "UNITS", before touching any of these. The PWM
        # values are known-good from controller.py; the SI values are a
        # declared normalisation, NOT a measurement of this robot.
        #
        # max_linear_speed is declared ONCE and used twice: as the ceiling the
        # shaper clamps to, and as the denominator of the PWM map. One number,
        # one meaning, so the two stages cannot disagree.
        p("max_linear_speed", 1.0)      # m/s   mapping to max_linear_pwm
        p("max_angular_speed", 1.0)     # rad/s mapping to max_angular_pwm
        p("max_linear_pwm", DEFAULT_LINEAR_PWM)
        p("max_angular_pwm", DEFAULT_ANGULAR_PWM)

        # ------------------------------------------------------- acceleration
        # 0.0 means "no limit", and that is the default because there is no
        # measured acceleration figure for this robot -- it has no encoders, so
        # any number here would be invented rather than observed. Set them from
        # what you actually see if the robot lurches.
        #
        # Braking is deliberately unlimited by default: a slow ramp up is
        # comfortable, a slow ramp down is a robot that will not stop when told.
        p("max_linear_accel", 0.0)      # m/s^2
        p("max_angular_accel", 0.0)     # rad/s^2
        p("max_linear_decel", 0.0)      # m/s^2,  0 = brake as hard as asked
        p("max_angular_decel", 0.0)     # rad/s^2

        # Wiring-dependent signs; see esp32_protocol.py.
        p("strafe_sign", 1)             # flip to -1 if A/D are swapped
        p("yaw_sign", -1)               # -1 reproduces controller.py exactly

        # ------------------------------------------------------------ timing
        p("publish_rate", 20.0)         # Hz, must stay well under the 400 ms
                                        # firmware deadman
        p("cmd_timeout", 0.5)           # s, zero the command if /cmd_vel is quiet
        p("reply_timeout", 1.0)         # s, call the link down after this
        p("diagnostic_period", 1.0)     # s, /diagnostics rate. Deliberately far
                                        # slower than the control loop.
        p("cmd_vel_topic", "cmd_vel")
        p("use_stamped_cmd_vel", False)

        # Every ACK republished as a ROS String is ~20 messages/second of
        # allocation and serialisation that nothing normally reads. Useful when
        # you are debugging the link, wasteful on a Pi the rest of the time.
        p("publish_replies", False)

        # The firmware clamps each wheel to +-255 AFTER subtracting rot, so a
        # full-speed translation combined with a full-speed spin saturates and
        # the robot curves away from the commanded heading instead of just
        # going slower. Enabling this scales translation+rotation down together
        # so their sum fits, which keeps the heading honest at the cost of
        # speed. Default False = behave exactly like controller.py.
        p("prevent_clipping", False)

        g = self.get_parameter

        # ------------------------------------------------------------ endpoint
        # Resolved once, here, so the control loop never does DNS. Raises
        # ValueError with an actionable message; main() turns that into a clean
        # exit rather than a traceback.
        self.ip, self.port = resolve_endpoint(str(g("esp32_ip").value),
                                              int(g("esp32_port").value))

        rate = float(g("publish_rate").value)
        if rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0 Hz, got {rate}")
        period = 1.0 / rate
        deadman = FIRMWARE_DEADMAN_MS / 1000.0
        if period > deadman / 2.0:
            self.get_logger().warn(
                f"publish_rate {rate:g} Hz gives a {period * 1e3:.0f} ms gap "
                f"between packets; the firmware brakes after "
                f"{FIRMWARE_DEADMAN_MS} ms, so a single lost packet may now "
                f"stall the robot. 20 Hz or faster is the intended range.")

        # ------------------------------------------------------------- policy
        self.watchdog = CommandWatchdog(timeout=float(g("cmd_timeout").value))
        self.shaper = VelocityShaper(
            max_linear_speed=float(g("max_linear_speed").value),
            max_angular_speed=float(g("max_angular_speed").value),
            max_linear_accel=float(g("max_linear_accel").value),
            max_angular_accel=float(g("max_angular_accel").value),
            max_linear_decel=float(g("max_linear_decel").value),
            max_angular_decel=float(g("max_angular_decel").value))
        self.enc = PacketEncoder(
            max_linear_speed=float(g("max_linear_speed").value),
            max_angular_speed=float(g("max_angular_speed").value),
            max_linear_pwm=int(g("max_linear_pwm").value),
            max_angular_pwm=int(g("max_angular_pwm").value),
            strafe_sign=int(g("strafe_sign").value),
            yaw_sign=int(g("yaw_sign").value),
            prevent_clipping=bool(g("prevent_clipping").value))
        self.link = LinkMonitor(reply_timeout=float(g("reply_timeout").value))

        # ------------------------------------------------------------ socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)     # never let recv stall the control loop

        # ------------------------------------------------------------- state
        self._last_tick: Optional[float] = None
        self._last_packet = b""

        # --------------------------------------------------------- ROS wiring
        topic = str(g("cmd_vel_topic").value)
        if bool(g("use_stamped_cmd_vel").value):
            self.create_subscription(TwistStamped, topic,
                                     lambda m: self._on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, topic, self._on_cmd, 10)

        self.reply_pub = (self.create_publisher(String, "~/esp32_reply", 10)
                          if bool(g("publish_replies").value) else None)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_timer(period, self._tick)
        self.create_timer(float(g("diagnostic_period").value), self._publish_diagnostics)

        self.get_logger().info(
            f"bridging {topic} -> udp {self.ip}:{self.port} at {rate:g} Hz")
        # Read off the encoder rather than keeping a second copy of the numbers
        # on this node. One owner per value; the mirror is what rotted before.
        self.get_logger().warn(
            f"speed map is a NORMALISATION, not a measurement: {self.enc}. "
            f"See README 'Calibrating the ESP32 bridge'.")

    # ------------------------------------------------------------------ time
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------- cmd
    def _on_cmd(self, msg: Twist) -> None:
        # Deliberately does nothing but record. Sending from the subscription
        # would tie the packet rate to whatever the publisher happens to do,
        # and the firmware deadman needs a packet on a clock, not on an event.
        self.watchdog.update(msg.linear.x, msg.linear.y, msg.angular.z,
                             self._now())

    # ------------------------------------------------------------ control loop
    def _tick(self) -> None:
        now = self._now()

        # Measured dt, not the nominal timer period: a Pi under load runs the
        # timer late, and acceleration limiting that assumes a perfect period
        # would ramp at the wrong rate exactly when the machine is struggling.
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        self._last_tick = now

        # resolve() is the only supported way to combine these two: a stale
        # command must HARD stop, never ramp down. See bridge_core.resolve.
        vx, vy, wz = resolve(self.watchdog, self.shaper, now, dt)

        # Keep transmitting zeros rather than going silent: an explicit
        # "A,0,0,0" stops the robot immediately, whereas silence only stops it
        # 400 ms later when the deadman expires. Transmitting while parked also
        # keeps the link statistics live, so you can tell a healthy idle robot
        # from an unreachable one.
        self._send(self.enc.packet(vx, vy, wz), now)
        self._drain_replies(now)

        change = self.link.poll(now)
        if change is not None:
            self._report_link(change, now)

    def _send(self, packet: bytes, now: float) -> None:
        self._last_packet = packet
        try:
            self.sock.sendto(packet, (self.ip, self.port))
            self.link.on_send(now)
        except OSError as exc:
            # WiFi dropped, no route, buffer full. Not fatal and not worth a
            # line per cycle: the socket is connectionless, so the next send
            # simply works again once the network returns. S14's "automatic
            # recovery" is this doing nothing special.
            self.link.on_send_error()
            self.get_logger().warn(f"udp send failed: {exc}",
                                   throttle_duration_sec=5.0)

    def _drain_replies(self, now: float) -> None:
        # Drains everything queued, not one per cycle: replies arrive in bursts
        # after a stall, and leaving them in the receive buffer would make the
        # link look worse than it is and eventually drop packets. The socket is
        # non-blocking, so the loop always terminates on BlockingIOError.
        while True:
            try:
                data, _ = self.sock.recvfrom(64)
            except (BlockingIOError, ConnectionResetError):
                return
            except OSError:
                return
            # The tick's timestamp rather than a fresh clock read per reply:
            # they differ by microseconds and this runs at the control rate.
            self.link.on_reply(now)
            if self.reply_pub is not None:
                msg = String()
                msg.data = data.decode("ascii", errors="replace").strip()
                self.reply_pub.publish(msg)

    def _report_link(self, state: str, now: float) -> None:
        """One line per TRANSITION. Nothing at all while the state holds."""
        if state == LinkMonitor.UP:
            self.get_logger().info(
                f"link up: ESP32 at {self.ip} is acknowledging"
                + (f" (~{self.link.rtt * 1e3:.0f} ms)" if self.link.rtt else ""))
        elif state == LinkMonitor.DOWN and self.link.received == 0:
            # Never once answered. Overwhelmingly a wrong address or an
            # unpowered board, so say that instead of "link lost".
            self.get_logger().error(
                f"no reply from {self.ip}:{self.port} since start-up. The "
                f"robot is probably NOT moving. Check that esp32_ip matches "
                f"the address the ESP32 prints at boot, that it is powered, "
                f"and that both are on the same network.")
        elif state == LinkMonitor.DOWN:
            age = self.link.last_reply_age(now)
            quiet = f" for {age:.1f} s" if age is not None else ""
            self.get_logger().warn(f"link lost: no ACK from {self.ip}{quiet}")

    # ------------------------------------------------------------ diagnostics
    def _publish_diagnostics(self) -> None:
        """S13, at 1 Hz rather than in the control loop.

        Answers, without anyone having to read the log:
          ros2 topic echo /diagnostics
        """
        now = self._now()
        state = self.link.state(now)
        age = self.link.last_reply_age(now)
        cmd_age = self.watchdog.age(now)

        if state == LinkMonitor.UP:
            level, text = DiagnosticStatus.OK, "ESP32 acknowledging"
        elif self.link.received == 0:
            level, text = DiagnosticStatus.ERROR, "never answered; check esp32_ip"
        else:
            level, text = DiagnosticStatus.WARN, "no recent ACK"

        link = DiagnosticStatus(
            level=level, name="esp32_bridge: link", message=text,
            hardware_id=f"{self.ip}:{self.port}",
            values=[
                KeyValue(key="state", value=state),
                KeyValue(key="last_reply_age_s",
                         value=f"{age:.3f}" if age is not None else "never"),
                # Approximate, and the protocol cannot do better: no sequence
                # number means an ACK cannot be matched to its packet. See
                # LinkMonitor. last_reply_age_s above is exact; prefer it.
                KeyValue(key="rtt_estimate_s",
                         value=f"{self.link.rtt:.3f}" if self.link.rtt else "unknown"),
                KeyValue(key="packets_sent", value=str(self.link.sent)),
                KeyValue(key="replies_received", value=str(self.link.received)),
                KeyValue(key="reply_ratio", value=f"{self.link.reply_ratio:.3f}"),
                KeyValue(key="send_errors", value=str(self.link.send_errors)),
            ])

        vx, vy, wz = self.shaper.current
        stale = self.watchdog.is_stale(now)
        cmd = DiagnosticStatus(
            level=DiagnosticStatus.WARN if stale else DiagnosticStatus.OK,
            name="esp32_bridge: command",
            message="stale, sending zeros" if stale else "fresh",
            hardware_id=f"{self.ip}:{self.port}",
            values=[
                KeyValue(key="cmd_vel_age_s",
                         value=f"{cmd_age:.3f}" if cmd_age is not None else "never"),
                KeyValue(key="cmd_timeout_s", value=f"{self.watchdog.timeout:.3f}"),
                KeyValue(key="sent_vx_mps", value=f"{vx:.3f}"),
                KeyValue(key="sent_vy_mps", value=f"{vy:.3f}"),
                KeyValue(key="sent_wz_rps", value=f"{wz:.3f}"),
                KeyValue(key="last_packet",
                         value=self._last_packet.decode("ascii", "replace")),
            ])

        msg = DiagnosticArray(status=[link, cmd])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(msg)

    # --------------------------------------------------------------- shutdown
    def emergency_stop(self, attempts: int = 3, spacing: float = 0.02) -> None:
        """Best effort full stop, called on the way out.

        Spaced, not bursted. WiFi loss tends to arrive in bursts, so three
        packets emitted back to back are far more likely to share a fate than
        three spread over 40 ms. This is the one packet that really matters and
        it is being sent over a link measured at 7.7% loss.

        If every attempt fails the robot still stops: the firmware's 400 ms
        deadman is the backstop, and it is the only layer that survives this
        process dying rather than exiting.
        """
        for i in range(attempts):
            try:
                self.sock.sendto(STOP_PACKET, (self.ip, self.port))
            except OSError:
                pass
            if i + 1 < attempts:
                time.sleep(spacing)

    def destroy_node(self) -> bool:
        try:
            self.sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = Esp32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        # Bad esp32_ip or an out-of-range rate: a configuration mistake, not a
        # crash. Say what is wrong and exit non-zero, without a traceback that
        # buries the message.
        print(f"esp32_bridge: {exc}")
        return 2
    finally:
        if node is not None:
            node.emergency_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

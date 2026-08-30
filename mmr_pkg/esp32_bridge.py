"""ROS 2 node: /cmd_vel -> UDP packets for the ESP32 omni firmware.

Bridges the ROS side (SI units, REP-103 frames) to the ESP32 sketch in
esp32/MotionTestOriginal/MotionTestOriginal.ino, which speaks a small ASCII UDP
protocol and thinks in raw PWM.

    ros2 run mmr_pkg esp32_bridge --ros-args -p esp32_ip:=10.229.5.249

It requires NO firmware change. The sketch already accepts a polar command and
does its own wheel mixing, so this node reproduces exactly what
esp32/controller.py sends and nothing more.

This file is only the plumbing. The protocol, the units, the sign conventions
and the arithmetic all live in mmr_pkg/esp32_protocol.py, which has no ROS
dependency so it can be tested and diffed against the microcontroller without a
ROS install -- the same split as kiwi_drive_node.py / kiwi_kinematics.py.
READ THAT FILE before changing any number here.

If you would rather move the kinematics into ROS and send three wheel PWMs --
the better long-term design, because kiwi_kinematics.py then becomes the single
source of truth for both simulation and hardware -- see README "Option B" and
run tools/firmware_diff.py first.

Topics
------
  subscribe   /cmd_vel              geometry_msgs/Twist (or TwistStamped)
  publish     ~/esp32_reply         std_msgs/String, the raw ACK/PONG text

Safety
------
Two independent watchdogs. This node stops sending motion if /cmd_vel goes
quiet for cmd_timeout; the firmware independently brakes if no packet arrives
for 400 ms. The second one is the one that matters, because it survives this
node being killed, the PC sleeping, and WiFi dropping.
"""
from __future__ import annotations

import socket

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from mmr_pkg.esp32_protocol import (DEFAULT_ANGULAR_PWM, DEFAULT_LINEAR_PWM,
                                    FIRMWARE_UDP_PORT, PacketEncoder)


class Esp32Bridge(Node):

    def __init__(self):
        super().__init__("esp32_bridge")

        p = self.declare_parameter

        # ------------------------------------------------------------ link
        # No default IP. The address is per-robot and a wrong one fails
        # silently -- UDP has no connection, so packets to a dead host just
        # vanish and the robot sits there looking broken. Better to refuse to
        # start. The ESP32 prints its address on the serial console at boot
        # ("Ready. IP: ...").
        p("esp32_ip", "")
        p("esp32_port", FIRMWARE_UDP_PORT)

        # ------------------------------------------------------- normalisation
        # See esp32_protocol.py, "UNITS", before touching any of these. The PWM
        # values are known-good from controller.py; the SI values are a
        # declared normalisation, NOT a measurement of this robot.
        p("max_linear_speed", 1.0)      # m/s   mapping to max_linear_pwm
        p("max_angular_speed", 1.0)     # rad/s mapping to max_angular_pwm
        p("max_linear_pwm", DEFAULT_LINEAR_PWM)
        p("max_angular_pwm", DEFAULT_ANGULAR_PWM)

        # Wiring-dependent signs; see docstring.
        p("strafe_sign", 1)             # flip to -1 if A/D are swapped
        p("yaw_sign", -1)               # -1 reproduces controller.py exactly

        # ------------------------------------------------------------ timing
        p("publish_rate", 20.0)         # Hz, must stay well under the 400 ms
                                        # firmware deadman
        p("cmd_timeout", 0.5)           # s, stop if /cmd_vel goes quiet
        p("cmd_vel_topic", "cmd_vel")
        p("use_stamped_cmd_vel", False)

        # The firmware clamps each wheel to +-255 AFTER subtracting rot, so a
        # full-speed translation combined with a full-speed spin saturates and
        # the robot curves away from the commanded heading instead of just
        # going slower. Enabling this scales translation+rotation down together
        # so their sum fits, which keeps the heading honest at the cost of
        # speed. Default False = behave exactly like controller.py.
        p("prevent_clipping", False)

        g = self.get_parameter
        self.ip = str(g("esp32_ip").value)
        self.port = int(g("esp32_port").value)
        if not self.ip:
            raise RuntimeError(
                "esp32_ip is not set. Start with e.g.\n"
                "  ros2 run mmr_pkg esp32_bridge --ros-args "
                "-p esp32_ip:=10.229.5.249\n"
                "The ESP32 prints its address on the serial console at boot.")

        self.cmd_timeout = float(g("cmd_timeout").value)
        self.enc = PacketEncoder(
            max_linear_speed=float(g("max_linear_speed").value),
            max_angular_speed=float(g("max_angular_speed").value),
            max_linear_pwm=int(g("max_linear_pwm").value),
            max_angular_pwm=int(g("max_angular_pwm").value),
            strafe_sign=int(g("strafe_sign").value),
            yaw_sign=int(g("yaw_sign").value),
            prevent_clipping=bool(g("prevent_clipping").value))

        # ------------------------------------------------------------ socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)     # never let recv stall the control loop

        # ------------------------------------------------------------- state
        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_time = None
        self._last_reply_time = None
        self._linked = False

        # --------------------------------------------------------- ROS wiring
        topic = str(g("cmd_vel_topic").value)
        if bool(g("use_stamped_cmd_vel").value):
            self.create_subscription(TwistStamped, topic,
                                     lambda m: self._on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, topic, self._on_cmd, 10)

        # Mirrors the ACK/PONG text back into the ROS graph so link health is
        # visible with `ros2 topic echo` instead of only in this node's log.
        self.reply_pub = self.create_publisher(String, "~/esp32_reply", 10)

        rate = float(g("publish_rate").value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"bridging {topic} -> udp {self.ip}:{self.port} at {rate:g} Hz")
        self.get_logger().warn(
            f"speed map is a NORMALISATION, not a measurement: "
            f"{self.max_v:g} m/s -> pwm {self.max_v_pwm}, "
            f"{self.max_w:g} rad/s -> pwm {self.max_w_pwm}. "
            f"See README 'Calibrating the ESP32 bridge'.")

    # ------------------------------------------------------------------ cmd
    def _on_cmd(self, msg: Twist):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()

        stale = (self._last_cmd_time is None or
                 (now - self._last_cmd_time).nanoseconds * 1e-9
                 > self.cmd_timeout)
        vx, vy, wz = (0.0, 0.0, 0.0) if stale else self._cmd

        # Keep transmitting zeros rather than going silent: an explicit
        # "A,0,0,0" stops the robot immediately, whereas silence only stops it
        # 400 ms later when the deadman expires. Transmitting while parked also
        # keeps the link-health indication live.
        try:
            self.sock.sendto(self.enc.packet(vx, vy, wz), (self.ip, self.port))
        except OSError as exc:
            self.get_logger().warn(f"udp send failed: {exc}", throttle_duration_sec=2.0)

        self._drain_replies()

        if self._last_reply_time is not None:
            quiet = (now - self._last_reply_time).nanoseconds * 1e-9
            if self._linked and quiet > 1.0:
                self._linked = False
                self.get_logger().warn("link lost: no ACK from the ESP32 for "
                                       "over 1 s")

    def _drain_replies(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(64)
            except (BlockingIOError, ConnectionResetError):
                return
            except OSError:
                return
            self._last_reply_time = self.get_clock().now()
            if not self._linked:
                self._linked = True
                self.get_logger().info("link up: ESP32 is acknowledging")
            msg = String()
            msg.data = data.decode("ascii", errors="replace").strip()
            self.reply_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Esp32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: tell the robot to stop on the way out rather than
        # relying on the deadman. Three times, because this is UDP and this is
        # the one packet that really matters.
        if node is not None:
            try:
                for _ in range(3):
                    node.sock.sendto(b"A,0,0,0", (node.ip, node.port))
            except OSError:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

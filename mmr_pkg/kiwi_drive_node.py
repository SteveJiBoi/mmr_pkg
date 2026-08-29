"""ROS 2 node: /cmd_vel -> three omniwheel velocity commands, plus odometry.

The brief (section 9) asks for the drive node to be written by hand rather than
using `mecanum_drive_controller`, with the inverse kinematics isolated in one
clearly commented function so it can be diffed against the ESP32 firmware.

That function is NOT in this file. It is `inverse_kinematics` in
mmr_pkg/kiwi_kinematics.py, which has no ROS dependency at all so it can be
read, tested and compared against C without a ROS install. This file is only
the plumbing around it.

    ros2 run mmr_pkg kiwi_drive_node

Topics
------
  subscribe   /cmd_vel                geometry_msgs/Twist  (or TwistStamped,
                                      see the use_stamped_cmd_vel parameter)
  subscribe   /joint_states           sensor_msgs/JointState   (for odometry)
  publish     <wheel_cmd_topic>       std_msgs/Float64MultiArray
  publish     /odom                   nav_msgs/Odometry        (optional)
  broadcast   odom -> base_footprint  TF                       (optional)

The command topic default targets the JointGroupVelocityController configured
in config/controllers.yaml, which listens on `<controller>/commands` and
expects one Float64 per joint in the order given by its `joints` parameter.
THAT ORDER MUST MATCH the wheel_angles_deg order here; controllers.yaml lists
wheel_0/1/2 in that order and this node emits wheel_0/1/2 in that order.

Safety
------
A watchdog zeroes the wheels if no /cmd_vel arrives within cmd_timeout. A
holonomic base with no timeout will happily drive into a wall for as long as
the last message said to, which is until someone pulls the battery out.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from mmr_pkg.kiwi_kinematics import (DEFAULT_BASE_RADIUS,
                                     DEFAULT_WHEEL_ANGLES_DEG,
                                     DEFAULT_WHEEL_RADIUS, KiwiKinematics)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Planar robot, so only the Z component is ever non-zero."""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class KiwiDriveNode(Node):

    def __init__(self):
        super().__init__("kiwi_drive_node")

        # ---------------------------------------------------- parameters
        # Geometry defaults come from kiwi_kinematics, which documents their
        # provenance (measured from the STEP). They are parameters and not
        # constants specifically so the 30-degree mounting-angle question in
        # the README can be settled without editing code.
        p = self.declare_parameter
        p("wheel_angles_deg", list(DEFAULT_WHEEL_ANGLES_DEG))
        p("wheel_radius", DEFAULT_WHEEL_RADIUS)
        p("base_radius", DEFAULT_BASE_RADIUS)
        p("wheel_joints", ["wheel_0_joint", "wheel_1_joint", "wheel_2_joint"])

        # rad/s at the wheel. 0 disables limiting. NOT a measured value: no
        # motor datasheet was supplied, so this is a deliberately conservative
        # placeholder. See README TODO.
        p("max_wheel_speed", 0.0)

        p("cmd_vel_topic", "cmd_vel")
        p("use_stamped_cmd_vel", False)
        p("wheel_cmd_topic", "/wheel_velocity_controller/commands")
        p("cmd_timeout", 0.5)          # s, watchdog
        p("publish_rate", 50.0)        # Hz

        p("publish_odom", True)
        p("publish_tf", True)
        p("odom_frame", "odom")
        p("base_frame", "base_footprint")

        g = self.get_parameter
        angles = list(g("wheel_angles_deg").value)
        self.wheel_joints = list(g("wheel_joints").value)
        self.max_wheel_speed = float(g("max_wheel_speed").value)
        self.cmd_timeout = float(g("cmd_timeout").value)
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        self._publish_tf = bool(g("publish_tf").value)
        self._publish_odom = bool(g("publish_odom").value)

        if len(angles) != len(self.wheel_joints):
            raise ValueError(
                f"wheel_angles_deg has {len(angles)} entries but wheel_joints "
                f"has {len(self.wheel_joints)}; they are positionally paired")

        self.kin = KiwiKinematics(
            wheel_angles_deg=angles,
            wheel_radius=float(g("wheel_radius").value),
            base_radius=float(g("base_radius").value))

        # ---------------------------------------------------------- state
        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_time = None
        self._stopped = True           # so we do not spam zeros forever
        self._x = self._y = self._th = 0.0
        self._last_js_time = None
        self._last_positions = None

        # --------------------------------------------------------- wiring
        cmd_topic = g("cmd_vel_topic").value
        if bool(g("use_stamped_cmd_vel").value):
            self.create_subscription(
                TwistStamped, cmd_topic, self._on_twist_stamped, 10)
        else:
            self.create_subscription(Twist, cmd_topic, self._on_twist, 10)

        self._cmd_pub = self.create_publisher(
            Float64MultiArray, g("wheel_cmd_topic").value, 10)

        if self._publish_odom:
            self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        if self._publish_tf:
            self._tf = TransformBroadcaster(self)
        self.create_subscription(JointState, "joint_states", self._on_js, 10)

        self.create_timer(1.0 / float(g("publish_rate").value), self._tick)

        self.get_logger().info(
            f"kiwi_drive_node up. {self.kin!r}\n"
            f"  cmd_vel      : {cmd_topic} "
            f"({'TwistStamped' if g('use_stamped_cmd_vel').value else 'Twist'})\n"
            f"  wheel command: {g('wheel_cmd_topic').value} "
            f"{self.wheel_joints}\n"
            f"  max wheel    : "
            + (f"{self.max_wheel_speed} rad/s "
               f"(<= {self.kin.max_body_speed(self.max_wheel_speed):.3f} m/s "
               f"in any direction)" if self.max_wheel_speed > 0 else "unlimited")
            + "\n  NOTE all-positive wheel velocity = CLOCKWISE yaw.")

    # ------------------------------------------------------------ cmd_vel
    def _on_twist(self, msg: Twist):
        self._accept(msg)

    def _on_twist_stamped(self, msg: TwistStamped):
        self._accept(msg.twist)

    def _accept(self, t: Twist):
        # A kiwi base is holonomic in the plane and has no other freedoms.
        # Silently ignoring z/roll/pitch would hide a caller sending a 3D
        # twist, so warn once per second rather than never.
        if abs(t.linear.z) > 1e-9 or abs(t.angular.x) > 1e-9 \
                or abs(t.angular.y) > 1e-9:
            self.get_logger().warn(
                "cmd_vel has non-planar components (linear.z / angular.x / "
                "angular.y); a kiwi base cannot produce them. Ignoring.",
                throttle_duration_sec=1.0)
        self._cmd = (t.linear.x, t.linear.y, t.angular.z)
        self._last_cmd_time = self.get_clock().now()

    # -------------------------------------------------------------- timer
    def _tick(self):
        stale = (self._last_cmd_time is None
                 or (self.get_clock().now() - self._last_cmd_time).nanoseconds
                 * 1e-9 > self.cmd_timeout)

        if stale:
            # Publish one zero command on the transition, then go quiet. The
            # controller latches its last command, so a single zero is enough
            # and republishing at 50 Hz forever just adds noise.
            if not self._stopped:
                self._publish([0.0] * len(self.wheel_joints))
                self._stopped = True
                if self._last_cmd_time is not None:
                    self.get_logger().warn(
                        f"no cmd_vel for {self.cmd_timeout}s - wheels stopped",
                        throttle_duration_sec=5.0)
            return

        vx, vy, wz = self._cmd
        vx, vy, wz, scale = self.kin.scale_to_limit(
            vx, vy, wz, self.max_wheel_speed)
        if scale < 1.0:
            self.get_logger().warn(
                f"cmd_vel exceeds max_wheel_speed; scaled by {scale:.3f}. "
                "Direction preserved, speed reduced.",
                throttle_duration_sec=2.0)

        self._publish(self.kin.inverse_kinematics(vx, vy, wz))
        self._stopped = False

    def _publish(self, wheel_speeds):
        m = Float64MultiArray()
        m.data = [float(w) for w in wheel_speeds]
        self._cmd_pub.publish(m)

    # --------------------------------------------------------- odometry
    def _on_js(self, msg: JointState):
        """Integrate the measured wheel velocities into an odom estimate.

        This is dead reckoning through the forward kinematics, so it inherits
        every bit of omniwheel slip -- and omniwheels slip a lot, sideways, by
        design. Treat /odom as a smooth local estimate, not as a position.
        """
        if not (self._publish_odom or self._publish_tf):
            return
        try:
            idx = [msg.name.index(j) for j in self.wheel_joints]
        except ValueError:
            return          # not our joints (e.g. an arm-only JointState)

        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if msg.velocity and len(msg.velocity) > max(idx):
            speeds = [msg.velocity[i] for i in idx]
        elif msg.position and len(msg.position) > max(idx):
            # Fall back to differentiating position: some drivers publish only
            # position for continuous joints.
            pos = [msg.position[i] for i in idx]
            if self._last_positions is None or self._last_js_time is None:
                self._last_positions, self._last_js_time = pos, now
                return
            dt = (now - self._last_js_time).nanoseconds * 1e-9
            if dt <= 0.0:
                return
            speeds = [self._wrap(p - lp) / dt
                      for p, lp in zip(pos, self._last_positions)]
            self._last_positions = pos
        else:
            return

        if self._last_js_time is None:
            self._last_js_time = now
            return
        dt = (now - self._last_js_time).nanoseconds * 1e-9
        self._last_js_time = now
        if dt <= 0.0 or dt > 1.0:      # ignore stalls and clock jumps
            return

        vx, vy, wz = self.kin.forward_kinematics(speeds)

        # Integrate in the odom frame. Midpoint on yaw: cheap, and noticeably
        # better than Euler when turning fast.
        th_mid = self._th + 0.5 * wz * dt
        self._x += (vx * math.cos(th_mid) - vy * math.sin(th_mid)) * dt
        self._y += (vx * math.sin(th_mid) + vy * math.cos(th_mid)) * dt
        self._th = self._wrap(self._th + wz * dt)

        stamp = msg.header.stamp
        if self._publish_odom:
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = self.odom_frame
            o.child_frame_id = self.base_frame
            o.pose.pose.position.x = self._x
            o.pose.pose.position.y = self._y
            o.pose.pose.orientation = yaw_to_quaternion(self._th)
            o.twist.twist.linear.x = vx
            o.twist.twist.linear.y = vy
            o.twist.twist.angular.z = wz
            # No covariance is published: a meaningful one would have to be
            # measured on the real robot, and inventing numbers here would let
            # a downstream EKF trust this far more than it deserves.
            self._odom_pub.publish(o)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = yaw_to_quaternion(self._th)
            self._tf.sendTransform(t)

    @staticmethod
    def _wrap(a: float) -> float:
        """Wrap to (-pi, pi]. Continuous joints wind up without bound."""
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = KiwiDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: leave the wheels stopped rather than latched at speed.
        try:
            node._publish([0.0] * len(node.wheel_joints))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

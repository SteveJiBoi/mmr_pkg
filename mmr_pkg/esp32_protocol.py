"""The ESP32 omni firmware's wire protocol, and the ROS -> PWM conversion.

*** THIS IS THE FILE TO DIFF AGAINST esp32/controller.py. ***

Deliberately free of ROS imports -- exactly like kiwi_kinematics.py, and for
the same reason: it can then be read, tested and compared against the
microcontroller on a machine with no ROS install. All the ROS plumbing lives in
esp32_bridge.py; all the arithmetic lives here.

The firmware is esp32/MotionTestOriginal/MotionTestOriginal.ino. It is the
fixed point. esp32/controller.py is the known-good pygame client. PacketEncoder
below is a second client that must be indistinguishable from that one --
test/test_esp32_bridge.py enforces that byte for byte.


THE PROTOCOL
------------
Datagram, ASCII, UDP port 1234, no framing, no sequence number, no checksum:

    "A,<angle>,<speed>,<rot>"    angle 0..359 deg, speed 0..255, rot -255..255
    "PING"                       -> firmware replies "PONG"

The firmware replies "ACK <angle> <speed> <rot>" to every accepted command and
"ERR" to anything it cannot parse. Replies are advisory: this is UDP, so a
missing ACK means "packet or reply was lost", not necessarily "robot stopped".

It parses with sscanf("%c,%d,%d,%d") into a 64-byte buffer, so a malformed or
over-long line is silently dropped as "ERR". Never emit a float or an exponent.

DEADMAN: the firmware brakes all three motors if no packet arrives within
DEADMAN_MS (400 ms). Any client MUST keep transmitting while the robot is meant
to move -- on a timer, not on message arrival.


UNITS: READ THIS BEFORE YOU TUNE ANYTHING
-----------------------------------------
The firmware has no encoders, no wheel radius and no closed loop. `speed` and
`rot` are raw 8-bit PWM duty, which is roughly proportional to motor VOLTAGE,
which is only loosely and non-linearly related to ground speed -- it varies
with battery charge, payload, floor friction and the motor dead zone.

There is therefore NO measured constant converting m/s to PWM, and this file
does not invent one. Instead the mapping is an explicit, declared
normalisation:

    max_linear_speed  (m/s)   maps to  max_linear_pwm  (duty)
    max_angular_speed (rad/s) maps to  max_angular_pwm (duty)

The PWM ends of those two are NOT arbitrary: 180 and 80 are the values
esp32/controller.py has been driven with, so they are known to work and known
not to brown out. The m/s and rad/s ends ARE arbitrary -- 1.0 is a placeholder,
not a measurement. Until the robot is measured, treat cmd_vel here as a
fraction of full scale rather than a physical speed.

To turn it into a real number: clear a few metres of floor, mark a start and an
end line, drive between them at a constant cmd_vel with a stopwatch, and set
max_linear_speed = distance / (time * commanded_fraction). Do the same with a
360 degree spin for max_angular_speed. See README "Calibrating the ESP32
bridge". Until then, odometry from this robot is not a thing that exists.


FRAME AND SIGN CONVENTION
-------------------------
ROS (REP-103, and what teleop_twist_keyboard emits):

    linear.x = forward    linear.y = LEFT    angular.z = CCW seen from above

esp32/controller.py builds its angle from a screen-style operator frame:

    x_op = right (D key)   y_op = forward (W key)
    angle = degrees(atan2(y_op, x_op))

so the conversion is x_op = -linear.y, y_op = +linear.x, giving

    angle = degrees(atan2(linear.x, -linear.y))  mod 360

and for yaw, controller.py sends rot = -80 for Q -- which is the left/CCW turn,
i.e. POSITIVE angular.z -- so rot carries the OPPOSITE sign to angular.z.

Both signs are constructor arguments (strafe_sign, yaw_sign) because they
depend on motor wiring that is not knowable from the source code. If the robot
strafes the wrong way, flip strafe_sign; do not "fix" it by editing the angle
formula, or this file and the firmware will disagree in a way that is very hard
to see later.

*** The firmware's wheel mixing does NOT agree with kiwi_kinematics.py about
*** which way is left. They agree exactly on forward/back and are exactly
*** negated on strafe -- a reflection, which no relabelling of the robot frame
*** can undo. Run `python tools/firmware_diff.py` for the table and the single
*** wiring hypothesis that reconciles them. This encoder sidesteps the question
*** by reproducing controller.py verbatim, so the robot drives today exactly as
*** it already does. The discrepancy is still real, and it matters the moment
*** you compare simulation against hardware.
"""
from __future__ import annotations

import math

# controller.py's own values: its cruise `speed` and its Q/E `rotation`. Known
# to drive this robot without browning out, which is why they are the defaults.
DEFAULT_LINEAR_PWM = 180
DEFAULT_ANGULAR_PWM = 80

# Firmware constants, mirrored here so clients can reason about them. Keep in
# step with the sketch; tools/firmware_diff.py reads the real values from it.
FIRMWARE_UDP_PORT = 1234
FIRMWARE_DEADMAN_MS = 400
FIRMWARE_MAX_PWM = 255


class PacketEncoder:
    """(vx, vy, wz) in ROS units -> the firmware's (angle, speed, rot)."""

    def __init__(self, max_linear_speed=1.0, max_angular_speed=1.0,
                 max_linear_pwm=DEFAULT_LINEAR_PWM,
                 max_angular_pwm=DEFAULT_ANGULAR_PWM,
                 strafe_sign=1, yaw_sign=-1, prevent_clipping=False):
        if max_linear_speed <= 0.0 or max_angular_speed <= 0.0:
            raise ValueError("max_linear_speed and max_angular_speed must be "
                             "> 0; they are the denominators of the PWM map")
        self.max_v = float(max_linear_speed)
        self.max_w = float(max_angular_speed)
        self.max_v_pwm = int(max_linear_pwm)
        self.max_w_pwm = int(max_angular_pwm)
        self.strafe_sign = 1 if strafe_sign >= 0 else -1
        self.yaw_sign = 1 if yaw_sign >= 0 else -1
        self.prevent_clipping = bool(prevent_clipping)

    def encode(self, vx: float, vy: float, wz: float):
        """Returns (angle_deg, speed_pwm, rot_pwm), all ints in firmware range."""
        # Operator frame: x = right, y = forward. ROS linear.y is LEFT, hence
        # the negation. strafe_sign exists to undo it for mirrored wiring.
        x_op = -vy * self.strafe_sign
        y_op = vx

        mag = math.hypot(x_op, y_op)
        if mag < 1e-9:
            angle = 0
            speed = 0
        else:
            # int() truncation and the mod, both matching controller.py.
            angle = int(math.degrees(math.atan2(y_op, x_op))) % 360
            speed = int(round(self.max_v_pwm * mag / self.max_v))
            speed = max(0, min(FIRMWARE_MAX_PWM, speed))

        rot = int(round(self.yaw_sign * self.max_w_pwm * wz / self.max_w))
        rot = max(-FIRMWARE_MAX_PWM, min(FIRMWARE_MAX_PWM, rot))

        if self.prevent_clipping and speed + abs(rot) > FIRMWARE_MAX_PWM:
            # The firmware computes power_i = speed*cos(...) - rot and clamps
            # PER WHEEL, so the worst-case wheel sees speed + |rot|. Above 255
            # it clips and the robot curves away from the commanded heading
            # instead of merely going slower. Scaling both terms by one factor
            # keeps the translation:rotation ratio, and therefore the path.
            s = float(FIRMWARE_MAX_PWM) / (speed + abs(rot))
            speed = int(speed * s)
            rot = int(rot * s)

        return angle, speed, rot

    def packet(self, vx: float, vy: float, wz: float) -> bytes:
        angle, speed, rot = self.encode(vx, vy, wz)
        return f"A,{angle},{speed},{rot}".encode("ascii")

    def __repr__(self):
        return (f"PacketEncoder({self.max_v:g} m/s -> pwm {self.max_v_pwm}, "
                f"{self.max_w:g} rad/s -> pwm {self.max_w_pwm}, "
                f"strafe_sign={self.strafe_sign}, yaw_sign={self.yaw_sign})")

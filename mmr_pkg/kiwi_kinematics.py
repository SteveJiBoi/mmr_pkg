"""Kiwi (3-wheel omni) kinematics for mmr_bot.

*** THIS IS THE FILE TO DIFF AGAINST THE ESP32 FIRMWARE. ***

It is deliberately free of ROS imports and, in the inverse direction, free of
numpy too: `inverse_kinematics` below is a plain loop over three wheels using
only `sin`/`cos`, so it can be compared line-by-line against C on the
microcontroller. Everything ROS-facing lives in kiwi_drive_node.py.

If the firmware and this file disagree, ONE of them is wrong about the physical
robot. Fix that one. Do not "fix" both to meet in the middle.


Geometry and sign convention
----------------------------
Derived in tools/kiwi_check.py from the generated URDF, not asserted here.
Re-run that script after changing any geometry; it re-reads build/robot.urdf
and re-derives everything below from the joint origins and axes.

Wheel i is mounted at bearing a_i, measured CCW from robot +X (forward), at
distance L from the centre. Its spin axis points radially OUTWARD:

    p_i = L (cos a_i, sin a_i, 0)          wheel centre
    u_i =   (cos a_i, sin a_i, 0)          spin axis, radially outward
    contact patch at -R z^   (directly below the axle)

A material point of the wheel at the contact patch moves at

    v_mat = w u_i x (-R z^) = w R (-sin a_i, cos a_i, 0)

Rolling without slip means that point is stationary against the ground, so a
POSITIVE w drives the chassis along

    d_i = -v_mat / (w R) = (sin a_i, -cos a_i, 0)      i.e. bearing a_i - 90 deg

Consequences, all verified numerically in tools/kiwi_check.py:

  * +1 rad/s on ONE wheel pushes the robot toward that wheel's mounting angle
    minus 90 degrees.
  * +1 rad/s on ALL THREE gives pure rotation with wz = -R/L, i.e. CLOCKWISE
    seen from above. All-positive is NEGATIVE yaw. This is the single easiest
    thing to get backwards, so it is called out here and in the README.

MOUNTING ANGLES: this robot measures 60 / 180 / 300 degrees, NOT the 30 / 150 /
270 quoted in the original brief. The two differ by a uniform 30 degrees. The
STEP was measured directly (see README, "The 30 degree wheel-angle
discrepancy") and forward is pinned by the lidar sitting exactly on CAD -Y.
The angles are a constructor argument so the firmware's convention can be
matched without editing code.


Units
-----
Everything is SI and radians: metres, m/s, rad, rad/s. Angles are stored in
radians internally; the constructor takes degrees only because that is how the
URDF property reads.
"""
from __future__ import annotations

import math

# Defaults measured from the STEP; see README "Derived geometry".
# base.xacro is the authority for these -- kiwi_drive_node passes them in as
# ROS parameters so there is exactly one place to change them.
DEFAULT_WHEEL_ANGLES_DEG = (60.0, 180.0, 300.0)
DEFAULT_WHEEL_RADIUS = 0.030      # m, measured max radius of RodaOmni 60 mm
DEFAULT_BASE_RADIUS = 0.135500    # m, triad centroid to wheel centre


class KiwiKinematics:
    """Body twist <-> wheel angular velocities for an N-wheel omni base.

    Written for N=3 but the maths is general; the pseudo-inverse used by
    `forward_kinematics` is built for whatever angles it is given.
    """

    def __init__(self,
                 wheel_angles_deg=DEFAULT_WHEEL_ANGLES_DEG,
                 wheel_radius: float = DEFAULT_WHEEL_RADIUS,
                 base_radius: float = DEFAULT_BASE_RADIUS):
        if len(wheel_angles_deg) < 3:
            raise ValueError(
                f"need at least 3 wheels to span a planar twist, got "
                f"{len(wheel_angles_deg)}")
        if wheel_radius <= 0.0:
            raise ValueError(f"wheel_radius must be > 0, got {wheel_radius}")
        if base_radius <= 0.0:
            raise ValueError(f"base_radius must be > 0, got {base_radius}")

        self.angles = [math.radians(a) for a in wheel_angles_deg]
        self.R = float(wheel_radius)
        self.L = float(base_radius)
        self.n = len(self.angles)

        # Precompute the sin/cos per wheel. The IK loop below then contains no
        # trigonometry at all, which is what you want on the ESP32 too.
        self._sin = [math.sin(a) for a in self.angles]
        self._cos = [math.cos(a) for a in self.angles]

        self._pinv = None   # built lazily by forward_kinematics (needs numpy)

    # ------------------------------------------------------------------ IK
    def inverse_kinematics(self, vx: float, vy: float, wz: float):
        """Body twist -> wheel angular velocities, rad/s.

        *** THE FUNCTION TO DIFF AGAINST THE FIRMWARE. ***

            w_i = ( vx*sin(a_i) - vy*cos(a_i) - L*wz ) / R

        vx  forward, m/s        (robot +X, the direction the lidar faces)
        vy  left,    m/s        (robot +Y)
        wz  yaw rate, rad/s     (CCW positive, the ROS convention)

        Returns a list of angular velocities in rad/s, one per wheel, in
        wheel_0, wheel_1, wheel_2 order.

        Derivation: the contact point of wheel i must move with the body, at
        v_i = v + wz x p_i. Only the component along the wheel's rolling
        direction d_i = (sin a_i, -cos a_i) can be produced by spinning it; the
        perpendicular component is absorbed by the free rollers. Setting
        v_i . d_i = w_i R and expanding wz x p_i = wz*L*(-sin a_i, cos a_i)
        gives the -L*wz term. R and L are the only lengths involved.
        """
        R_inv = 1.0 / self.R
        L = self.L
        return [(vx * self._sin[i] - vy * self._cos[i] - L * wz) * R_inv
                for i in range(self.n)]

    # ------------------------------------------------------------------ FK
    def forward_kinematics(self, wheel_speeds):
        """Wheel angular velocities (rad/s) -> body twist (vx, vy, wz).

        The exact inverse when the wheels are consistent, and the least-squares
        best fit when they are not (which is the normal case on a real robot,
        where slip makes the three readings over-determined and contradictory).

        Uses numpy for the pseudo-inverse, built once on first call. For three
        EQUALLY SPACED wheels this reduces to the closed form

            vx =  (2R/3) * sum( w_i sin a_i )
            vy = -(2R/3) * sum( w_i cos a_i )
            wz = -(R/3L) * sum( w_i )

        which test_kiwi_kinematics.py checks against the general solution. The
        general path is kept because the mounting angles are a parameter and
        must not silently assume 120 degree spacing.
        """
        if len(wheel_speeds) != self.n:
            raise ValueError(
                f"expected {self.n} wheel speeds, got {len(wheel_speeds)}")
        if self._pinv is None:
            self._pinv = self._build_pinv()
        import numpy as np
        return tuple(float(v) for v in self._pinv @ np.asarray(
            wheel_speeds, dtype=float))

    def _build_pinv(self):
        import numpy as np
        M = np.array([[self._sin[i] / self.R,
                       -self._cos[i] / self.R,
                       -self.L / self.R] for i in range(self.n)])
        return np.linalg.pinv(M)

    # -------------------------------------------------------------- limits
    def scale_to_limit(self, vx: float, vy: float, wz: float,
                       max_wheel_speed: float):
        """Shrink a twist until every wheel is within max_wheel_speed.

        Returns (vx, vy, wz, scale). scale is 1.0 when nothing was clipped.

        This scales the WHOLE twist by one factor rather than clamping each
        wheel independently. Per-wheel clamping silently changes the direction
        of travel -- the robot curves away from the commanded heading instead
        of simply going slower. Uniform scaling keeps the direction exact and
        only loses speed, which is almost always what you want. The IK is
        linear in the twist, so one factor is sufficient and exact.

        max_wheel_speed <= 0 disables limiting.
        """
        if max_wheel_speed <= 0.0:
            return vx, vy, wz, 1.0
        peak = max(abs(w) for w in self.inverse_kinematics(vx, vy, wz))
        if peak <= max_wheel_speed:
            return vx, vy, wz, 1.0
        s = max_wheel_speed / peak
        return vx * s, vy * s, wz * s, s

    # ----------------------------------------------------------------- misc
    def max_body_speed(self, max_wheel_speed: float) -> float:
        """Fastest straight-line speed, m/s, at a given wheel speed limit.

        Direction-dependent for a kiwi base; this returns the WORST case over
        all headings, so it is a speed the robot can achieve in any direction.
        """
        worst = 0.0
        for k in range(360):
            th = math.radians(k)
            peak = max(abs(w) for w in self.inverse_kinematics(
                math.cos(th), math.sin(th), 0.0))
            worst = max(worst, peak)
        return max_wheel_speed / worst if worst > 0 else 0.0

    def __repr__(self):
        deg = [round(math.degrees(a), 3) for a in self.angles]
        return (f"KiwiKinematics(angles_deg={deg}, R={self.R}, L={self.L})")

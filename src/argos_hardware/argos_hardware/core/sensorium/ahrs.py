"""
Madgwick AHRS filter for ARGOS.

Fuses MPU-6050 (gyro + accel) with Flotilla LSM303D (magnetometer)
to produce stable roll/pitch/yaw with absolute compass heading.

9DOF mode: gyro + accel + mag → drift-free heading
6DOF fallback: gyro + accel only → heading drifts over time

Algorithm: Madgwick (2010) gradient-descent orientation filter.
Reference: "An efficient orientation filter for inertial and
inertial/magnetic sensor arrays", Sebastian Madgwick.

Usage::

    from argos.sensorium.ahrs import MadgwickAHRS

    ahrs = MadgwickAHRS(beta=0.05)

    # In a loop at ~50 Hz:
    ahrs.update(
        gyro=(gx, gy, gz),     # rad/s
        accel=(ax, ay, az),    # g (normalized internally)
        mag=(mx, my, mz),      # any units (normalized internally)
        dt=0.02,               # seconds since last update
    )

    print(ahrs.roll, ahrs.pitch, ahrs.yaw)   # degrees

    # Without magnetometer (6DOF, yaw drifts):
    ahrs.update(gyro=(...), accel=(...), dt=0.02)

Axis convention (Euler output):
    roll   — rotation about X, right-side-down positive  (-180..180)
    pitch  — rotation about Y, nose-up positive          (-90..90)
    yaw    — rotation about Z, clockwise from mag north  (0..360)

Beta tuning:
    Higher beta → faster convergence, more accelerometer/mag noise.
    Lower beta → smoother, slower to correct drift.
    Typical: 0.01 (very smooth) to 0.1 (aggressive correction).
    Default 0.05 is a reasonable starting point.
"""

import math
import time
from dataclasses import dataclass, field


@dataclass
class Orientation:
    """AHRS output: orientation as quaternion and Euler angles."""

    q_w: float = 1.0
    q_x: float = 0.0
    q_y: float = 0.0
    q_z: float = 0.0
    timestamp: float = field(default_factory=time.monotonic)

    @property
    def roll_deg(self) -> float:
        """Roll in degrees (-180 to +180)."""
        w, x, y, z = self.q_w, self.q_x, self.q_y, self.q_z
        return math.degrees(
            math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        )

    @property
    def pitch_deg(self) -> float:
        """Pitch in degrees (-90 to +90)."""
        w, x, y, z = self.q_w, self.q_x, self.q_y, self.q_z
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        return math.degrees(math.asin(sinp))

    @property
    def yaw_deg(self) -> float:
        """Yaw in degrees (0 to 360, clockwise from magnetic north)."""
        w, x, y, z = self.q_w, self.q_x, self.q_y, self.q_z
        raw = math.degrees(
            math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        )
        return raw % 360.0


class MadgwickAHRS:
    """
    Madgwick gradient-descent AHRS filter.

    Maintains an internal quaternion estimate of orientation, updated
    each time update() is called with new sensor data.

    Parameters
    ----------
    beta : float
        Filter gain.  Controls how aggressively the accelerometer and
        magnetometer correct gyro drift.  See module docstring for tuning.
    """

    def __init__(self, beta: float = 0.05):
        self._beta = beta
        self._q = [1.0, 0.0, 0.0, 0.0]   # [w, x, y, z]
        self._roll_offset  = 0.0           # degrees — mounting bias zeroed at startup
        self._pitch_offset = 0.0           # degrees

    @property
    def beta(self) -> float:
        return self._beta

    @beta.setter
    def beta(self, value: float):
        self._beta = value

    def reset(self):
        """Reset orientation to identity (level, heading=0) and clear level reference."""
        self._q = [1.0, 0.0, 0.0, 0.0]
        self._roll_offset  = 0.0
        self._pitch_offset = 0.0

    def init_from_accel(self, ax: float, ay: float, az: float):
        """Initialise the quaternion from a gravity vector for instant convergence.

        Instead of starting from identity [1,0,0,0] and waiting ~5 s for the
        filter to find the correct tilt, compute roll and pitch directly from
        the accelerometer and set the quaternion to match.  Yaw is left at 0
        (unknown without a magnetometer).

        Call once after gyro calibration, before the main loop.
        """
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-10:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # Tilt from gravity — matches Madgwick filter's ZYX Euler convention.
        # ax = -sin(pitch), ay = cos(pitch)*sin(roll), az = cos(pitch)*cos(roll)
        roll  = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        cr = math.cos(roll  / 2.0)
        sr = math.sin(roll  / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)

        # Quaternion for ZYX Euler (yaw=0): q = q_pitch * q_roll
        self._q = [cp * cr, cp * sr, sp * cr, -sp * sr]

    def calibrate_level(self):
        """Record the current orientation as the robot's level reference.

        After calling this with the robot sitting flat and still, roll and
        pitch will read 0 regardless of how the IMU chip is physically
        mounted on the chassis.  Call after init_from_accel() and a brief
        warm-up so the filter has settled.

        Returns the detected mounting offsets (roll_deg, pitch_deg) for
        logging.
        """
        self._roll_offset  = self.orientation.roll_deg
        self._pitch_offset = self.orientation.pitch_deg
        return self._roll_offset, self._pitch_offset

    def update(self, gyro, accel, mag=None, dt: float = 0.02):
        """
        Run one filter iteration.

        Parameters
        ----------
        gyro : tuple of 3 floats
            Angular velocity (gx, gy, gz) in **rad/s**.
        accel : tuple of 3 floats
            Acceleration (ax, ay, az) in any consistent units —
            normalized internally.
        mag : tuple of 3 floats, or None
            Magnetic field (mx, my, mz) in any units — normalized
            internally.  Pass None for 6DOF (heading will drift).
        dt : float
            Time step in seconds since last update.
        """
        if mag is not None:
            mx, my, mz = mag
            if not (mx == 0.0 and my == 0.0 and mz == 0.0):
                self._update_marg(gyro, accel, mag, dt)
                return
        self._update_imu(gyro, accel, dt)

    # --- orientation output ---------------------------------------------------

    @property
    def orientation(self) -> Orientation:
        """Current orientation as an Orientation dataclass."""
        return Orientation(
            q_w=self._q[0], q_x=self._q[1],
            q_y=self._q[2], q_z=self._q[3],
        )

    @property
    def quaternion(self):
        """Current quaternion as (w, x, y, z) tuple."""
        return tuple(self._q)

    @property
    def roll(self) -> float:
        """Roll in degrees (-180 to +180), relative to the level reference."""
        return self.orientation.roll_deg - self._roll_offset

    @property
    def pitch(self) -> float:
        """Pitch in degrees (-90 to +90), relative to the level reference."""
        return self.orientation.pitch_deg - self._pitch_offset

    @property
    def yaw(self) -> float:
        """Yaw in degrees (0 to 360)."""
        return self.orientation.yaw_deg

    # --- 6DOF update (gyro + accel) -------------------------------------------

    def _update_imu(self, gyro, accel, dt):
        q0, q1, q2, q3 = self._q
        gx, gy, gz = gyro
        ax, ay, az = accel

        # Normalize accelerometer
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-10:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # Auxiliary variables
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3

        # Gradient descent corrective step
        s0 = (_4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay)
        s1 = (_4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay
              - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az)
        s2 = (4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
              - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az)
        s3 = (4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay)

        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm < 1e-10:
            return
        s0 /= norm
        s1 /= norm
        s2 /= norm
        s3 /= norm

        # Rate of change of quaternion (gyro integration - beta * gradient)
        q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self._beta * s0
        q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self._beta * s1
        q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self._beta * s2
        q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self._beta * s3

        # Integrate
        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        # Normalize quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self._q = [q0 / norm, q1 / norm, q2 / norm, q3 / norm]

    # --- 9DOF update (gyro + accel + mag) -------------------------------------

    def _update_marg(self, gyro, accel, mag, dt):
        q0, q1, q2, q3 = self._q
        gx, gy, gz = gyro
        ax, ay, az = accel
        mx, my, mz = mag

        # Normalize accelerometer
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-10:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # Normalize magnetometer
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm < 1e-10:
            self._update_imu(gyro, accel, dt)
            return
        mx /= norm
        my /= norm
        mz /= norm

        # Auxiliary variables
        _2q0mx = 2.0 * q0 * mx
        _2q0my = 2.0 * q0 * my
        _2q0mz = 2.0 * q0 * mz
        _2q1mx = 2.0 * q1 * mx
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q0q2 = 2.0 * q0 * q2
        _2q2q3 = 2.0 * q2 * q3
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        # Reference direction of Earth's magnetic field
        hx = (mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
              + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3)
        hy = (_2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
              - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3)
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = (-_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
                - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3)
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient descent corrective step (6 objective functions)
        s0 = (-_2q2 * (2.0 * q1q3 - _2q0q2 - ax)
              + _2q1 * (2.0 * q0q1 + _2q2q3 - ay)
              - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3)
                             + _2bz * (q1q3 - q0q2) - mx)
              + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3)
                                             + _2bz * (q0q1 + q2q3) - my)
              + _2bx * q2 * (_2bx * (q0q2 + q1q3)
                             + _2bz * (0.5 - q1q1 - q2q2) - mz))

        s1 = (_2q3 * (2.0 * q1q3 - _2q0q2 - ax)
              + _2q0 * (2.0 * q0q1 + _2q2q3 - ay)
              - 4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
              + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3)
                             + _2bz * (q1q3 - q0q2) - mx)
              + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3)
                                            + _2bz * (q0q1 + q2q3) - my)
              + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3)
                                            + _2bz * (0.5 - q1q1 - q2q2) - mz))

        s2 = (-_2q0 * (2.0 * q1q3 - _2q0q2 - ax)
              + _2q3 * (2.0 * q0q1 + _2q2q3 - ay)
              - 4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
              + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3)
                                             + _2bz * (q1q3 - q0q2) - mx)
              + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3)
                                            + _2bz * (q0q1 + q2q3) - my)
              + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3)
                                            + _2bz * (0.5 - q1q1 - q2q2) - mz))

        s3 = (_2q1 * (2.0 * q1q3 - _2q0q2 - ax)
              + _2q2 * (2.0 * q0q1 + _2q2q3 - ay)
              + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3)
                                             + _2bz * (q1q3 - q0q2) - mx)
              + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3)
                                             + _2bz * (q0q1 + q2q3) - my)
              + _2bx * q1 * (_2bx * (q0q2 + q1q3)
                             + _2bz * (0.5 - q1q1 - q2q2) - mz))

        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm < 1e-10:
            return
        s0 /= norm
        s1 /= norm
        s2 /= norm
        s3 /= norm

        # Rate of change of quaternion (gyro integration - beta * gradient)
        q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self._beta * s0
        q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self._beta * s1
        q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self._beta * s2
        q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self._beta * s3

        # Integrate
        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        # Normalize quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self._q = [q0 / norm, q1 / norm, q2 / norm, q3 / norm]

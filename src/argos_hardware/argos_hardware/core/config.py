"""
Central hardware configuration for ARGOS.

All hardware mappings live here — motors, sensors, I2C addresses, and
Flotilla dock assignments.  Import from this module rather than
hardcoding anything in drivers, controllers, or test scripts.
"""

from dataclasses import dataclass


@dataclass
class MotorConfig:
    motor_id: int
    positive: str        # physical effect of +ve speed
    negative: str        # physical effect of -ve speed
    max_speed: int = 100 # hard upper cap (0–100); motor never exceeds this
    min_speed: int = 0   # minimum effective speed; non-zero requests clamped up to this
    max_duration: float = 30.0  # seconds before watchdog auto-stops the motor


# ---------------------------------------------------------------------------
# Tracks — Waveshare Motor Driver HAT (I2C 0x40, bus 1)
# ---------------------------------------------------------------------------
TRACK_MOTORS = {
    "left":  MotorConfig(0, "forward",  "backward", max_speed=100, min_speed=30, max_duration=10.0),
    "right": MotorConfig(1, "forward",  "backward", max_speed=100, min_speed=30, max_duration=10.0),
}

# ---------------------------------------------------------------------------
# Arm joints — SB Components MotorShield (GPIO BOARD pins)
# Wiring confirmed on hardware 2026-02-24.
# ---------------------------------------------------------------------------
ARM_JOINTS = {
    "shoulder": MotorConfig(4, "raise",   "lower",   max_speed=80, min_speed=25, max_duration=3.0),
    "elbow":    MotorConfig(2, "extend",  "retract", max_speed=80, min_speed=25, max_duration=3.0),
    "wrist":    MotorConfig(3, "raise",   "lower",   max_speed=80, min_speed=25, max_duration=3.0),
    "gripper":  MotorConfig(1, "open",    "close",   max_speed=80, min_speed=20, max_duration=2.0),
}

# ---------------------------------------------------------------------------
# Sensorium — I2C addresses and Flotilla dock channel assignments
# ---------------------------------------------------------------------------

# MPU-6050 inertial measurement unit (gyro + accel)
IMU_I2C_BUS     = 1       # Raspberry Pi I2C bus
IMU_I2C_ADDR    = 0x68    # default MPU-6050 address (AD0 low)

# Flotilla Motion module dock ports
# ch 6 — body-mounted LSM303D (accel + mag); used for AHRS heading
# ch 1 — shoulder-to-elbow arm link LSM303D; used for joint angle estimation
FLOTILLA_BODY_MOTION_CH = 6
FLOTILLA_ARM_MOTION_CH  = 1

# HC-SR04 ultrasonic (BOARD pin numbers; voltage divider fitted on CN10)
SONAR_TRIG_PIN  = 29
SONAR_ECHO_PIN  = 31

# IR proximity sensors (BOARD pin numbers)
IR_PIN_1        = 7    # CN9
IR_PIN_2        = 12   # CN8

# ---------------------------------------------------------------------------
# Sensor axis remapping — chip frame → AHRS filter frame
# ---------------------------------------------------------------------------
# Madgwick filter convention (see argos/sensorium/ahrs.py):
#   az = +1g when robot is flat;  ax > 0 = nose pitches up;  ay > 0 = right rolls down
#   i.e.  filter X = forward,  filter Y = right,  filter Z = up
#
# Remap format: ((sign, src), (sign, src), (sign, src))  for (filter_x, filter_y, filter_z)
#   src  0 = chip_x,  1 = chip_y,  2 = chip_z
#   Apply as: filter[i] = sign * chip[src]
#   Applies identically to accel, gyro, and (for body LSM) mag — same chip axes.
#
# Probe results 2026-02-27 with probe_sensor_axes.py:
#
#   Test 1 — robot flat on surface (gravity = DOWN = robot −Z):
#     MPU-6050     accel: ax = −0.967g  →  chip +X = DOWN        ★ measured
#     Body LSM303D accel: ax = −0.999g  →  chip +X = DOWN        ★ measured
#     Arm  LSM303D accel: ax = +0.966g  →  chip +X = UP          ★ measured
#
#   Test 2 — robot tilted nose-down ~90° (gravity = FORWARD = robot +X):
#     MPU-6050     accel: az = −1.100g  →  chip +Z = FORWARD     ★ measured
#     Body LSM303D accel: ay = −1.009g  →  chip +Y = FORWARD     ★ measured
#     Arm  LSM303D accel: ay = +0.976g  →  chip −Y = FORWARD     ★ measured
#
#   Test 3 — robot tilted 90° right side down (gravity = RIGHT = robot +Y):
#     MPU-6050     accel: ay = +1.000g  →  chip +Y = RIGHT     ★ measured (confirms RHR)
#     Body LSM303D accel: az = −0.895g  →  chip +Z = RIGHT     ★ measured (contradicts RHR)
#     Arm  LSM303D accel: az = −0.914g  →  chip +Z = RIGHT     ★ measured (contradicts RHR)
#
#   Note: both Flotilla LSM303D chips appear to use a LEFT-HANDED axis convention
#   (Z = −(X × Y)).  All three axes are now fully measured — no inferred values.

# MPU-6050: chip X=down, Y=right, Z=forward  →  filter X=+chipZ, Y=+chipY, Z=−chipX
# Same remap applies to MPU gyro channels.
IMU_AXIS_REMAP = (
    (+1,  2),   # filter X (forward) = +chip_z   ★ nose-down test
    (+1,  1),   # filter Y (right)   = +chip_y   ★ right-tilt test
    (-1,  0),   # filter Z (up)      = −chip_x   ★ flat test
)

# Body LSM303D (ch6): chip X=down, Y=forward, Z=right  →  filter X=+chipY, Y=+chipZ, Z=−chipX
# Same remap applies to LSM303D magnetometer (shared chip axes).
BODY_MOTION_AXIS_REMAP = (
    (+1,  1),   # filter X (forward) = +chip_y   ★ nose-down test
    (+1,  2),   # filter Y (right)   = +chip_z   ★ right-tilt test
    (-1,  0),   # filter Z (up)      = −chip_x   ★ flat test
)

# Magnetometer uses the same LSM303D chip axes as the accelerometer above.
#
# Compass spin test 2026-02-28 (robot flat, turned CW in 90° steps):
#   Raw chip readings:
#     0°  : mx=+4202  my=+4672  mz= +8880
#     90° : mx=+3640  my=+2223  mz= +8308
#     180°: mx=+3739  my=+1581  mz=+10995
#
#   heading = atan2(−mz_cal, my_cal) increases clockwise ✓  (remap confirmed)
#   mx (filter Z = −mx) varies only ~500 counts across yaw — vertical field stable ✓
#
#   Hard-iron bias is large (mz offset ≈ 5× signal amplitude — motor magnets nearby).
#   Rough offsets from this half-rotation:  my_bias ≈ +3127,  mz_bias ≈ +9938.
#   Run a full 360° slow spin to finalise — update MAG_HARD_IRON_BIAS below.
BODY_MOTION_MAG_REMAP = BODY_MOTION_AXIS_REMAP

# Hard-iron bias for body LSM303D magnetometer.
# Subtract from raw chip (mx, my, mz) before applying BODY_MOTION_MAG_REMAP.
# Derived from: bias = (axis_max + axis_min) / 2  over a full level 360° spin.
# Rough values from 2026-02-28 half-rotation — replace after full spin calibration.
MAG_HARD_IRON_BIAS = (
    0,      # mx bias  (filter Z = up;  vertical component — small effect on heading)
    3127,   # my bias  (filter X = forward)   rough, half-rotation only
    9938,   # mz bias  (filter Y = right)      rough, half-rotation only
)

# Arm LSM303D (ch1): chip X=up, −Y=forward, Z=right  →  filter X=−chipY, Y=+chipZ, Z=+chipX
ARM_MOTION_AXIS_REMAP = (
    (-1,  1),   # filter X (forward) = −chip_y   ★ nose-down test
    (+1,  2),   # filter Y (right)   = +chip_z   ★ right-tilt test
    (+1,  0),   # filter Z (up)      = +chip_x   ★ flat test
)

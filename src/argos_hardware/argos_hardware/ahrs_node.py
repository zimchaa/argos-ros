"""
AHRS node — fuses IMU + Flotilla data via Madgwick filter.

Subscribes:
  /imu/raw    (sensor_msgs/Imu)      — gyro + accel from MPU-6050
  /flotilla   (argos_msgs/FlotillaData) — magnetometer from body LSM303D

Publishes:
  /imu/data   (sensor_msgs/Imu)      — with orientation quaternion
  /ahrs       (argos_msgs/AhrsData)  — roll/pitch/yaw/heading

The filter runs on a timer at publish_rate Hz using the latest received
values from each topic. Axis remaps from config.py are applied before
passing data to MadgwickAHRS.

Parameters:
  publish_rate: float  (default 50.0 Hz)
  beta:         float  (default 0.05  — Madgwick filter gain)
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from argos_msgs.msg import FlotillaData, AhrsData

from argos_hardware.core.sensorium.ahrs import MadgwickAHRS
from argos_hardware.core.config import (
    IMU_AXIS_REMAP, BODY_MOTION_AXIS_REMAP, BODY_MOTION_MAG_REMAP, MAG_HARD_IRON_BIAS
)

_G = 9.80665


def _apply_remap(vec, remap):
    """Apply axis remap: remap = ((sign, src), ...) for each output axis."""
    return tuple(sign * vec[src] for sign, src in remap)


class AhrsNode(Node):

    def __init__(self):
        super().__init__('ahrs_node')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('beta', 0.05)
        rate = self.get_parameter('publish_rate').value
        beta = self.get_parameter('beta').value

        self._filter = MadgwickAHRS(beta=beta)
        self._dt     = 1.0 / rate   # fallback for first iteration

        self._latest_imu      = None
        self._latest_flotilla = None
        self._last_update_time = None   # monotonic clock for real dt
        self._orientation_initialized = False

        self.create_subscription(Imu, '/imu/raw', self._imu_cb, 10)
        self.create_subscription(FlotillaData, '/flotilla', self._flotilla_cb, 10)

        self._imu_pub  = self.create_publisher(Imu, '/imu/data', 10)
        self._ahrs_pub = self.create_publisher(AhrsData, '/ahrs', 10)

        self.create_timer(self._dt, self._update)
        self.get_logger().info(f'AhrsNode ready — rate={rate} Hz  beta={beta}')

    def _imu_cb(self, msg: Imu):
        self._latest_imu = msg

    def _flotilla_cb(self, msg: FlotillaData):
        self._latest_flotilla = msg

    def _update(self):
        if self._latest_imu is None:
            return

        imu = self._latest_imu
        flo = self._latest_flotilla

        # Raw chip values
        raw_accel = (
            imu.linear_acceleration.x / _G,
            imu.linear_acceleration.y / _G,
            imu.linear_acceleration.z / _G,
        )
        raw_gyro = (
            math.degrees(imu.angular_velocity.x),
            math.degrees(imu.angular_velocity.y),
            math.degrees(imu.angular_velocity.z),
        )

        # Apply IMU axis remap
        accel_f = _apply_remap(raw_accel, IMU_AXIS_REMAP)
        gyro_f  = _apply_remap(raw_gyro,  IMU_AXIS_REMAP)
        gyro_rad = tuple(math.radians(g) for g in gyro_f)

        # Bootstrap orientation from first gravity reading
        if not self._orientation_initialized:
            self._filter.init_from_accel(*accel_f)
            self._orientation_initialized = True
            self.get_logger().info('Orientation bootstrapped from gravity')

        # Real elapsed time (fall back to nominal dt on first iteration)
        now_mono = time.monotonic()
        if self._last_update_time is not None:
            dt = now_mono - self._last_update_time
        else:
            dt = self._dt
        self._last_update_time = now_mono

        # Magnetometer from Flotilla body sensor (subtract hard-iron bias first)
        mag_f = None
        if flo is not None and flo.has_body_motion:
            bx, by, bz = MAG_HARD_IRON_BIAS
            raw_mag = (flo.body_mag_x - bx, flo.body_mag_y - by, flo.body_mag_z - bz)
            mag_f = _apply_remap(raw_mag, BODY_MOTION_MAG_REMAP)

        self._filter.update(
            gyro=gyro_rad,
            accel=accel_f,
            mag=mag_f,
            dt=dt,
        )

        now = self.get_clock().now().to_msg()
        qw, qx, qy, qz = self._filter.quaternion

        # Publish sensor_msgs/Imu with orientation
        imu_out = Imu()
        imu_out.header.stamp    = now
        imu_out.header.frame_id = 'imu_link'
        imu_out.orientation.w  = qw
        imu_out.orientation.x  = qx
        imu_out.orientation.y  = qy
        imu_out.orientation.z  = qz
        imu_out.angular_velocity.x    = imu.angular_velocity.x
        imu_out.angular_velocity.y    = imu.angular_velocity.y
        imu_out.angular_velocity.z    = imu.angular_velocity.z
        imu_out.linear_acceleration.x = imu.linear_acceleration.x
        imu_out.linear_acceleration.y = imu.linear_acceleration.y
        imu_out.linear_acceleration.z = imu.linear_acceleration.z
        self._imu_pub.publish(imu_out)

        # Publish AhrsData
        ahrs_out = AhrsData()
        ahrs_out.header.stamp    = now
        ahrs_out.header.frame_id = 'imu_link'
        ahrs_out.roll    = self._filter.roll
        ahrs_out.pitch   = self._filter.pitch
        ahrs_out.yaw     = self._filter.yaw
        ahrs_out.heading = self._filter.yaw
        ahrs_out.q_w = qw
        ahrs_out.q_x = qx
        ahrs_out.q_y = qy
        ahrs_out.q_z = qz
        self._ahrs_pub.publish(ahrs_out)


def main(args=None):
    rclpy.init(args=args)
    node = AhrsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

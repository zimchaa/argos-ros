"""
IMU node — publishes raw MPU-6050 data as sensor_msgs/Imu.

Topic: /imu/raw (sensor_msgs/Imu)
  orientation_covariance[0] = -1  (orientation unknown — use ahrs_node)
  angular_velocity in rad/s  (gyro-bias-corrected)
  linear_acceleration in m/s²

At startup the node collects calibration_samples readings (default 200,
~2 s at 100 Hz) while the robot must be stationary.  The mean gyro value
per axis is stored and subtracted from every subsequent reading.  A small
deadband (default 0.5 deg/s) zeros out residual noise so the AHRS filter
sees true zero angular velocity when the robot is still.

Parameters:
  publish_rate:        float (default 100.0 Hz)
  calibration_samples: int   (default 200)
  gyro_deadband_dps:   float (default 0.5 deg/s)

Rate: ~100 Hz (param: publish_rate, default 100.0)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time


_G = 9.80665  # m/s² per g


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('calibration_samples', 200)
        self.declare_parameter('gyro_deadband_dps', 0.5)

        rate = self.get_parameter('publish_rate').value
        self._cal_target = self.get_parameter('calibration_samples').value
        self._deadband = self.get_parameter('gyro_deadband_dps').value

        self._pub = self.create_publisher(Imu, '/imu/raw', 10)

        # Gyro bias calibration state
        self._calibrating = True
        self._cal_samples = []
        self._gyro_bias = (0.0, 0.0, 0.0)

        try:
            from argos_hardware.core.sensorium.imu import MPU6050
            self._imu = MPU6050()
            self.get_logger().info(
                f'MPU-6050 initialised — calibrating gyro bias '
                f'({self._cal_target} samples, keep robot still)...')
        except Exception as e:
            self._imu = None
            self._calibrating = False
            self.get_logger().error(f'MPU-6050 init failed: {e}')

        self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        if self._imu is None:
            return
        try:
            r = self._imu.read()
        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}', throttle_duration_sec=5.0)
            return

        # --- Gyro bias calibration phase ---
        if self._calibrating:
            self._cal_samples.append(
                (r.gyro_x_dps, r.gyro_y_dps, r.gyro_z_dps))
            if len(self._cal_samples) >= self._cal_target:
                n = len(self._cal_samples)
                bx = sum(s[0] for s in self._cal_samples) / n
                by = sum(s[1] for s in self._cal_samples) / n
                bz = sum(s[2] for s in self._cal_samples) / n
                self._gyro_bias = (bx, by, bz)
                self._calibrating = False
                self._cal_samples = []  # free memory
                self.get_logger().info(
                    f'Gyro bias calibrated ({n} samples): '
                    f'bx={bx:.3f}  by={by:.3f}  bz={bz:.3f} deg/s')
            return  # don't publish during calibration

        # --- Apply bias correction + deadband ---
        gx = r.gyro_x_dps - self._gyro_bias[0]
        gy = r.gyro_y_dps - self._gyro_bias[1]
        gz = r.gyro_z_dps - self._gyro_bias[2]

        if abs(gx) < self._deadband:
            gx = 0.0
        if abs(gy) < self._deadband:
            gy = 0.0
        if abs(gz) < self._deadband:
            gz = 0.0

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation_covariance[0] = -1.0  # orientation unknown

        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)
        msg.angular_velocity_covariance[0] = 0.0

        msg.linear_acceleration.x = r.accel_x_g * _G
        msg.linear_acceleration.y = r.accel_y_g * _G
        msg.linear_acceleration.z = r.accel_z_g * _G
        msg.linear_acceleration_covariance[0] = 0.0

        self._pub.publish(msg)

    def destroy_node(self):
        if self._imu is not None:
            self._imu.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

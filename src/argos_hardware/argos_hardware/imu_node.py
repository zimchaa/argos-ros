"""
IMU node — publishes raw MPU-6050 data as sensor_msgs/Imu.

Topic: /imu/raw (sensor_msgs/Imu)
  orientation_covariance[0] = -1  (orientation unknown — use ahrs_node)
  angular_velocity in rad/s
  linear_acceleration in m/s²

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
        rate = self.get_parameter('publish_rate').value

        self._pub = self.create_publisher(Imu, '/imu/raw', 10)

        try:
            from argos_hardware.core.sensorium.imu import MPU6050
            self._imu = MPU6050()
            self.get_logger().info('MPU-6050 initialised')
        except Exception as e:
            self._imu = None
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

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation_covariance[0] = -1.0  # orientation unknown

        msg.angular_velocity.x = math.radians(r.gyro_x_dps)
        msg.angular_velocity.y = math.radians(r.gyro_y_dps)
        msg.angular_velocity.z = math.radians(r.gyro_z_dps)
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

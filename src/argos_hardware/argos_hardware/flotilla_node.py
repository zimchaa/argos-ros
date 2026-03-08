"""
Flotilla node — publishes sensor data from the Pimoroni Flotilla dock.

Topic: /flotilla (argos_msgs/FlotillaData)
  Weather (BMP280): temperature_c, pressure_hpa
  Body Motion (LSM303D ch6): body_acc_{x,y,z}, body_mag_{x,y,z}, body_heading
  Arm Motion (LSM303D ch1): arm_acc_{x,y,z}
  Validity flags: has_weather, has_body_motion, has_arm_motion

Rate: ~50 Hz (param: publish_rate, default 50.0)
"""

import rclpy
from rclpy.node import Node
from argos_msgs.msg import FlotillaData
from argos_hardware.core.config import FLOTILLA_BODY_MOTION_CH, FLOTILLA_ARM_MOTION_CH


class FlotillaNode(Node):

    def __init__(self):
        super().__init__('flotilla_node')
        self.declare_parameter('publish_rate', 50.0)
        rate = self.get_parameter('publish_rate').value

        self._pub = self.create_publisher(FlotillaData, '/flotilla', 10)

        try:
            from argos_hardware.core.sensorium.flotilla import FlotillaReader
            self._reader = FlotillaReader().start()
            self.get_logger().info('FlotillaReader started')
        except Exception as e:
            self._reader = None
            self.get_logger().error(f'Flotilla init failed: {e}')

        self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        if self._reader is None:
            return

        msg = FlotillaData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        w = self._reader.weather
        if w is not None:
            msg.temperature_c = w.temperature_c
            msg.pressure_hpa  = w.pressure_hpa
            msg.has_weather   = True

        # Try configured channels first, fall back to dynamic slot order
        body = self._reader.motion_channel(FLOTILLA_BODY_MOTION_CH)
        arm = self._reader.motion_channel(FLOTILLA_ARM_MOTION_CH)
        if body is None and arm is None:
            body = self._reader.motion
            arm = self._reader.motion2

        if body is not None:
            msg.body_acc_x   = body.acc_x_g
            msg.body_acc_y   = body.acc_y_g
            msg.body_acc_z   = body.acc_z_g
            msg.body_mag_x   = body.mag_x
            msg.body_mag_y   = body.mag_y
            msg.body_mag_z   = body.mag_z
            msg.body_heading = body.heading
            msg.has_body_motion = True

        if arm is not None:
            msg.arm_acc_x = arm.acc_x_g
            msg.arm_acc_y = arm.acc_y_g
            msg.arm_acc_z = arm.acc_z_g
            msg.has_arm_motion = True

        self._pub.publish(msg)

    def destroy_node(self):
        if self._reader is not None:
            self._reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlotillaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

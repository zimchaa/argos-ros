"""
Sonar node — HC-SR04 ultrasonic distance sensor.

Topic: /sonar/range (sensor_msgs/Range)
  radiation_type: ULTRASOUND
  field_of_view:  0.26 rad (~15°)
  min_range:      0.02 m
  max_range:      4.00 m
  range:          measured distance in metres (NaN if no echo)

Rate: ~10 Hz (param: publish_rate, default 10.0)
Temperature correction uses /flotilla weather if available, else 20°C.

Parameters:
  publish_rate: float (default 10.0)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from argos_msgs.msg import FlotillaData


class SonarNode(Node):

    def __init__(self):
        super().__init__('sonar_node')
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').value

        self._pub = self.create_publisher(Range, '/sonar/range', 10)
        self._temperature_c = 20.0
        self.create_subscription(FlotillaData, '/flotilla', self._flotilla_cb, 10)

        try:
            from argos_hardware.core.sensorium.sonar import HCSR04
            self._sonar = HCSR04()
            self.get_logger().info('HC-SR04 initialised')
        except Exception as e:
            self._sonar = None
            self.get_logger().error(f'HC-SR04 init failed: {e}')

        self.create_timer(1.0 / rate, self._publish)

    def _flotilla_cb(self, msg: FlotillaData):
        if msg.has_weather:
            self._temperature_c = msg.temperature_c

    def _publish(self):
        if self._sonar is None:
            return

        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar_link'
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = 0.26  # ~15 degrees
        msg.min_range       = 0.02
        msg.max_range       = 4.00

        try:
            dist_cm = self._sonar.read_cm(self._temperature_c)
        except Exception as e:
            self.get_logger().warn(f'Sonar read error: {e}', throttle_duration_sec=5.0)
            return

        msg.range = dist_cm / 100.0 if dist_cm is not None else float('nan')
        self._pub.publish(msg)

    def destroy_node(self):
        if self._sonar is not None:
            self._sonar.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

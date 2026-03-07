"""
IR proximity node — dual digital IR sensors.

Topic: /ir/proximity (argos_msgs/IrProximity)
  ir1: BOARD 7  (CN9) — True = obstacle detected
  ir2: BOARD 12 (CN8) — True = obstacle detected

Rate: ~20 Hz (param: publish_rate, default 20.0)
"""

import rclpy
from rclpy.node import Node
from argos_msgs.msg import IrProximity


class IrNode(Node):

    def __init__(self):
        super().__init__('ir_node')
        self.declare_parameter('publish_rate', 20.0)
        rate = self.get_parameter('publish_rate').value

        self._pub = self.create_publisher(IrProximity, '/ir/proximity', 10)

        try:
            from argos_hardware.core.sensorium.ir import IRPair
            self._ir = IRPair()
            self.get_logger().info('IRPair initialised')
        except Exception as e:
            self._ir = None
            self.get_logger().error(f'IR init failed: {e}')

        self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        if self._ir is None:
            return
        try:
            ir1, ir2 = self._ir.read()
        except Exception as e:
            self.get_logger().warn(f'IR read error: {e}', throttle_duration_sec=5.0)
            return

        msg = IrProximity()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.ir1 = ir1
        msg.ir2 = ir2
        self._pub.publish(msg)

    def destroy_node(self):
        if self._ir is not None:
            self._ir.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

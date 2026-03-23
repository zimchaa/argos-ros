"""
Flotilla node — publishes sensor data from the Pimoroni Flotilla dock.

Topic: /flotilla (argos_msgs/FlotillaData)
  Weather (BMP280): temperature_c, pressure_hpa
  Body Motion (LSM303D): body_acc_{x,y,z}, body_mag_{x,y,z}, body_heading
  Arm Motion (LSM303D): arm_acc_{x,y,z}
  Validity flags: has_weather, has_body_motion, has_arm_motion

Motion modules are selected by dock channel number (configurable via
body_motion_channel and arm_motion_channel parameters) rather than arrival
order, so swapping ports or restarting won't mix up body and arm.

Rate: ~50 Hz (param: publish_rate, default 50.0)
"""

import logging
import rclpy
from rclpy.node import Node
from argos_msgs.msg import FlotillaData
from argos_hardware.core.config import FLOTILLA_BODY_MOTION_CH, FLOTILLA_ARM_MOTION_CH


class _RosLogHandler(logging.Handler):
    """Forward Python logging from FlotillaReader to ROS logger."""

    def __init__(self, ros_logger):
        super().__init__()
        self._logger = ros_logger

    def emit(self, record):
        msg = self.format(record)
        if record.levelno >= logging.ERROR:
            self._logger.error(msg)
        elif record.levelno >= logging.WARNING:
            self._logger.warn(msg)
        elif record.levelno >= logging.INFO:
            self._logger.info(msg)
        else:
            self._logger.debug(msg)


class FlotillaNode(Node):

    def __init__(self):
        super().__init__('flotilla_node')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('body_motion_channel', FLOTILLA_BODY_MOTION_CH)
        self.declare_parameter('arm_motion_channel', FLOTILLA_ARM_MOTION_CH)

        rate = self.get_parameter('publish_rate').value
        self._body_ch = self.get_parameter('body_motion_channel').value
        self._arm_ch = self.get_parameter('arm_motion_channel').value

        self._pub = self.create_publisher(FlotillaData, '/flotilla', 10)
        self._last_modules = {}

        # Route FlotillaReader's Python logging to ROS so connect/disconnect
        # events and serial errors are visible in the node output
        flotilla_logger = logging.getLogger('argos_hardware.core.sensorium.flotilla')
        flotilla_logger.setLevel(logging.DEBUG)
        flotilla_logger.addHandler(_RosLogHandler(self.get_logger()))

        try:
            from argos_hardware.core.sensorium.flotilla import FlotillaReader
            self._reader = FlotillaReader().start()
            self.get_logger().info(
                f'FlotillaReader started (body=ch{self._body_ch}, arm=ch{self._arm_ch})')
        except Exception as e:
            self._reader = None
            self.get_logger().error(f'Flotilla init failed: {e}')

        self.create_timer(1.0 / rate, self._publish)
        # Periodic module check — detect connect/disconnect
        self.create_timer(2.0, self._check_modules)
        # Re-enumerate periodically to catch slow modules (weather can take 60s+)
        self._enumerate_timer = self.create_timer(5.0, self._re_enumerate)

    def _re_enumerate(self):
        """Periodically ask the dock to re-announce modules until all are found."""
        if self._reader is None:
            return
        modules = self._reader.connected_modules
        has_body = self._body_ch in modules
        has_arm = self._arm_ch in modules
        has_weather = any(v == 'weather' for v in modules.values())
        if has_body and has_arm and has_weather:
            # All found, stop re-enumerating
            self._enumerate_timer.cancel()
            self.get_logger().info('All modules found, stopping re-enumeration')
            return
        # Ask dock to re-announce
        try:
            self._reader._serial.write(b"e\r\n")
        except Exception:
            pass

    def _check_modules(self):
        if self._reader is None:
            return
        modules = self._reader.connected_modules
        if modules != self._last_modules:
            if modules:
                parts = ', '.join(f'ch{ch}={mod}' for ch, mod in sorted(modules.items()))
                self.get_logger().info(f'Connected modules: {parts}')
            else:
                self.get_logger().warn('No modules connected')

            # Warn if configured channels aren't present
            if self._body_ch not in modules:
                self.get_logger().warn(
                    f'Body motion channel {self._body_ch} not found in connected modules')
            if self._arm_ch not in modules:
                self.get_logger().warn(
                    f'Arm motion channel {self._arm_ch} not found in connected modules')

            self._last_modules = dict(modules)

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

        # Select motion modules by channel number
        body = self._reader.motion_channel(self._body_ch)
        arm = self._reader.motion_channel(self._arm_ch)

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
            msg.arm_mag_x = arm.mag_x
            msg.arm_mag_y = arm.mag_y
            msg.arm_mag_z = arm.mag_z
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

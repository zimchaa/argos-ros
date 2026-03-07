"""
Camera node — USB webcam publisher (no cv_bridge dependency).

Topic: /camera/image_raw (sensor_msgs/Image)
  encoding: bgr8
  frame_id: camera_link

Parameters:
  device_index:  int   (default 0 → /dev/video0)
  width:         int   (default 640)
  height:        int   (default 480)
  fps:           int   (default 30)
  publish_rate:  float (default 30.0)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30.0)

        idx    = self.get_parameter('device_index').value
        width  = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps    = self.get_parameter('fps').value
        rate   = self.get_parameter('publish_rate').value

        self._pub = self.create_publisher(Image, '/camera/image_raw', 5)

        try:
            from argos_hardware.core.vision.camera import Camera
            self._cam = Camera(index=idx, width=width, height=height, fps=fps)
            w, h = self._cam.actual_resolution()
            self.get_logger().info(f'Camera opened: {w}×{h} @ {self._cam.actual_fps():.0f} fps')
        except Exception as e:
            self._cam = None
            self.get_logger().error(f'Camera init failed: {e}')

        self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        if self._cam is None:
            return
        try:
            frame = self._cam.capture()
        except Exception as e:
            self.get_logger().warn(f'Camera capture error: {e}', throttle_duration_sec=5.0)
            return

        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height     = frame.shape[0]
        msg.width      = frame.shape[1]
        msg.encoding   = 'bgr8'
        msg.is_bigendian = 0
        msg.step       = frame.shape[1] * 3
        msg.data       = frame.tobytes()
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cam is not None:
            self._cam.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

"""
Camera node — USB webcam publisher (no cv_bridge dependency).

Topics:
  /camera/image_raw   (sensor_msgs/Image)       — raw frames, bgr8
  /camera/camera_info (sensor_msgs/CameraInfo)  — intrinsic calibration

Parameters:
  device_index:  int   (default 0 → /dev/video0)
  width:         int   (default 640)
  height:        int   (default 480)
  fps:           int   (default 30)
  publish_rate:  float (default 30.0)
  camera_info_url: str (default: package config/c270_640x480.yaml)
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python.packages import get_package_share_directory


def _load_camera_info(yaml_path):
    """Load a ROS camera_info YAML and return a CameraInfo message."""
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    msg = CameraInfo()
    msg.width  = data['image_width']
    msg.height = data['image_height']
    msg.distortion_model = data['distortion_model']
    msg.k = data['camera_matrix']['data']
    msg.d = data['distortion_coefficients']['data']
    msg.r = data['rectification_matrix']['data']
    msg.p = data['projection_matrix']['data']
    return msg


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

        _config_dir = get_package_share_directory('argos_hardware')
        if width == 1280 and height == 720:
            _default_cal = 'c270_1280x720.yaml'
        else:
            _default_cal = 'c270_640x480.yaml'
        default_yaml = os.path.join(_config_dir, 'config', _default_cal)
        self.declare_parameter('camera_info_url', default_yaml)
        yaml_path = self.get_parameter('camera_info_url').value

        self._pub = self.create_publisher(Image, '/camera/image_raw', 5)
        self._info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 5)

        self._camera_info = None
        try:
            self._camera_info = _load_camera_info(yaml_path)
            self.get_logger().info(f'Camera info loaded from {yaml_path}')
        except Exception as e:
            self.get_logger().warn(f'Could not load camera_info: {e}')

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

        now = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp    = now
        msg.header.frame_id = 'camera_link'
        msg.height     = frame.shape[0]
        msg.width      = frame.shape[1]
        msg.encoding   = 'bgr8'
        msg.is_bigendian = 0
        msg.step       = frame.shape[1] * 3
        msg.data       = frame.tobytes()
        self._pub.publish(msg)

        if self._camera_info is not None:
            self._camera_info.header.stamp    = now
            self._camera_info.header.frame_id = 'camera_link'
            self._info_pub.publish(self._camera_info)

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

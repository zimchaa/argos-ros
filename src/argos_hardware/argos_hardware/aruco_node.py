"""
ArUco marker detection and pose estimation node.

Uses OpenCV's cv2.aruco module to detect fiducial markers and estimate
their 6DOF pose relative to the camera.

Subscribes:
  /camera/image_raw   (sensor_msgs/Image)      — raw camera frames
  /camera/camera_info (sensor_msgs/CameraInfo) — camera intrinsics (latched)

Publishes:
  /aruco/poses  (geometry_msgs/PoseArray) — detected marker poses in camera_link frame
  /aruco/image  (sensor_msgs/Image)       — annotated debug image with axes drawn

TF:
  camera_link → aruco_<id>  for each detected marker

Parameters:
  marker_size:   float  (default 0.05)          — physical marker side length in metres
  dictionary:    str    (default 'DICT_4X4_50')  — OpenCV ArUco dictionary name
  marker_ids:    int[]  (default [0,1,2,3])      — which marker IDs to track (others ignored)
  joint_names:   str[]  (default ['shoulder','elbow','wrist','gripper'])
                         — joint label per marker_id (same order); used in TF frame names
                           e.g. marker_ids[0]=0, joint_names[0]='shoulder' → TF: aruco_shoulder
  publish_rate:  float  (default 0.0)            — throttle detection to this rate in Hz
                                                   (0.0 = process every incoming frame)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from tf2_ros import TransformBroadcaster


class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Parameters
        self.declare_parameter('marker_size', 0.02)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_ids', [0, 1, 2, 3])
        self.declare_parameter('joint_names',
                               ['shoulder', 'elbow', 'wrist', 'gripper'])
        self.declare_parameter('publish_rate', 0.0)

        self._marker_size = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('dictionary').value
        marker_ids = list(self.get_parameter('marker_ids').value)
        joint_names = list(self.get_parameter('joint_names').value)
        self._publish_rate = self.get_parameter('publish_rate').value

        # Build ID → joint name mapping
        if len(joint_names) < len(marker_ids):
            joint_names.extend(
                [f'marker_{mid}' for mid in marker_ids[len(joint_names):]])
        self._id_to_name = dict(zip(marker_ids, joint_names))
        self._tracked_ids = set(marker_ids)

        # OpenCV ArUco setup
        self._cv2 = None
        self._aruco_dict = None
        self._aruco_params = None
        try:
            import cv2
            self._cv2 = cv2
            aruco = cv2.aruco
            dict_id = getattr(aruco, dict_name)
            self._aruco_dict = aruco.getPredefinedDictionary(dict_id)
            self._aruco_params = aruco.DetectorParameters_create()
            self.get_logger().info(
                f'ArUco: dict={dict_name}, size={self._marker_size} m, '
                f'tracking IDs {marker_ids} → {joint_names}')
        except Exception as e:
            self.get_logger().error(f'ArUco init failed: {e}')

        # Camera intrinsics (latched from first CameraInfo message)
        self._camera_matrix = None
        self._dist_coeffs = None

        # Rate limiting
        self._last_process_time = 0.0
        if self._publish_rate > 0.0:
            self._min_interval = 1.0 / self._publish_rate
        else:
            self._min_interval = 0.0

        # Publishers
        self._tf_broadcaster = TransformBroadcaster(self)
        self._poses_pub = self.create_publisher(PoseArray, '/aruco/poses', 5)
        self._image_pub = self.create_publisher(Image, '/aruco/image', 5)

        # Subscribers
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_cb, 5)
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 5)

    def _camera_info_cb(self, msg):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(
                msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received')

    def _image_cb(self, msg):
        if self._cv2 is None or self._camera_matrix is None:
            return

        # Rate limiting
        if self._min_interval > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - self._last_process_time) < self._min_interval:
                return
            self._last_process_time = now

        cv2 = self._cv2

        # Decode image
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = 'camera_optical_link'

        annotated = frame.copy()

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self._marker_size,
                self._camera_matrix, self._dist_coeffs)

            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            detected_names = []
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)

                # Skip markers we're not tracking
                if marker_id not in self._tracked_ids:
                    continue

                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                rot_mat, _ = cv2.Rodrigues(rvec)
                qx, qy, qz, qw = _rot_to_quaternion(rot_mat)

                # Pose for PoseArray
                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw
                pose_array.poses.append(pose)

                # TF broadcast with joint name
                joint_name = self._id_to_name.get(
                    marker_id, f'marker_{marker_id}')
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = 'camera_optical_link'
                t.child_frame_id = f'aruco_{joint_name}'
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self._tf_broadcaster.sendTransform(t)

                detected_names.append(joint_name)

                # Draw pose axes on annotated image
                cv2.aruco.drawAxis(
                    annotated, self._camera_matrix, self._dist_coeffs,
                    rvec, tvec, self._marker_size * 0.5)

            if detected_names:
                self.get_logger().debug(
                    f'Detected: {detected_names}',
                    throttle_duration_sec=1.0)

        self._poses_pub.publish(pose_array)

        # Publish annotated image
        out = Image()
        out.header = msg.header
        out.height = annotated.shape[0]
        out.width = annotated.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = annotated.shape[1] * 3
        out.data = annotated.tobytes()
        self._image_pub.publish(out)


def _rot_to_quaternion(R):
    """Convert 3x3 rotation matrix to (x, y, z, w) quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

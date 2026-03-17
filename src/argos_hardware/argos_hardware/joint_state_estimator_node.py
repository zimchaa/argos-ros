"""Joint state estimator — converts ArUco marker TF to /joint_states.

For each arm joint (shoulder, elbow, wrist), an ArUco marker is mounted on
that joint's link.  As the joint rotates (around the Y axis), the marker
rotates with it.  We look up the marker transform in the arm_base_link frame
and extract the Y-axis rotation angle to get the joint angle.

Requires:
  - robot_state_publisher running with the ARGOS URDF
      (publishes fixed transforms: base_link → arm_base_link,
       base_link → camera_optical_link)
  - aruco_node running
      (broadcasts camera_optical_link → aruco_shoulder/elbow/wrist/gripper)

Publishes:
  /joint_states  (sensor_msgs/JointState)  — shoulder, elbow, wrist, gripper

Services:
  /arm/calibrate_zero  (std_srvs/Trigger)
      Record current marker orientations as the zero-angle references.
      Point all joints to their physical zero position, then call this once.

Parameters:
  update_rate:      float   (default 20.0)         — publish rate in Hz
  reference_frame:  str     (default 'arm_base_link') — frame for angle extraction
  zero_offsets:     float[] (default [0,0,0,0])    — per-joint raw angle at zero
                                                      (set automatically by calibrate_zero)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformException


def _quat_to_rot(x, y, z, w):
    """Convert quaternion (x,y,z,w) to 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _extract_y_rotation(R):
    """Extract Y-axis rotation angle from rotation matrix.

    For a pure Y rotation by θ:
        R[0,2] = sin θ,  R[2,2] = cos θ  →  θ = atan2(R[0,2], R[2,2])

    Any mounting offset is absorbed by the zero calibration.
    """
    return math.atan2(R[0, 2], R[2, 2])


class JointStateEstimatorNode(Node):

    _JOINTS = ['shoulder', 'elbow', 'wrist']
    _URDF_JOINTS = ['shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_joint']

    def __init__(self):
        super().__init__('joint_state_estimator')

        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('reference_frame', 'arm_base_link')
        self.declare_parameter('zero_offsets', [0.0, 0.0, 0.0, 0.0])

        rate = self.get_parameter('update_rate').value
        self._ref_frame = self.get_parameter('reference_frame').value
        self._zero_offsets = list(self.get_parameter('zero_offsets').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_service(Trigger, '/arm/calibrate_zero', self._calibrate_cb)
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f'JointStateEstimator ready (ref={self._ref_frame}, '
            f'rate={rate} Hz, zero_offsets={[round(v,3) for v in self._zero_offsets]})')

    def _lookup_rotation(self, marker_frame):
        """Look up rotation matrix of marker_frame in reference_frame. Returns R or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._ref_frame,
                marker_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            q = t.transform.rotation
            return _quat_to_rot(q.x, q.y, q.z, q.w)
        except TransformException:
            return None

    def _update(self):
        angles = []
        for joint in self._JOINTS:
            R = self._lookup_rotation(f'aruco_{joint}')
            if R is None:
                return  # don't publish partial state — wait until all are visible
            angles.append(_extract_y_rotation(R))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._URDF_JOINTS
        msg.position = [
            angles[0] - self._zero_offsets[0],  # shoulder
            angles[1] - self._zero_offsets[1],  # elbow
            angles[2] - self._zero_offsets[2],  # wrist
            0.0,                                 # gripper (no marker yet)
        ]
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self._pub.publish(msg)

    def _calibrate_cb(self, _request, response):
        """Record current marker orientations as zero references."""
        new_offsets = []
        for joint in self._JOINTS:
            R = self._lookup_rotation(f'aruco_{joint}')
            if R is None:
                response.success = False
                response.message = f'aruco_{joint} not visible — calibration failed'
                self.get_logger().warn(response.message)
                return response
            new_offsets.append(_extract_y_rotation(R))

        self._zero_offsets[:3] = new_offsets
        parts = ', '.join(
            f'{j}={math.degrees(a):.1f}°'
            for j, a in zip(self._JOINTS, new_offsets))
        response.success = True
        response.message = f'Zero calibrated: {parts}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = JointStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

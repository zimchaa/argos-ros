"""Joint state estimator — converts ArUco marker TF + accelerometer to /joint_states.

For each arm joint (shoulder, elbow, wrist), an ArUco marker is mounted on
that joint's link.  As the joint rotates (around the Y axis), the marker
rotates with it.  We look up the marker transform in the arm_base_link frame
and extract the Y-axis rotation angle to get the joint angle.

Additionally, the Flotilla arm motion sensor (LSM303D on the upper arm link)
provides a gravity-based shoulder angle estimate.  When both ArUco and
accelerometer readings are available for the shoulder, they are blended using
accel_weight.  When only one source is available, it is used alone.

Requires:
  - robot_state_publisher running with the ARGOS URDF
      (publishes fixed transforms: base_link → arm_base_link,
       base_link → camera_optical_link)
  - aruco_node running
      (broadcasts camera_optical_link → aruco_shoulder/elbow/wrist/gripper)
  - flotilla_node running (optional — for accelerometer shoulder estimation)

Publishes:
  /joint_states  (sensor_msgs/JointState)  — shoulder, elbow, wrist, gripper

Services:
  /arm/calibrate_zero  (std_srvs/Trigger)
      Record current marker orientations and accelerometer reading as the
      zero-angle references.  Point all joints to their physical zero
      position, then call this once.

Parameters:
  update_rate:      float   (default 20.0)         — publish rate in Hz
  reference_frame:  str     (default 'arm_base_link') — frame for angle extraction
  zero_offsets:     float[] (default [0,0,0,0])    — per-joint raw angle at zero
                                                      (set automatically by calibrate_zero)
  accel_zero_offset: float  (default 0.0)          — accelerometer raw angle at zero
  body_pitch_zero:  float   (default 0.0)          — body pitch raw angle at zero
                                                      (calibration subtracts body tilt from arm reading)
  accel_weight:     float   (default 0.3)          — blend weight for accelerometer
                                                      (0=ArUco only, 1=accel only)
  max_velocity:     float   (default 1.05)         — max joint speed in rad/s (~60°/s)
                                                      readings implying faster motion are rejected
  ema_alpha:        float   (default 0.4)          — EMA smoothing factor (0=frozen, 1=no filter)
"""

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from argos_msgs.msg import FlotillaData
from argos_hardware.core.config import ARM_MOTION_AXIS_REMAP, BODY_MOTION_AXIS_REMAP
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
        self.declare_parameter('accel_zero_offset', 0.0)
        self.declare_parameter('body_pitch_zero', 0.0)
        self.declare_parameter('accel_weight', 0.3)
        self.declare_parameter('max_velocity', 1.05)   # rad/s — reject readings above this
        self.declare_parameter('ema_alpha', 0.4)        # EMA smoothing (0=frozen, 1=no filter)

        rate = self.get_parameter('update_rate').value
        self._ref_frame = self.get_parameter('reference_frame').value
        self._zero_offsets = list(self.get_parameter('zero_offsets').value)
        self._accel_zero_offset = self.get_parameter('accel_zero_offset').value
        self._body_pitch_zero = self.get_parameter('body_pitch_zero').value
        self._accel_weight = self.get_parameter('accel_weight').value
        self._max_velocity = self.get_parameter('max_velocity').value
        self._ema_alpha = self.get_parameter('ema_alpha').value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Last known good angles — held when a marker is temporarily not visible
        self._last_angles = [0.0, 0.0, 0.0]
        # Track time of last accepted reading per joint (for velocity gate)
        self._last_update_time = [None, None, None]

        # Accelerometer angles from Flotilla motion sensors
        self._accel_shoulder_raw = None  # latest raw angle from arm accel
        self._body_pitch_raw = None      # latest raw angle from body accel
        # Pre-compute axis remap: (sign, index) for X (forward) and Z (up)
        self._accel_remap_x = ARM_MOTION_AXIS_REMAP[0]
        self._accel_remap_z = ARM_MOTION_AXIS_REMAP[2]
        self._body_remap_x = BODY_MOTION_AXIS_REMAP[0]
        self._body_remap_z = BODY_MOTION_AXIS_REMAP[2]

        self.create_subscription(FlotillaData, '/flotilla', self._flotilla_cb, 10)

        # Locate config file for saving calibration
        self._config_path = os.path.join(
            get_package_share_directory('argos_hardware'),
            'config', 'joint_state_estimator.yaml')

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_service(Trigger, '/arm/calibrate_zero', self._calibrate_cb)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f'JointStateEstimator ready (ref={self._ref_frame}, '
            f'rate={rate} Hz, max_vel={math.degrees(self._max_velocity):.0f}°/s, '
            f'ema_alpha={self._ema_alpha}, accel_weight={self._accel_weight}, '
            f'zero_offsets={[round(v,3) for v in self._zero_offsets]}, '
            f'accel_zero={round(self._accel_zero_offset,3)}, '
            f'body_pitch_zero={round(self._body_pitch_zero,3)})')

    def _flotilla_cb(self, msg):
        """Extract body pitch and arm shoulder angle from accelerometers."""
        if msg.has_body_motion:
            chip = (msg.body_acc_x, msg.body_acc_y, msg.body_acc_z)
            sx, ix = self._body_remap_x
            sz, iz = self._body_remap_z
            fwd = sx * chip[ix]
            up = sz * chip[iz]
            self._body_pitch_raw = math.atan2(fwd, up)

        if msg.has_arm_motion:
            chip = (msg.arm_acc_x, msg.arm_acc_y, msg.arm_acc_z)
            sx, ix = self._accel_remap_x
            sz, iz = self._accel_remap_z
            fwd = sx * chip[ix]
            up = sz * chip[iz]
            self._accel_shoulder_raw = math.atan2(fwd, up)

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
        now = self.get_clock().now().nanoseconds * 1e-9
        any_updated = False
        for i, joint in enumerate(self._JOINTS):
            R = self._lookup_rotation(f'aruco_{joint}')
            aruco_angle = None
            if R is not None:
                aruco_angle = _extract_y_rotation(R) - self._zero_offsets[i]

            # For shoulder (i=0), also consider accelerometer with body tilt compensation
            accel_angle = None
            if i == 0 and self._accel_shoulder_raw is not None:
                arm_tilt = self._accel_shoulder_raw - self._accel_zero_offset
                # Subtract body pitch so robot tilt doesn't affect shoulder reading
                body_pitch = 0.0
                if self._body_pitch_raw is not None:
                    body_pitch = self._body_pitch_raw - self._body_pitch_zero
                accel_angle = arm_tilt - body_pitch

            # Blend sources for this joint
            if aruco_angle is not None and accel_angle is not None:
                # Both available — weighted blend
                w = self._accel_weight
                raw_angle = (1.0 - w) * aruco_angle + w * accel_angle
            elif aruco_angle is not None:
                raw_angle = aruco_angle
            elif accel_angle is not None:
                raw_angle = accel_angle
            else:
                continue

            # Velocity gate: reject readings that imply impossible speed
            if self._last_update_time[i] is not None:
                dt = now - self._last_update_time[i]
                if dt > 0:
                    velocity = abs(raw_angle - self._last_angles[i]) / dt
                    if velocity > self._max_velocity:
                        self.get_logger().debug(
                            f'{joint}: rejected {math.degrees(raw_angle):.1f}° '
                            f'(vel={math.degrees(velocity):.0f}°/s)',
                            throttle_duration_sec=1.0)
                        continue

            # EMA smoothing
            if self._last_update_time[i] is None:
                # First reading — accept directly, no smoothing
                self._last_angles[i] = raw_angle
            else:
                alpha = self._ema_alpha
                self._last_angles[i] = alpha * raw_angle + (1.0 - alpha) * self._last_angles[i]

            self._last_update_time[i] = now
            any_updated = True

        if not any_updated:
            return  # nothing seen at all yet, don't publish

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._URDF_JOINTS
        msg.position = self._last_angles + [0.0]  # gripper at zero for now
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self._pub.publish(msg)

    def _calibrate_cb(self, _request, response):
        """Record current marker orientations and accel as zero references."""
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

        # Calibrate accelerometers if available
        accel_part = ''
        if self._accel_shoulder_raw is not None:
            self._accel_zero_offset = self._accel_shoulder_raw
            accel_part = f', accel={math.degrees(self._accel_zero_offset):.1f}°'
        else:
            accel_part = ' (arm accel not available)'

        if self._body_pitch_raw is not None:
            self._body_pitch_zero = self._body_pitch_raw
            accel_part += f', body_pitch={math.degrees(self._body_pitch_zero):.1f}°'
        else:
            accel_part += ' (body accel not available)'

        # Reset filters so smoothing starts fresh from new zero
        self._last_angles = [0.0, 0.0, 0.0]
        self._last_update_time = [None, None, None]

        self._save_config()

        parts = ', '.join(
            f'{j}={math.degrees(a):.1f}°'
            for j, a in zip(self._JOINTS, new_offsets))
        response.success = True
        response.message = f'Zero calibrated and saved: {parts}{accel_part}'
        self.get_logger().info(response.message)
        return response

    def _save_config(self):
        """Write current parameters back to the config YAML file."""
        offsets_str = ', '.join(f'{v:.6f}' for v in self._zero_offsets)
        content = (
            f'joint_state_estimator:\n'
            f'  ros__parameters:\n'
            f'    update_rate: {1.0 / self._dt}\n'
            f'    reference_frame: {self._ref_frame}\n'
            f'    max_velocity: {self._max_velocity}\n'
            f'    ema_alpha: {self._ema_alpha}\n'
            f'    accel_weight: {self._accel_weight}\n'
            f'    zero_offsets: [{offsets_str}]\n'
            f'    accel_zero_offset: {self._accel_zero_offset:.6f}\n'
            f'    body_pitch_zero: {self._body_pitch_zero:.6f}\n'
        )
        try:
            with open(self._config_path, 'w') as f:
                f.write(content)
            self.get_logger().info(f'Config saved to {self._config_path}')
        except OSError as e:
            self.get_logger().warn(f'Failed to save config: {e}')


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

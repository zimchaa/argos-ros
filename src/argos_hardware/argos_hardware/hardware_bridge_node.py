"""
Hardware bridge node — owns the SafetyMonitor and exposes motor control via ROS2.

Subscriptions:
  /cmd_vel  (geometry_msgs/Twist)      — base motion
  /arm/joint_speeds (argos_msgs/JointSpeeds) — arm joint speeds

Services:
  /emergency_stop (std_srvs/Trigger)   — immediate halt

cmd_vel mapping (unicycle model):
  linear.x  in [-1.0, 1.0] → [-100, 100]% forward/backward speed
  angular.z positive = turn left (CCW, ROS convention)
  left_speed  = (linear.x - angular.z) * 100
  right_speed = (linear.x + angular.z) * 100
  Both clamped to [-100, 100] before passing to SafetyMonitor.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from argos_msgs.msg import JointSpeeds


def _clamp(v, lo=-100, hi=100):
    return max(lo, min(hi, int(v)))


class HardwareBridgeNode(Node):

    def __init__(self):
        super().__init__('hardware_bridge')

        self._safety = None
        try:
            from argos_hardware.core.safety.monitor import SafetyMonitor
            self._safety = SafetyMonitor()
            self.get_logger().info('SafetyMonitor initialised')
        except Exception as e:
            self.get_logger().error(f'SafetyMonitor init failed: {e}')

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(JointSpeeds, '/arm/joint_speeds', self._arm_cb, 10)
        self.create_service(Trigger, '/emergency_stop', self._estop_cb)

        self.get_logger().info('HardwareBridgeNode ready')

    def _cmd_vel_cb(self, msg: Twist):
        if self._safety is None:
            return
        left  = _clamp((msg.linear.x - msg.angular.z) * 100.0)
        right = _clamp((msg.linear.x + msg.angular.z) * 100.0)
        from argos_hardware.core.config import TRACK_MOTORS
        self._safety._checked_run(self._safety._base_ctrl.left,  left,  TRACK_MOTORS['left'])
        self._safety._checked_run(self._safety._base_ctrl.right, right, TRACK_MOTORS['right'])

    def _arm_cb(self, msg: JointSpeeds):
        if self._safety is None:
            return
        self._safety.arm.shoulder.run(msg.shoulder)
        self._safety.arm.elbow.run(msg.elbow)
        self._safety.arm.wrist.run(msg.wrist)
        self._safety.arm.gripper.run(msg.gripper)

    def _estop_cb(self, _request, response):
        if self._safety is not None:
            self._safety.emergency_stop()
        response.success = True
        response.message = 'Emergency stop executed'
        self.get_logger().warn('Emergency stop triggered')
        return response

    def destroy_node(self):
        if self._safety is not None:
            self._safety.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

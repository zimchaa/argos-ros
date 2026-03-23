"""Magnetometer hard-iron calibration for body and arm LSM303D sensors.

Subscribes to /flotilla and collects raw magnetometer data while the user
slowly spins the robot 360 degrees on a flat surface.  Computes hard-iron
bias (offset) for each sensor as (max + min) / 2 per axis.

Procedure:
  1. Place robot on a flat surface
  2. Start this node:  ros2 run argos_hardware calibrate_mag
  3. Slowly spin the robot a full 360 degrees (take ~30 seconds)
  4. Press ENTER when done
  5. Node prints the bias values to paste into config.py

The arm should be in a fixed position during the spin (e.g. folded down)
so the arm sensor rotates with the body.

Usage:
  ros2 run argos_hardware calibrate_mag
"""

import threading
import rclpy
from rclpy.node import Node
from argos_msgs.msg import FlotillaData


class CalibrateMagNode(Node):

    def __init__(self):
        super().__init__('calibrate_mag')

        self._body_samples = []
        self._arm_samples = []
        self._collecting = False

        self.create_subscription(FlotillaData, '/flotilla', self._flotilla_cb, 10)

        self._input_thread = threading.Thread(target=self._interactive, daemon=True)
        self._input_thread.start()

    def _flotilla_cb(self, msg):
        if not self._collecting:
            return

        if msg.has_body_motion:
            self._body_samples.append(
                (msg.body_mag_x, msg.body_mag_y, msg.body_mag_z))

        if msg.has_arm_motion:
            self._arm_samples.append(
                (msg.arm_mag_x, msg.arm_mag_y, msg.arm_mag_z))

    def _compute_bias(self, samples, name):
        """Compute hard-iron bias from min/max per axis."""
        if not samples:
            print(f'\n  {name}: NO DATA (sensor not connected?)')
            return None

        xs = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        zs = [s[2] for s in samples]

        bias_x = (max(xs) + min(xs)) // 2
        bias_y = (max(ys) + min(ys)) // 2
        bias_z = (max(zs) + min(zs)) // 2

        range_x = max(xs) - min(xs)
        range_y = max(ys) - min(ys)
        range_z = max(zs) - min(zs)

        print(f'\n  {name} ({len(samples)} samples):')
        print(f'    X: min={min(xs):+6d}  max={max(xs):+6d}  '
              f'range={range_x:5d}  bias={bias_x:+6d}')
        print(f'    Y: min={min(ys):+6d}  max={max(ys):+6d}  '
              f'range={range_y:5d}  bias={bias_y:+6d}')
        print(f'    Z: min={min(zs):+6d}  max={max(zs):+6d}  '
              f'range={range_z:5d}  bias={bias_z:+6d}')

        # Warn if ranges are very uneven (suggests incomplete rotation or
        # soft-iron distortion)
        ranges = [range_x, range_y, range_z]
        # Only check horizontal axes (the two with largest range)
        ranges_sorted = sorted(ranges, reverse=True)
        if ranges_sorted[0] > 0 and ranges_sorted[1] / ranges_sorted[0] < 0.5:
            print(f'    WARNING: axis ranges are very uneven — '
                  f'incomplete rotation or strong soft-iron distortion')

        return (bias_x, bias_y, bias_z)

    def _interactive(self):
        import time
        time.sleep(1.0)  # let subscription connect

        print()
        print('=' * 55)
        print(' Magnetometer Hard-Iron Calibration')
        print('=' * 55)
        print()
        print('Make sure flotilla_node is running.')
        print()
        print('Place the robot FLAT on a surface.')
        print('Keep the arm in a fixed position (e.g. folded down).')
        print()
        input('Press ENTER to start collecting data... ')

        self._body_samples = []
        self._arm_samples = []
        self._collecting = True

        print()
        print('COLLECTING — slowly spin the robot a full 360 degrees.')
        print('Take about 30 seconds for a smooth, level rotation.')
        print()
        input('Press ENTER when you have completed the full rotation... ')

        self._collecting = False

        print()
        print('=' * 55)
        print(' Results')
        print('=' * 55)

        body_bias = self._compute_bias(self._body_samples, 'Body LSM303D')
        arm_bias = self._compute_bias(self._arm_samples, 'Arm LSM303D')

        print()
        print('-' * 55)
        print(' Values for config.py')
        print('-' * 55)

        if body_bias:
            print(f'''
MAG_HARD_IRON_BIAS = (
    {body_bias[0]},   # mx bias
    {body_bias[1]},   # my bias
    {body_bias[2]},   # mz bias
)''')

        if arm_bias:
            print(f'''
ARM_MAG_HARD_IRON_BIAS = (
    {arm_bias[0]},   # mx bias
    {arm_bias[1]},   # my bias
    {arm_bias[2]},   # mz bias
)''')

        print()
        print('Copy the values above into:')
        print('  src/argos_hardware/argos_hardware/core/config.py')
        print()

        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateMagNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

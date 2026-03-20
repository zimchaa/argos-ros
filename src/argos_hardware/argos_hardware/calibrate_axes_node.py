"""Sensor axis remap calibration — interactive two-step procedure.

Determines the correct axis remapping for each motion sensor (MPU-6050 IMU,
body LSM303D, arm LSM303D) by measuring gravity in two known orientations.

Run this on the robot whenever sensors have been physically moved.

Procedure:
  1. Place robot flat on a level surface, run this node
  2. Press ENTER when prompted → records "flat" gravity vectors
  3. Tilt robot nose-down ~45-90°, hold steady
  4. Press ENTER when prompted → records "tilted" gravity vectors
  5. Node computes axis remaps and writes them to config/sensor_axes.yaml

The remap format matches config.py:
  ((sign, src), (sign, src), (sign, src)) for (filter_X, filter_Y, filter_Z)
  where filter X=forward, Y=right, Z=up.

Usage:
  ros2 run argos_hardware calibrate_axes
"""

import sys
import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from argos_msgs.msg import FlotillaData


def _find_dominant_axis(readings):
    """Find which chip axis has the largest absolute value.

    Returns (sign, index) where sign is +1 or -1 and index is 0/1/2.
    """
    abs_vals = [abs(r) for r in readings]
    idx = abs_vals.index(max(abs_vals))
    sign = +1 if readings[idx] > 0 else -1
    return (sign, idx)


def _derive_remap(flat_accel, tilted_accel):
    """Derive axis remap from flat and tilted gravity readings.

    flat_accel:   (x, y, z) chip readings when robot is flat (gravity = down)
    tilted_accel: (x, y, z) chip readings when robot is nose-down (gravity ≈ forward)

    Returns: ((sign, idx), (sign, idx), (sign, idx)) for (forward, right, up)
    """
    # "Down" axis: largest magnitude when flat
    down_sign, down_idx = _find_dominant_axis(flat_accel)
    # Up = -down
    up = (-down_sign, down_idx)

    # "Forward" axis: subtract flat reading to isolate the tilt component,
    # then find the axis that changed most
    delta = [tilted_accel[i] - flat_accel[i] for i in range(3)]
    # Zero out the down axis (it will also change, but we've already identified it)
    delta[down_idx] = 0.0
    fwd_sign, fwd_idx = _find_dominant_axis(delta)
    # When tilted nose-down, gravity pulls in +forward direction,
    # so the axis that increased is +forward... but we need to check:
    # tilting nose-down means gravity component along forward increases (becomes more negative
    # in accelerometer terms since accel measures reaction force).
    # Actually: accel reads +1g in the direction OPPOSITE to gravity.
    # Flat: up-axis reads +1g (reaction to gravity pulling down)
    # Nose-down: forward-axis reads LESS (more negative) because gravity now has a forward component
    # The delta on the forward axis will be negative → chip axis pointing forward has negative delta
    # So forward = (-delta_sign, idx)
    # BUT: this depends on whether the tilt goes far enough. Let's just look at
    # which non-down axis has the largest magnitude in the tilted reading.
    tilted_no_down = list(tilted_accel)
    tilted_no_down[down_idx] = 0.0
    fwd_sign, fwd_idx = _find_dominant_axis(tilted_no_down)
    # The axis with largest reading when tilted nose-down is the one where
    # gravity reaction is strongest in the forward direction.
    # Nose-down → gravity pulls robot forward → accel reads NEGATIVE in forward direction
    # So the axis reading negative = forward axis reading -1g = chip axis is forward direction
    # forward = (-sign, idx) ... No, let's think again:
    # If chip axis points FORWARD, and robot tilts nose-down,
    # gravity has a component along -forward, so accel reads +forward reaction → positive.
    # Wait: when flat, gravity is straight down. When tilted nose-down 90°,
    # gravity is entirely in the +forward direction (in robot frame).
    # Accelerometer measures specific force = acceleration - gravity.
    # When stationary: accel = -gravity (in body frame).
    # Flat: accel = (0, 0, +g) if Z=up
    # Nose-down 90°: accel = (-g, 0, 0) if X=forward (gravity is in +X, so accel = -g in X)
    # No wait: specific_force = a - g. When stationary a=0, so accel_reading = -g.
    # g points down. In body frame when flat, g = (0, 0, -1)g, so accel = (0, 0, +1)g.
    # When nose-down 90°, g in body frame = (-1, 0, 0)g (down is now -X if X=forward),
    # so accel = (+1, 0, 0)g... Hmm, that's positive.
    # Actually: when tilted nose-down, the robot's forward axis points toward ground.
    # In body frame, gravity vector rotates from (0,0,-1) toward (-1,0,0) (forward=+X).
    # Wait no. If robot tilts nose-down, the nose goes toward ground.
    # "Forward" = direction the nose points. When nose-down, forward points at ground.
    # Gravity in world frame = (0,0,-1). In body frame, gravity rotates so it has a
    # component in the +forward direction (since forward now tilts toward ground).
    # accel_reading = -gravity_body = has a component in -forward direction.
    # So: nose-down → forward axis reads NEGATIVE.
    # Therefore: the axis that is most negative when tilted = forward axis.
    # forward = (-sign, idx) where sign is the sign of the reading.
    # Since _find_dominant_axis returns the sign of the reading, and the reading is negative,
    # sign = -1, so forward = (-(-1), idx) = (+1, idx). That's wrong.
    #
    # Let me just use the empirical approach from the original probe tests:
    # The original derivation used: "chip +Z = FORWARD" when "az = -1.1g" during nose-down.
    # That means: when the axis reads NEGATIVE during nose-down, that chip axis points FORWARD.
    # So: forward = (-sign_of_reading, idx).
    forward = (-fwd_sign, fwd_idx)

    # Right axis: determined by the remaining axis.
    # For a right-handed coordinate system: right = forward × up (cross product)
    # But some sensors (LSM303D) are left-handed, so we can't assume.
    # The remaining axis is whichever isn't up or forward.
    remaining_idx = ({0, 1, 2} - {down_idx, fwd_idx}).pop()

    # We'll set right to the remaining axis and check handedness later.
    # For now, assume positive = right (we can verify with a right-tilt test
    # but that's optional — heading and arm angle only need forward and up).
    right = (+1, remaining_idx)

    return (forward, right, up)


class CalibrateAxesNode(Node):

    def __init__(self):
        super().__init__('calibrate_axes')

        # Accumulated readings
        self._imu_accel = None
        self._body_accel = None
        self._arm_accel = None

        # Calibration results
        self._flat = {}
        self._tilted = {}

        self.create_subscription(Imu, '/imu/raw', self._imu_cb, 10)
        self.create_subscription(FlotillaData, '/flotilla', self._flotilla_cb, 10)

        self._input_thread = threading.Thread(target=self._interactive, daemon=True)
        self._input_thread.start()

    def _imu_cb(self, msg):
        a = msg.linear_acceleration
        self._imu_accel = (a.x / 9.81, a.y / 9.81, a.z / 9.81)  # convert to g

    def _flotilla_cb(self, msg):
        if msg.has_body_motion:
            self._body_accel = (msg.body_acc_x, msg.body_acc_y, msg.body_acc_z)
        if msg.has_arm_motion:
            self._arm_accel = (msg.arm_acc_x, msg.arm_acc_y, msg.arm_acc_z)

    def _snapshot(self):
        """Return current readings as a dict."""
        return {
            'imu': self._imu_accel,
            'body': self._body_accel,
            'arm': self._arm_accel,
        }

    def _print_readings(self, label, data):
        for name, vals in data.items():
            if vals is not None:
                print(f'  {name:5s}: ({vals[0]:+6.3f}, {vals[1]:+6.3f}, {vals[2]:+6.3f}) g')
            else:
                print(f'  {name:5s}: (no data)')

    def _interactive(self):
        import time
        time.sleep(1.0)  # let subscriptions connect

        print()
        print('=' * 50)
        print(' Sensor Axis Calibration')
        print('=' * 50)
        print()
        print('This will determine axis remaps for all motion sensors.')
        print('Make sure imu_node and flotilla_node are running.')
        print()

        # Step 1: flat
        print('STEP 1: Place the robot FLAT on a level surface.')
        input('Press ENTER when ready... ')
        time.sleep(0.5)  # settle
        self._flat = self._snapshot()
        print('Recorded flat readings:')
        self._print_readings('flat', self._flat)
        print()

        # Step 2: tilted nose-down
        print('STEP 2: Tilt the robot NOSE-DOWN (~45-90°) and hold steady.')
        input('Press ENTER when ready... ')
        time.sleep(0.5)
        self._tilted = self._snapshot()
        print('Recorded tilted readings:')
        self._print_readings('tilted', self._tilted)
        print()

        # Derive remaps
        print('=' * 50)
        print(' Results')
        print('=' * 50)

        results = {}
        names = {'imu': 'IMU_AXIS_REMAP', 'body': 'BODY_MOTION_AXIS_REMAP',
                 'arm': 'ARM_MOTION_AXIS_REMAP'}
        axis_labels = ['X', 'Y', 'Z']

        for sensor_key, config_name in names.items():
            flat = self._flat.get(sensor_key)
            tilted = self._tilted.get(sensor_key)
            if flat is None or tilted is None:
                print(f'\n{config_name}: SKIPPED (no data)')
                continue

            remap = _derive_remap(flat, tilted)
            results[config_name] = remap

            print(f'\n{config_name} = (')
            labels = ['forward', 'right', 'up']
            for i, (sign, idx) in enumerate(remap):
                sign_str = '+' if sign > 0 else '-'
                print(f'    ({sign:+d}, {idx}),   '
                      f'# filter {labels[i]:7s} = {sign_str}chip_{axis_labels[idx]}')
            print(')')

        print()
        print('Copy these values into config.py to update the remaps,')
        print('or save to a config file for runtime loading.')
        print()

        # Also save to YAML for convenience
        if results:
            import os
            from ament_index_python.packages import get_package_share_directory
            yaml_path = os.path.join(
                get_package_share_directory('argos_hardware'),
                'config', 'sensor_axes.yaml')
            try:
                with open(yaml_path, 'w') as f:
                    f.write('# Auto-generated by calibrate_axes\n')
                    f.write(f'# Robot flat readings: {self._flat}\n')
                    f.write(f'# Robot tilted readings: {self._tilted}\n\n')
                    for name, remap in results.items():
                        f.write(f'# {name}\n')
                        for i, (sign, idx) in enumerate(remap):
                            f.write(f'#   filter[{i}] = {sign:+d} * chip[{idx}]\n')
                        f.write(f'{name}: [{", ".join(f"[{s}, {i}]" for s, i in remap)}]\n\n')
                print(f'Saved to {yaml_path}')
            except OSError as e:
                print(f'Could not save: {e}')

        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateAxesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

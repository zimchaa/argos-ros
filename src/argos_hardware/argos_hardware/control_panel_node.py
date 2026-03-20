"""
Control panel — terminal UI for testing all actuators and sensors.

Run locally on the dev machine to control the robot over the network.
Displays live sensor data and accepts keyboard commands for motors.

Controls:
  Base:   W/S = forward/backward   A/D = pivot left/right   X = stop base
  Arm:    I/K = shoulder up/down   O/L = elbow extend/retract
          P/; = wrist up/down      [/] = gripper open/close
  Speed:  +/- = adjust speed (10% steps)
  Safety: SPACE = emergency stop   Q = quit

Subscriptions (live display):
  /ir/proximity   /sonar/range   /imu/raw   /ahrs   /flotilla
"""

import sys
import math
import threading
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range, JointState
from std_srvs.srv import Trigger
from argos_msgs.msg import JointSpeeds, IrProximity, AhrsData, FlotillaData


CONTROLS_HELP = """
 ARGOS Control Panel
 ===================
 Base:  W=fwd  S=back  A=left  D=right  X=stop
 Arm:   I/K=shoulder  O/L=elbow  P/;=wrist  [/]=gripper
 Speed: +/-  adjust (current: {speed}%)
 SPACE= e-stop   Q=quit
"""


class ControlPanelNode(Node):

    def __init__(self):
        super().__init__('control_panel')

        self._speed = 50  # current speed level (10-100)

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._arm_pub = self.create_publisher(JointSpeeds, '/arm/joint_speeds', 10)

        # Emergency stop client
        self._estop_client = self.create_client(Trigger, '/emergency_stop')

        # Calibrate zero client
        self._cal_client = self.create_client(Trigger, '/arm/calibrate_zero')

        # Sensor state
        self._ir = None
        self._sonar = None
        self._ahrs = None
        self._imu = None
        self._flotilla = None
        self._joint_state = None

        # Subscriptions
        self.create_subscription(IrProximity, '/ir/proximity', self._ir_cb, 10)
        self.create_subscription(Range, '/sonar/range', self._sonar_cb, 10)
        self.create_subscription(AhrsData, '/ahrs', self._ahrs_cb, 10)
        self.create_subscription(Imu, '/imu/raw', self._imu_cb, 10)
        self.create_subscription(FlotillaData, '/flotilla', self._flotilla_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Display timer
        self.create_timer(0.25, self._display)

        self._running = True
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

    # --- sensor callbacks ---

    def _ir_cb(self, msg):      self._ir = msg
    def _sonar_cb(self, msg):   self._sonar = msg
    def _ahrs_cb(self, msg):    self._ahrs = msg
    def _imu_cb(self, msg):     self._imu = msg
    def _flotilla_cb(self, msg): self._flotilla = msg
    def _joint_state_cb(self, msg): self._joint_state = msg

    # --- motor commands ---

    def _send_base(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)

    def _send_arm(self, shoulder=0.0, elbow=0.0, wrist=0.0, gripper=0.0):
        msg = JointSpeeds()
        msg.shoulder = shoulder
        msg.elbow = elbow
        msg.wrist = wrist
        msg.gripper = gripper
        self._arm_pub.publish(msg)

    def _stop_all(self):
        self._send_base(0.0, 0.0)
        self._send_arm()

    def _emergency_stop(self):
        self._stop_all()
        if self._estop_client.service_is_ready():
            req = Trigger.Request()
            self._estop_client.call_async(req)
            self.get_logger().warn('Emergency stop sent')

    def _calibrate_zero(self):
        if self._cal_client.service_is_ready():
            req = Trigger.Request()
            future = self._cal_client.call_async(req)
            future.add_done_callback(self._cal_done_cb)
            self.get_logger().info('Calibrate zero requested...')
        else:
            self.get_logger().warn('calibrate_zero service not available')

    def _cal_done_cb(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'Calibration: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')

    # --- keyboard input ---

    def _key_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._running:
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                self._handle_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _handle_key(self, ch):
        s = self._speed / 100.0  # 0.0–1.0

        if ch in ('q', 'Q', '\x03'):  # q or Ctrl+C
            self._stop_all()
            self._running = False
            raise SystemExit(0)

        elif ch == ' ':
            self._emergency_stop()

        # Base
        elif ch in ('w', 'W'): self._send_base(s, 0.0)
        elif ch in ('s', 'S'): self._send_base(-s, 0.0)
        elif ch in ('a', 'A'): self._send_base(0.0, s)
        elif ch in ('d', 'D'): self._send_base(0.0, -s)
        elif ch in ('x', 'X'): self._send_base(0.0, 0.0)

        # Arm
        elif ch in ('i', 'I'): self._send_arm(shoulder=float(self._speed))
        elif ch in ('k', 'K'): self._send_arm(shoulder=float(-self._speed))
        elif ch in ('o', 'O'): self._send_arm(elbow=float(self._speed))
        elif ch in ('l', 'L'): self._send_arm(elbow=float(-self._speed))
        elif ch in ('p', 'P'): self._send_arm(wrist=float(self._speed))
        elif ch == ';':        self._send_arm(wrist=float(-self._speed))
        elif ch == '[':        self._send_arm(gripper=float(self._speed))
        elif ch == ']':        self._send_arm(gripper=float(-self._speed))

        # Speed adjust
        elif ch in ('+', '='):
            self._speed = min(100, self._speed + 10)
        elif ch in ('-', '_'):
            self._speed = max(10, self._speed - 10)

        # Calibrate zero
        elif ch in ('c', 'C'):
            self._calibrate_zero()

    # --- display ---

    def _display(self):
        if not self._running:
            return

        lines = [
            '\033[2J\033[H',  # clear screen + home
            ' ARGOS Control Panel',
            ' ===================',
            f' Speed: {self._speed}%',
            ' Base:  W/S=fwd/back  A/D=left/right  X=stop',
            ' Arm:   I/K=shoulder  O/L=elbow  P/;=wrist  [/]=gripper',
            ' C=calibrate zero  SPACE=e-stop  Q=quit',
            '',
        ]

        # Joint angles (from joint_state_estimator)
        if self._joint_state is not None:
            js = self._joint_state
            parts = []
            for name, pos in zip(js.name, js.position):
                parts.append(f'{name.replace("_joint","")}={math.degrees(pos):+6.1f}°')
            lines.append(' Joints: ' + '  '.join(parts))
        else:
            lines.append(' Joints: (no data)')

        lines.append('')
        lines.append(' --- Sensors ---')

        # IR
        if self._ir is not None:
            lines.append(f' IR:     ir1={"BLOCKED" if self._ir.ir1 else "clear"}  '
                         f'ir2={"BLOCKED" if self._ir.ir2 else "clear"}')
        else:
            lines.append(' IR:     (no data)')

        # Sonar
        if self._sonar is not None:
            r = self._sonar.range
            if math.isnan(r):
                lines.append(' Sonar:  out of range')
            else:
                lines.append(f' Sonar:  {r*100:.1f} cm')
        else:
            lines.append(' Sonar:  (no data)')

        # AHRS (fused output)
        if self._ahrs is not None:
            a = self._ahrs
            lines.append(f' AHRS:   roll={a.roll:+6.1f}°  pitch={a.pitch:+6.1f}°  '
                         f'yaw={a.yaw:+6.1f}°  hdg={a.heading:5.1f}°')
        else:
            lines.append(' AHRS:   (no data)')

        # IMU raw (accel + gyro)
        if self._imu is not None:
            acc = self._imu.linear_acceleration
            gyr = self._imu.angular_velocity
            lines.append(f' IMU ac: ax={acc.x:+6.2f}  ay={acc.y:+6.2f}  '
                         f'az={acc.z:+6.2f} m/s²')
            lines.append(f' IMU gy: gx={gyr.x:+6.2f}  gy={gyr.y:+6.2f}  '
                         f'gz={gyr.z:+6.2f} rad/s')
        else:
            lines.append(' IMU:    (no data)')

        # Flotilla
        if self._flotilla is not None:
            f = self._flotilla
            if f.has_weather:
                lines.append(f' Weather: {f.temperature_c:.1f}°C  '
                             f'{f.pressure_hpa:.0f}hPa')
            else:
                lines.append(' Weather: (no module)')
            if f.has_body_motion:
                lines.append(f' Body:   acc=({f.body_acc_x:+5.2f} '
                             f'{f.body_acc_y:+5.2f} {f.body_acc_z:+5.2f})g  '
                             f'mag=({f.body_mag_x:+6d} {f.body_mag_y:+6d} '
                             f'{f.body_mag_z:+6d})  hdg={f.body_heading:.0f}°')
            else:
                lines.append(' Body:   (no module)')
            if f.has_arm_motion:
                lines.append(f' Arm:    acc=({f.arm_acc_x:+5.2f} '
                             f'{f.arm_acc_y:+5.2f} {f.arm_acc_z:+5.2f})g')
            else:
                lines.append(' Arm:    (no module)')
        else:
            lines.append(' Flotilla: (no data)')

        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main(args=None):
    # Save terminal settings before anything else
    fd = sys.stdin.fileno()
    original_term = termios.tcgetattr(fd)

    rclpy.init(args=args)
    node = ControlPanelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit,
            rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Always restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, original_term)
        # Show cursor and clear screen artifacts
        sys.stdout.write('\033[?25h\033[0m\n')
        sys.stdout.flush()
        if rclpy.ok():
            node._stop_all()
            node.destroy_node()
            rclpy.shutdown()

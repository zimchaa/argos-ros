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
from sensor_msgs.msg import Imu, Range
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

        # Sensor state
        self._ir = None
        self._sonar = None
        self._ahrs = None
        self._imu = None
        self._flotilla = None

        # Subscriptions
        self.create_subscription(IrProximity, '/ir/proximity', self._ir_cb, 10)
        self.create_subscription(Range, '/sonar/range', self._sonar_cb, 10)
        self.create_subscription(AhrsData, '/ahrs', self._ahrs_cb, 10)
        self.create_subscription(Imu, '/imu/raw', self._imu_cb, 10)
        self.create_subscription(FlotillaData, '/flotilla', self._flotilla_cb, 10)

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
            ' SPACE=e-stop  Q=quit',
            '',
            ' --- Sensors ---',
        ]

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

        # AHRS
        if self._ahrs is not None:
            a = self._ahrs
            lines.append(f' AHRS:   roll={a.roll:+6.1f}  pitch={a.pitch:+6.1f}  '
                         f'yaw={a.yaw:5.1f}')
        else:
            lines.append(' AHRS:   (no data)')

        # IMU raw
        if self._imu is not None:
            acc = self._imu.linear_acceleration
            lines.append(f' IMU:    ax={acc.x:+6.2f}  ay={acc.y:+6.2f}  '
                         f'az={acc.z:+6.2f} m/s²')
        else:
            lines.append(' IMU:    (no data)')

        # Flotilla
        if self._flotilla is not None:
            f = self._flotilla
            parts = []
            if f.has_weather:
                parts.append(f'{f.temperature_c:.1f}°C  {f.pressure_hpa:.0f}hPa')
            if f.has_body_motion:
                parts.append(f'hdg={f.body_heading:.0f}°')
            lines.append(f' Flotilla: {" | ".join(parts) if parts else "(no modules)"}')
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
        node._stop_all()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

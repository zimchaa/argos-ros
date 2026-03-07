"""
High-level robot arm controller for ARGOS.
Wraps four GPIOMotor instances with named joint attributes.
"""

import RPi.GPIO as GPIO
from argos_hardware.core.drivers.gpio_motor import GPIOMotor
from argos_hardware.core.config import ARM_JOINTS


class RobotArm:
    """
    Controller for the ARGOS robot arm (4 joints via MotorShield GPIO).
    Sets GPIO mode on construction; call cleanup() on shutdown.
    """

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        self.shoulder = GPIOMotor(ARM_JOINTS["shoulder"].motor_id)
        self.elbow    = GPIOMotor(ARM_JOINTS["elbow"].motor_id)
        self.wrist    = GPIOMotor(ARM_JOINTS["wrist"].motor_id)
        self.gripper  = GPIOMotor(ARM_JOINTS["gripper"].motor_id)
        self._joints  = [self.shoulder, self.elbow, self.wrist, self.gripper]

    def stop(self):
        for joint in self._joints:
            joint.stop()

    def cleanup(self):
        self.stop()
        for joint in self._joints:
            joint.cleanup()
        GPIO.cleanup()

"""
High-level tracked base controller for ARGOS.
Wraps two I2CMotor instances (left and right) with named drive commands.
"""

from argos_hardware.core.drivers.pca9685 import I2CMotor


class TrackedBase:
    """
    Differential drive controller for a two-track robot.
    Speed values are 0–100 (percent).
    """

    def __init__(self, address=0x40, bus=1):
        self.left  = I2CMotor(address=address, motor_id=0, bus=bus)
        self.right = I2CMotor(address=address, motor_id=1, bus=bus)

    def forward(self, speed=50):
        self.left.run(speed)
        self.right.run(speed)

    def backward(self, speed=50):
        self.left.run(-speed)
        self.right.run(-speed)

    def turn_left(self, speed=50):
        """Gentle arc left: left track half-speed, right full."""
        self.left.run(speed // 2)
        self.right.run(speed)

    def turn_right(self, speed=50):
        """Gentle arc right: right track half-speed, left full."""
        self.left.run(speed)
        self.right.run(speed // 2)

    def pivot_left(self, speed=50):
        """Spin in place: tracks opposing."""
        self.left.run(-speed)
        self.right.run(speed)

    def pivot_right(self, speed=50):
        self.left.run(speed)
        self.right.run(-speed)

    def stop(self):
        self.left.stop()
        self.right.stop()

    def close(self):
        self.stop()
        I2CMotor.close_all()

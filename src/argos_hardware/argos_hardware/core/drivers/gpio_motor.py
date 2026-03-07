"""
GPIO-based DC motor driver for SB Components MotorShield.
Written from scratch using RPi.GPIO. No vendor library imported.

Vendor source read before writing:
  github.com/sbcshop/MotorShield

Each motor uses three BOARD-numbered GPIO pins:
  enable: software PWM output (default 50 Hz)
  pin_a:  direction pin A
  pin_b:  direction pin B

Positive speed → pin_a HIGH, pin_b LOW
Negative speed → pin_a LOW,  pin_b HIGH
Zero           → both LOW, PWM off

Which physical direction positive or negative produces depends on wiring.
To reverse a motor's effective direction, swap pin_a and pin_b in _MOTOR_PINS.

BOARD pin map (from vendor motorpins dict):
  Motor 1: enable=11, pin_a=15, pin_b=13
  Motor 2: enable=22, pin_a=16, pin_b=18
  Motor 3: enable=19, pin_a=21, pin_b=23
  Motor 4: enable=32, pin_a=24, pin_b=26

Note: Motor 3 uses SPI pins (MOSI/MISO/SCLK), Motor 4 uses SPI CE0/CE1 and
BCM12. Safe as GPIO when SPI is disabled (default on Pi OS Lite).

Caller must call GPIO.setmode(GPIO.BOARD) before instantiating GPIOMotor.
Interface is identical to I2CMotor: run(speed) where speed is -100..100.
"""

import RPi.GPIO as GPIO

_PWM_FREQ = 50   # Hz — matches vendor default

_MOTOR_PINS = {
    1: {"enable": 11, "pin_a": 15, "pin_b": 13},
    2: {"enable": 22, "pin_a": 16, "pin_b": 18},
    3: {"enable": 19, "pin_a": 21, "pin_b": 23},
    4: {"enable": 32, "pin_a": 24, "pin_b": 26},
}


class GPIOMotor:
    """
    Single DC motor on the SB Components MotorShield.
    Assumes GPIO.setmode(GPIO.BOARD) has already been called.
    Interface is identical to I2CMotor.
    """

    def __init__(self, motor_id, freq=_PWM_FREQ):
        if motor_id not in _MOTOR_PINS:
            raise ValueError(f"motor_id must be 1–4, got {motor_id!r}")
        pins = _MOTOR_PINS[motor_id]
        self._enable = pins["enable"]
        self._pin_a  = pins["pin_a"]
        self._pin_b  = pins["pin_b"]

        GPIO.setup(self._enable, GPIO.OUT)
        GPIO.setup(self._pin_a,  GPIO.OUT)
        GPIO.setup(self._pin_b,  GPIO.OUT)
        GPIO.output(self._pin_a, GPIO.LOW)
        GPIO.output(self._pin_b, GPIO.LOW)

        self._pwm = GPIO.PWM(self._enable, freq)
        self._pwm.start(0)

    def run(self, speed):
        """speed: -100 to 100  (negative = reverse, 0 = stop)"""
        speed = max(-100, min(100, speed))
        if speed == 0:
            self._pwm.ChangeDutyCycle(0)
            GPIO.output(self._pin_a, GPIO.LOW)
            GPIO.output(self._pin_b, GPIO.LOW)
        elif speed > 0:
            GPIO.output(self._pin_a, GPIO.HIGH)
            GPIO.output(self._pin_b, GPIO.LOW)
            self._pwm.ChangeDutyCycle(speed)
        else:
            GPIO.output(self._pin_a, GPIO.LOW)
            GPIO.output(self._pin_b, GPIO.HIGH)
            self._pwm.ChangeDutyCycle(abs(speed))

    def stop(self):
        self.run(0)

    def cleanup(self):
        self.stop()
        self._pwm.stop()
        self._pwm = None

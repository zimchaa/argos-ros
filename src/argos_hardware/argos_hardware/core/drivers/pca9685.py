"""
PCA9685 driver and I2CMotor for Waveshare Motor Driver HAT.
Written from scratch using smbus2. No vendor library imported.

Vendor source read before writing:
  github.com/waveshare/Motor-Driver-HAT  (PCA9685.py + MotorDriver.py)

PCA9685 registers (25 MHz oscillator, 12-bit counter):
  MODE1    0x00  reset/sleep/restart
  PRESCALE 0xFE  clock divider — only writable in SLEEP mode
  LEDn_ON_L/H  = 0x06 + 4n, 0x07 + 4n
  LEDn_OFF_L/H = 0x08 + 4n, 0x09 + 4n

TB6612FNG channel layout on Waveshare Motor Driver HAT @ 0x40:
  Motor 0: PWMA=ch0, AIN1=ch1, AIN2=ch2
  Motor 1: PWMB=ch5, BIN1=ch3, BIN2=ch4

Direction encoding (from vendor MotorDriver.py):
  positive speed: IN1=0, IN2=1
  negative speed: IN1=1, IN2=0
"""

import time
import math
import smbus2

_MODE1     = 0x00
_PRESCALE  = 0xFE
_LED0_ON_L = 0x06

_MOTOR_CHANNELS = {
    0: {"pwm": 0, "in1": 1, "in2": 2},   # Motor 0
    1: {"pwm": 5, "in1": 3, "in2": 4},   # Motor 1
}

# Shared PCA9685 instances keyed by (bus, address).
# Motors on the same chip share one connection so the chip is only
# initialised and frequency-set once.
_pca_instances = {}


class PCA9685:
    """Low-level PCA9685 register driver over smbus2."""

    def __init__(self, address=0x40, bus=1):
        self._bus  = smbus2.SMBus(bus)
        self._addr = address
        self._write(_MODE1, 0x00)

    def _write(self, reg, value):
        self._bus.write_byte_data(self._addr, reg, value)

    def _read(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    def set_frequency(self, freq):
        """Set PWM frequency in Hz. Puts chip to sleep to write PRESCALE."""
        prescaleval = 25_000_000.0 / 4096.0 / float(freq) - 1.0
        prescale    = math.floor(prescaleval + 0.5)
        oldmode = self._read(_MODE1)
        self._write(_MODE1, (oldmode & 0x7F) | 0x10)   # SLEEP
        self._write(_PRESCALE, int(prescale))
        self._write(_MODE1, oldmode)
        time.sleep(0.005)
        self._write(_MODE1, oldmode | 0x80)             # RESTART

    def set_pwm(self, channel, on, off):
        """Write raw 12-bit ON/OFF counts to a channel."""
        base = _LED0_ON_L + 4 * channel
        self._write(base,     on  & 0xFF)
        self._write(base + 1, 0xFF & (on  >> 8))
        self._write(base + 2, off & 0xFF)
        self._write(base + 3, 0xFF & (off >> 8))

    def set_duty(self, channel, percent):
        """Set duty cycle 0–100."""
        self.set_pwm(channel, 0, int(percent * 4096 // 100))

    def set_level(self, channel, value):
        """Set channel to digital 1 (high) or 0 (low)."""
        self.set_pwm(channel, 0, 4095 if value else 0)

    def zero_all(self):
        for ch in range(16):
            self.set_pwm(ch, 0, 0)

    def close(self):
        self.zero_all()
        self._bus.close()


class I2CMotor:
    """
    Single DC motor on a PCA9685-based HAT (Waveshare Motor Driver HAT).

    Motors at the same I2C address share a PCA9685 instance automatically,
    so the chip is only initialised once regardless of how many I2CMotors
    are created for it.

    Interface is identical to GPIOMotor: run(speed) where speed is -100..100.
    """

    def __init__(self, address=0x40, motor_id=0, bus=1, frequency=50):
        if motor_id not in _MOTOR_CHANNELS:
            raise ValueError(f"motor_id must be 0 or 1, got {motor_id!r}")
        key = (bus, address)
        if key not in _pca_instances:
            pca = PCA9685(address, bus)
            pca.set_frequency(frequency)
            _pca_instances[key] = pca
        self._pca = _pca_instances[key]
        self._ch  = _MOTOR_CHANNELS[motor_id]

    def run(self, speed):
        """speed: -100 to 100  (negative = reverse, 0 = stop)"""
        speed = max(-100, min(100, speed))
        ch = self._ch
        if speed == 0:
            self._pca.set_duty(ch["pwm"], 0)
            self._pca.set_level(ch["in1"], 0)
            self._pca.set_level(ch["in2"], 0)
        elif speed > 0:
            self._pca.set_duty(ch["pwm"], speed)
            self._pca.set_level(ch["in1"], 0)
            self._pca.set_level(ch["in2"], 1)
        else:
            self._pca.set_duty(ch["pwm"], abs(speed))
            self._pca.set_level(ch["in1"], 1)
            self._pca.set_level(ch["in2"], 0)

    def stop(self):
        self.run(0)

    @staticmethod
    def close_all():
        """Zero all channels and close every shared PCA9685 connection."""
        for pca in _pca_instances.values():
            pca.close()
        _pca_instances.clear()

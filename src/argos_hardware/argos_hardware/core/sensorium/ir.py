"""
IR proximity sensor driver for ARGOS.
Written from scratch using RPi.GPIO. No vendor library imported.

Hardware:
  IR sensor 1 → BOARD 7  (GPIO4,  CN9 on SB Components MotorShield)
  IR sensor 2 → BOARD 12 (GPIO18, CN8 on SB Components MotorShield)

Typical LM393-based digital IR proximity module output convention:
  No obstacle   → output HIGH
  Obstacle near → output LOW   ← active-low (default here)

Set active_low=False if your modules output HIGH on detection instead.
Detection range is trimmed by the onboard potentiometer.

Internal pull-ups are enabled on both input pins so unconnected or
disconnected sensors read HIGH (not-detected) rather than floating.
"""

import RPi.GPIO as GPIO

_IR1_PIN = 7    # BOARD — CN9
_IR2_PIN = 12   # BOARD — CN8


class IRSensor:
    """
    Single digital IR proximity sensor.

    detected() returns True when an obstacle is within the sensor's range.
    raw() returns the GPIO level (0 or 1) without interpretation.
    """

    def __init__(self, pin, active_low=True):
        self._pin       = pin
        self._active_low = active_low

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def detected(self) -> bool:
        """Return True if an obstacle is detected."""
        level = GPIO.input(self._pin)
        return (level == GPIO.LOW) if self._active_low else (level == GPIO.HIGH)

    def raw(self) -> int:
        """Return the raw GPIO level (0 or 1)."""
        return GPIO.input(self._pin)

    def close(self):
        GPIO.cleanup([self._pin])


class IRPair:
    """
    Both IR proximity sensors as a unit.

    Attributes:
        ir1: IRSensor on BOARD 7  (CN9)
        ir2: IRSensor on BOARD 12 (CN8)

    Usage::

        ir = IRPair()
        left, right = ir.read()   # (bool, bool)
        ir.close()
    """

    def __init__(self, active_low=True):
        self.ir1 = IRSensor(_IR1_PIN, active_low)
        self.ir2 = IRSensor(_IR2_PIN, active_low)

    def read(self):
        """Return (ir1_detected, ir2_detected) tuple of bools."""
        return self.ir1.detected(), self.ir2.detected()

    def close(self):
        self.ir1.close()
        self.ir2.close()

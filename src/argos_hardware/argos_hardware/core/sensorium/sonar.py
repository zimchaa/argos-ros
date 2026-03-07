"""
HC-SR04 ultrasonic distance sensor driver for ARGOS.
Written from scratch using RPi.GPIO. No vendor library imported.

Hardware:
  Trigger → BOARD 29 (GPIO5, CN10 on SB Components MotorShield)
  Echo    → BOARD 31 (GPIO6, CN10 on SB Components MotorShield)
  CN10 carries a 1kΩ/2kΩ voltage divider on ECHO — already level-shifted
  to ~3.3 V, safe to connect directly to the Pi GPIO input.

Operation:
  1. Pull TRIG HIGH for ≥10 μs.
  2. HC-SR04 emits 8 × 40 kHz ultrasonic pulses and raises ECHO HIGH.
  3. ECHO stays HIGH for a duration equal to the round-trip travel time.
  4. Distance (one-way) = (echo_duration × speed_of_sound) / 2.

Speed of sound:
  Default 343 m/s (≈20°C dry air). Pass temperature_c to compensate:
    v = 331.3 + 0.606 × T  (Laplace approximation, m/s)

Range: 2–400 cm. Accuracy ≈ ±3 mm at short range.
Allow ≥60 ms between successive read_cm() calls; the sensor needs time
to decay before the next trigger.

Timeouts:
  Echo start: 10 ms — if ECHO hasn't risen the path is blocked.
  Echo pulse: 38 ms — HC-SR04 holds ECHO for up to 38 ms at max range;
              beyond this the sensor has given up and ECHO will not fall.
  Both conditions return None (no valid reading).
"""

import time
import RPi.GPIO as GPIO

_TRIG = 29   # BOARD pin
_ECHO = 31   # BOARD pin

_TRIG_PULSE_S       = 0.00001   # 10 μs — minimum trigger pulse width
_ECHO_START_TIMEOUT = 0.010     # 10 ms — max wait for ECHO to rise
_ECHO_MAX_DURATION  = 0.038     # 38 ms — HC-SR04 max echo pulse


def _speed_of_sound(temperature_c: float) -> float:
    """Speed of sound in m/s at a given air temperature (Laplace formula)."""
    return 331.3 + 0.606 * temperature_c


class HCSR04:
    """
    HC-SR04 ultrasonic distance sensor driver.

    Sets GPIO mode to BOARD on construction (idempotent if already set).
    Call close() to release the two GPIO pins when done.

    Usage::

        sonar = HCSR04()
        dist = sonar.read_cm()   # float cm, or None if no echo
        sonar.close()
    """

    def __init__(self, trig=_TRIG, echo=_ECHO):
        self._trig = trig
        self._echo = echo

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self._trig, GPIO.OUT)
        GPIO.setup(self._echo, GPIO.IN)
        GPIO.output(self._trig, GPIO.LOW)
        time.sleep(0.05)   # let sensor settle after pin init

    def read_cm(self, temperature_c: float = 20.0):
        """
        Trigger one measurement and return distance in cm.

        Returns None if no echo is received within the timeout (target out
        of range, path blocked, or wiring fault).

        temperature_c: ambient temperature for speed-of-sound correction.
        Do not call more frequently than once per 60 ms.
        """
        # Send trigger pulse
        GPIO.output(self._trig, GPIO.HIGH)
        time.sleep(_TRIG_PULSE_S)
        GPIO.output(self._trig, GPIO.LOW)

        # Wait for ECHO to rise (sensor confirms it fired)
        deadline = time.monotonic() + _ECHO_START_TIMEOUT
        while GPIO.input(self._echo) == GPIO.LOW:
            if time.monotonic() > deadline:
                return None   # ECHO never rose — path blocked or no sensor

        pulse_start = time.monotonic()

        # Wait for ECHO to fall (round-trip complete)
        deadline = pulse_start + _ECHO_MAX_DURATION
        while GPIO.input(self._echo) == GPIO.HIGH:
            if time.monotonic() > deadline:
                return None   # ECHO held too long — target beyond range

        duration = time.monotonic() - pulse_start
        return (duration * _speed_of_sound(temperature_c) * 100.0) / 2.0

    def close(self):
        GPIO.cleanup([self._trig, self._echo])

"""Pimoroni Flotilla dock serial driver.

Opens the USB serial port to the Flotilla dock and reads module updates
in a background thread, storing typed sensor readings accessible at any
time via thread-safe properties.

Supported modules
-----------------
motion  — LSM303D 3-axis accelerometer + 3-axis magnetometer
weather — temperature + pressure (BMP085/BMP180)
colour  — RGBC (TCS34725)

Usage::

    reader = FlotillaReader()
    reader.start()

    m = reader.motion       # MotionReading or None
    w = reader.weather      # WeatherReading or None
    c = reader.colour       # ColourReading or None

    reader.close()

Two Motion modules are supported; the first seen on the dock becomes
`.motion`, the second (shoulder-link accelerometer/compass) becomes
`.motion2`.

Scale factors
-------------
Raw integer values are stored exactly as the firmware sends them.
Confirmed conversions are provided as properties:

- WeatherReading: .temperature_c (raw / 100) and .pressure_hpa (raw / 100)
- MotionReading:  .acc_{x,y,z}_g (raw / 16384, ±2g full scale confirmed from
  Pimoroni Motion source).  Magnetometer used raw in atan2 ratios.
"""

from __future__ import annotations

import glob
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import serial

log = logging.getLogger(__name__)

_BAUD = 115200
_READ_TIMEOUT = 0.1   # seconds — short so the loop stays responsive


# ---------------------------------------------------------------------------
# Reading dataclasses
# ---------------------------------------------------------------------------

_ACC_SCALE = 1.0 / 16384.0   # raw counts → g  (±2g full scale, 16-bit signed)


@dataclass
class MotionReading:
    """LSM303D readings from a Flotilla Motion module.

    Raw integer counts as sent by the Flotilla firmware (16-bit signed).
    With the board flat and gravity along +Z:
      - acc_z ≈ +16384 (1 g)
      - acc_x, acc_y ≈ 0

    Accelerometer scale: raw / 16384 → g  (±2g full scale).
    Magnetometer raw counts are used directly — scale cancels in atan2.

    Derived properties
    ------------------
    acc_{x,y,z}_g   : accelerometer in g
    roll_rad         : roll  (rotation about X, rad)
    pitch_rad        : pitch (rotation about Y, rad)
    heading          : tilt-compensated magnetic heading, degrees 0–360
    """
    acc_x: int = 0
    acc_y: int = 0
    acc_z: int = 0
    mag_x: int = 0
    mag_y: int = 0
    mag_z: int = 0
    timestamp: float = field(default_factory=time.monotonic)

    # --- accelerometer in physical units ------------------------------------

    @property
    def acc_x_g(self) -> float:
        return self.acc_x * _ACC_SCALE

    @property
    def acc_y_g(self) -> float:
        return self.acc_y * _ACC_SCALE

    @property
    def acc_z_g(self) -> float:
        return self.acc_z * _ACC_SCALE

    # --- tilt (roll / pitch) ------------------------------------------------

    def _unit_gravity(self) -> Tuple[float, float, float]:
        """Return the gravity unit vector (ax, ay, az); (0,0,0) if degenerate."""
        ax, ay, az = self.acc_x_g, self.acc_y_g, self.acc_z_g
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        if mag < 1e-6:
            return 0.0, 0.0, 0.0
        return ax / mag, ay / mag, az / mag

    @property
    def pitch_rad(self) -> float:
        """Pitch angle in radians (nose-up positive)."""
        ax, ay, az = self._unit_gravity()
        try:
            return math.asin(-ax)
        except ValueError:
            return 0.0

    @property
    def roll_rad(self) -> float:
        """Roll angle in radians (right-side-down positive)."""
        ax, ay, az = self._unit_gravity()
        try:
            pitch = math.asin(-ax)
            cp = math.cos(pitch)
            if abs(cp) >= abs(ay):
                return math.asin(ay / cp)
            return math.copysign(math.pi / 2, ay)
        except ValueError:
            return 0.0

    # --- heading ------------------------------------------------------------

    @property
    def heading(self) -> float:
        """Tilt-compensated magnetic heading, degrees 0–360 clockwise from north.

        Fixes two bugs present in the official Pimoroni Motion.heading:

        1. Normalization: the official code clamps raw counts to ±1 via
           ``min(fabs(raw), 1.0)``; all values outside ±1 (i.e. every real
           reading at ±2g full scale) get clamped to ±1.  We normalize to a
           proper unit vector instead.

        2. tiltcomp_y formula: the official code has a stray ``+ sin(pitch)``
           term that should be ``* sin(pitch)`` (part of the
           ``mx·sin(roll)·sin(pitch)`` product got split off).  The correct
           Freescale AN4248 formula is::

               tiltcomp_y = mx·sin(r)·sin(p) + my·cos(r) - mz·sin(r)·cos(p)
        """
        ax, ay, az = self._unit_gravity()
        if ax == ay == az == 0.0:
            return 0.0
        try:
            pitch = math.asin(-ax)
            cp = math.cos(pitch)
            roll = (
                math.asin(ay / cp)
                if abs(cp) >= abs(ay)
                else math.copysign(math.pi / 2, ay)
            )
        except ValueError:
            return 0.0

        sp, sr = math.sin(pitch), math.sin(roll)
        cp, cr = math.cos(pitch), math.cos(roll)
        mx, my, mz = self.mag_x, self.mag_y, self.mag_z

        tiltcomp_x = mx * cp + mz * sp
        tiltcomp_y = mx * sr * sp + my * cr - mz * sr * cp   # fixed formula

        h = math.atan2(tiltcomp_y, tiltcomp_x)
        if h < 0:
            h += 2 * math.pi
        return round(math.degrees(h), 2)


@dataclass
class WeatherReading:
    """Raw readings from a Flotilla Weather module.

    temperature_raw: hundredths of °C (e.g. 2062 → 20.62 °C)
    pressure_raw:    Pascals          (e.g. 101325 → 1013.25 hPa)

    Use .temperature_c and .pressure_hpa for converted values.
    Confirm the pressure scale against a known reading if needed.
    """
    temperature_raw: int = 0
    pressure_raw: int = 0
    timestamp: float = field(default_factory=time.monotonic)

    @property
    def temperature_c(self) -> float:
        return self.temperature_raw / 100.0

    @property
    def pressure_hpa(self) -> float:
        return self.pressure_raw / 100.0


@dataclass
class ColourReading:
    """Raw RGBC counts from a Flotilla Colour module (TCS34725)."""
    red: int = 0
    green: int = 0
    blue: int = 0
    clear: int = 0
    timestamp: float = field(default_factory=time.monotonic)


# ---------------------------------------------------------------------------
# Module parsers
# ---------------------------------------------------------------------------

def _parse_motion(data: str) -> MotionReading:
    v = [int(x) for x in data.split(",")]
    return MotionReading(
        acc_x=v[0], acc_y=v[1], acc_z=v[2],
        mag_x=v[3], mag_y=v[4], mag_z=v[5],
        timestamp=time.monotonic(),
    )


def _parse_weather(data: str) -> WeatherReading:
    v = [int(x) for x in data.split(",")]
    return WeatherReading(
        temperature_raw=v[0],
        pressure_raw=v[1],
        timestamp=time.monotonic(),
    )


def _parse_colour(data: str) -> ColourReading:
    v = [int(x) for x in data.split(",")]
    return ColourReading(
        red=v[0], green=v[1], blue=v[2], clear=v[3],
        timestamp=time.monotonic(),
    )


_PARSERS = {
    "motion":  _parse_motion,
    "weather": _parse_weather,
    "colour":  _parse_colour,
}


# ---------------------------------------------------------------------------
# Port detection
# ---------------------------------------------------------------------------

def _find_flotilla_port() -> Optional[str]:
    """Return the first /dev/ttyACM* device, or None."""
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    return candidates[0] if candidates else None


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class FlotillaReader:
    """Background reader for a Pimoroni Flotilla dock.

    Spawns a daemon thread that continuously reads module updates from
    the dock's USB serial port and stores the latest reading for each
    module type.  All public properties are thread-safe.

    Parameters
    ----------
    port:
        Serial device path (e.g. ``/dev/ttyACM0``).  Auto-detected
        from ``/dev/ttyACM*`` if not given.
    """

    def __init__(self, port: Optional[str] = None):
        resolved = port or _find_flotilla_port()
        if not resolved:
            raise RuntimeError(
                "Flotilla dock not found — is it plugged in? "
                "Expected /dev/ttyACM* to exist."
            )
        self._serial = serial.Serial(resolved, _BAUD, timeout=_READ_TIMEOUT)
        log.info("Flotilla dock opened on %s", resolved)

        self._lock = threading.Lock()

        # Up to 2 Motion modules, assigned by channel as updates arrive
        self._motion: List[Optional[MotionReading]] = [None, None]
        self._motion_ch: List[Optional[int]] = [None, None]

        self._weather: Optional[WeatherReading] = None
        self._colour: Optional[ColourReading] = None

        self._modules: Dict[int, str] = {}   # channel → module_type

        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="FlotillaReader"
        )
        self._running = False

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> "FlotillaReader":
        """Start the background reader thread.  Returns self for chaining."""
        self._running = True
        self._thread.start()
        # Ask the dock to re-announce all connected modules.  The dock sends
        # 'c channel/module' lines at boot, before we are listening; 'e' makes
        # it repeat them so connected_modules is populated immediately.
        self._serial.write(b"e\r\n")
        return self

    def close(self):
        """Stop the reader thread and close the serial port."""
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._serial.is_open:
            self._serial.close()
        log.info("FlotillaReader closed")

    # --- sensor properties --------------------------------------------------

    @property
    def motion(self) -> Optional[MotionReading]:
        """First Motion module (dock channel order), or None."""
        with self._lock:
            return self._motion[0]

    @property
    def motion2(self) -> Optional[MotionReading]:
        """Second Motion module (shoulder-link LSM303D), or None."""
        with self._lock:
            return self._motion[1]

    def motion_channel(self, ch: int) -> Optional[MotionReading]:
        """Return the MotionReading for a specific dock channel number, or None.

        Use this to select a Motion module by its physical port (1–8) rather
        than relying on arrival order.  Example: ``flotilla.motion_channel(6)``
        for the body-mounted sensor on port 6.
        """
        with self._lock:
            try:
                slot = self._motion_ch.index(ch)
                return self._motion[slot]
            except ValueError:
                return None

    @property
    def weather(self) -> Optional[WeatherReading]:
        """Latest WeatherReading, or None."""
        with self._lock:
            return self._weather

    @property
    def colour(self) -> Optional[ColourReading]:
        """Latest ColourReading, or None."""
        with self._lock:
            return self._colour

    @property
    def connected_modules(self) -> Dict[int, str]:
        """Dict of channel → module_type for all modules the dock has reported."""
        with self._lock:
            return dict(self._modules)

    # --- private ------------------------------------------------------------

    def _loop(self):
        while self._running:
            try:
                raw = self._serial.readline()
            except serial.SerialException as exc:
                log.error("FlotillaReader serial error: %s", exc)
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            try:
                self._dispatch(line)
            except Exception as exc:
                log.debug("FlotillaReader parse error on %r: %s", line, exc)

    def _dispatch(self, line: str):
        if line.startswith("u "):
            self._on_update(line[2:])
        elif line.startswith(("c ", "C ")):
            self._on_connect(line[2:])
        elif line.startswith(("d ", "D ")):
            self._on_disconnect(line[2:])
        # '#' info lines, 'v' version lines: silently ignored

    def _on_connect(self, rest: str):
        # Protocol: "c <channel>/<module_type>"
        rest = rest.strip()
        if "/" in rest:
            ch_str, mod = rest.split("/", 1)
        else:
            parts = rest.split()
            if len(parts) < 2:
                return
            ch_str, mod = parts[0], parts[1]
        try:
            ch = int(ch_str)
        except ValueError:
            return
        with self._lock:
            self._modules[ch] = mod
        log.info("Flotilla: %-12s connected on channel %d", mod, ch)

    def _on_disconnect(self, rest: str):
        # Protocol: "d <channel>/<module_type>" or "d <channel>"
        rest = rest.strip()
        ch_str = rest.split("/")[0].split()[0]
        try:
            ch = int(ch_str)
        except ValueError:
            return
        with self._lock:
            mod = self._modules.pop(ch, "?")
            for i, mch in enumerate(self._motion_ch):
                if mch == ch:
                    self._motion[i] = None
                    self._motion_ch[i] = None
        log.info("Flotilla: channel %d (%s) disconnected", ch, mod)

    def _on_update(self, rest: str):
        # Protocol: "u <channel>/<module_type> <csv_data>"
        parts = rest.split(" ", 1)
        if len(parts) < 2:
            return
        ch_mod, data = parts[0], parts[1]
        if "/" not in ch_mod:
            return
        ch_str, mod_type = ch_mod.split("/", 1)
        try:
            ch = int(ch_str)
        except ValueError:
            return
        # Auto-register from update if the c-announce was missed at boot
        with self._lock:
            if ch not in self._modules:
                self._modules[ch] = mod_type
                log.info("Flotilla: %-12s auto-registered on channel %d", mod_type, ch)

        parser = _PARSERS.get(mod_type)
        if parser is None:
            return

        try:
            reading = parser(data)
        except (ValueError, IndexError):
            return

        with self._lock:
            if mod_type == "motion":
                # Assign channel to a slot on first sight
                if ch not in self._motion_ch:
                    slot = next(
                        (i for i, v in enumerate(self._motion_ch) if v is None),
                        None,
                    )
                    if slot is not None:
                        self._motion_ch[slot] = ch
                        log.info(
                            "Motion module on channel %d → slot %d", ch, slot
                        )
                try:
                    self._motion[self._motion_ch.index(ch)] = reading
                except ValueError:
                    pass   # all slots taken, extra module ignored
            elif mod_type == "weather":
                self._weather = reading
            elif mod_type == "colour":
                self._colour = reading

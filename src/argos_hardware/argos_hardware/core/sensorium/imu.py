"""
MPU-6050 6-axis IMU driver for ARGOS.
Written from scratch using smbus2. No vendor library imported.

Hardware: MPU-6050 at I2C 0x68, connected via Waveshare HAT 5-pin expansion
header (VIN, 3V3, GND, SDA, SCL). AD0 tied to GND → address 0x68.

Register map (relevant subset — full map: InvenSense RM-MPU-6000A-00):
  WHO_AM_I     0x75  identity; returns 0x68
  PWR_MGMT_1   0x6B  bit 6 = SLEEP (set at power-on); write 0x00 to wake
  GYRO_CONFIG  0x1B  bits [4:3] = FS_SEL  (00=±250°/s used here)
  ACCEL_CONFIG 0x1C  bits [4:3] = AFS_SEL (00=±2g used here)

Burst read starting at 0x3B returns 14 bytes (all signed 16-bit big-endian):
  0x3B–0x3C  ACCEL_XOUT
  0x3D–0x3E  ACCEL_YOUT
  0x3F–0x40  ACCEL_ZOUT
  0x41–0x42  TEMP_OUT
  0x43–0x44  GYRO_XOUT
  0x45–0x46  GYRO_YOUT
  0x47–0x48  GYRO_ZOUT

Scale factors (default ranges):
  Accelerometer ±2g:   16384 LSB/g   → raw / 16384.0
  Gyroscope ±250°/s:   131.0 LSB/°/s → raw / 131.0
  Temperature:         °C = raw / 340.0 + 36.53
"""

import time
import smbus2
from dataclasses import dataclass, field

_ADDR = 0x68
_BUS  = 1

_REG_WHO_AM_I     = 0x75
_REG_PWR_MGMT_1   = 0x6B
_REG_GYRO_CONFIG  = 0x1B
_REG_ACCEL_CONFIG = 0x1C
_REG_DATA_START   = 0x3B   # ACCEL_XOUT_H — first of 14-byte burst

_ACCEL_SCALE = 1.0 / 16384.0   # ±2g full scale
_GYRO_SCALE  = 1.0 / 131.0     # ±250°/s full scale


def _s16(hi, lo):
    """Combine two bytes into a signed 16-bit integer (big-endian)."""
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


@dataclass
class ImuReading:
    """Raw and scaled readings from one MPU-6050 sample.

    Raw integers are the exact values from the sensor registers.
    Physical-unit properties apply the datasheet scale factors.

    Orientation convention (sensor flat, connector away from you):
      +X → right
      +Y → away from you
      +Z → up
    Accelerometer reads ~+1 g on Z when flat and stationary.
    """

    accel_x_raw: int = 0
    accel_y_raw: int = 0
    accel_z_raw: int = 0
    temp_raw:    int = 0
    gyro_x_raw:  int = 0
    gyro_y_raw:  int = 0
    gyro_z_raw:  int = 0
    timestamp: float = field(default_factory=time.monotonic)

    # --- accelerometer in g -------------------------------------------------

    @property
    def accel_x_g(self) -> float:
        return self.accel_x_raw * _ACCEL_SCALE

    @property
    def accel_y_g(self) -> float:
        return self.accel_y_raw * _ACCEL_SCALE

    @property
    def accel_z_g(self) -> float:
        return self.accel_z_raw * _ACCEL_SCALE

    # --- gyroscope in degrees per second ------------------------------------

    @property
    def gyro_x_dps(self) -> float:
        return self.gyro_x_raw * _GYRO_SCALE

    @property
    def gyro_y_dps(self) -> float:
        return self.gyro_y_raw * _GYRO_SCALE

    @property
    def gyro_z_dps(self) -> float:
        return self.gyro_z_raw * _GYRO_SCALE

    # --- temperature --------------------------------------------------------

    @property
    def temperature_c(self) -> float:
        """Die temperature in °C (datasheet formula: raw/340 + 36.53)."""
        return self.temp_raw / 340.0 + 36.53


class MPU6050:
    """
    MPU-6050 6-axis IMU driver (I2C).

    Wakes the sensor from sleep on init, sets ±2g / ±250°/s default ranges,
    and verifies device identity via WHO_AM_I.

    Usage::

        imu = MPU6050()
        reading = imu.read()
        print(reading.accel_x_g, reading.gyro_z_dps, reading.temperature_c)
        imu.close()
    """

    def __init__(self, address=_ADDR, bus=_BUS):
        self._bus  = smbus2.SMBus(bus)
        self._addr = address

        who = self._read(_REG_WHO_AM_I)
        if who != 0x68:
            raise RuntimeError(
                f"MPU-6050 WHO_AM_I returned 0x{who:02X}, expected 0x68 — "
                "check wiring and I2C address"
            )

        # Wake from sleep; use internal 8 MHz oscillator (CLKSEL=0)
        self._write(_REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)   # allow sensor to stabilise after wake

        # ±2g, ±250°/s (register reset values — written explicitly for clarity)
        self._write(_REG_ACCEL_CONFIG, 0x00)
        self._write(_REG_GYRO_CONFIG,  0x00)

    def _write(self, reg, value):
        self._bus.write_byte_data(self._addr, reg, value)

    def _read(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    def read(self) -> ImuReading:
        """Read all sensor data in a single 14-byte I2C burst."""
        data = self._bus.read_i2c_block_data(self._addr, _REG_DATA_START, 14)
        return ImuReading(
            accel_x_raw=_s16(data[0],  data[1]),
            accel_y_raw=_s16(data[2],  data[3]),
            accel_z_raw=_s16(data[4],  data[5]),
            temp_raw   =_s16(data[6],  data[7]),
            gyro_x_raw =_s16(data[8],  data[9]),
            gyro_y_raw =_s16(data[10], data[11]),
            gyro_z_raw =_s16(data[12], data[13]),
            timestamp  =time.monotonic(),
        )

    def close(self):
        self._bus.close()

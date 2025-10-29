# MPU6050_studentversion.py — minimal accel-only driver for MicroPython
# Works with either machine.I2C or machine.SoftI2C objects you pass in.

import math

# MPU6050 register map (subset)
_REG_PWR_MGMT_1   = 0x6B
_REG_ACCEL_CONFIG = 0x1C
_REG_ACCEL_XOUT_H = 0x3B

# Sensitivity scale factor for ±2g (default): 16384 LSB/g
_LSB_PER_G = 16384.0
_G         = 9.80665  # m/s^2

def _to_int16(msb, lsb):
    v = (msb << 8) | lsb
    return v - 65536 if v & 0x8000 else v

class MPU6050:
    """
    MPU6050 accel-only, using the i2c object provided by the caller.
    - i2c: machine.I2C or machine.SoftI2C instance
    - addr: 0x68 (AD0 low) or 0x69 (AD0 high)
    - up_axis: 'x'|'y'|'z' axis roughly aligned with gravity when level
    - up_sign: +1 if that axis points up, -1 if down
    """
    def __init__(self, i2c, addr=0x68, up_axis='z', up_sign=+1):
        self.i2c   = i2c
        self.addr  = addr
        self.up_axis = up_axis
        self.up_sign = 1 if up_sign >= 0 else -1
        self.acc = (0.0, 0.0, 0.0)  # m/s^2

        # Wake up device (clear sleep bit), set accel range to ±2g
        # Use writeto_mem/readfrom_mem API which is universally supported.
        self.i2c.writeto_mem(self.addr, _REG_PWR_MGMT_1, b'\x00')  # wake
        self.i2c.writeto_mem(self.addr, _REG_ACCEL_CONFIG, b'\x00')  # ±2g

        # Quick probe read to make sure the device responds; raises OSError if not OK
        _ = self.i2c.readfrom_mem(self.addr, _REG_ACCEL_XOUT_H, 6)

    def readData(self):
        """
        Reads accelerometer, stores (ax, ay, az) in m/s^2 into self.acc,
        and returns them as a tuple.
        """
        data = self.i2c.readfrom_mem(self.addr, _REG_ACCEL_XOUT_H, 6)
        ax_raw = _to_int16(data[0], data[1])
        ay_raw = _to_int16(data[2], data[3])
        az_raw = _to_int16(data[4], data[5])

        ax = (ax_raw / _LSB_PER_G) * _G
        ay = (ay_raw / _LSB_PER_G) * _G
        az = (az_raw / _LSB_PER_G) * _G

        self.acc = (ax, ay, az)
        return self.acc

    def tiltAngle(self):
        """
        Returns the board tilt angle in degrees, using the axis you configured
        as 'up_axis' relative to the horizontal plane.
        """
        ax, ay, az = self.readData()  # keep fresh

        if self.up_axis == 'x':
            up = self.up_sign * ax
            horiz = (ay * ay + az * az) ** 0.5
        elif self.up_axis == 'y':
            up = self.up_sign * ay
            horiz = (ax * ax + az * az) ** 0.5
        else:  # 'z'
            up = self.up_sign * az
            horiz = (ax * ax + ay * ay) ** 0.5

        # Angle between gravity vector and your up-axis; 0° = level
        # Using atan2 keeps it robust to small-number division.
        angle = math.degrees(math.atan2(horiz, up))
        return angle

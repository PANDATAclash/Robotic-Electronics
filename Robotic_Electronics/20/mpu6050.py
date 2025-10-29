import math

def convert_to_int16(msb, lsb):
    value = msb * 256 + lsb
    return value - 65536 if (msb & 0x80) else value

class MPU6050:
    """
    Minimal MPU6050 accel-only reader using a SoftI2C-like object that
    supports start(), write(buf), readinto(buf), stop().
    Reads ax, ay, az in m/s^2 and computes a tilt angle (deg).

    up_axis: which sensor axis is aligned with gravity when level: 'x', 'y', or 'z'
    up_sign: +1 if the up axis points upward when level, -1 if it points downward
    """

    def __init__(self, i2c, address, up_axis='x', up_sign=+1):
        self.i2c = i2c
        self.write = 2 * address
        self.read = 2 * address + 1
        self.up_axis = up_axis
        self.up_sign = 1 if up_sign >= 0 else -1

        # Wake the device (PWR_MGMT_1 = 107, set CLKSEL=1)
        self.i2c.start()
        self.i2c.write(bytearray([self.write, 107, 1]))
        self.i2c.stop()

        self.acc = [0.0, 0.0, 0.0]                 # ax, ay, az [m/s^2]
        self.buf1 = bytearray([self.write, 0x3B])  # ACCEL_XOUT_H
        self.buf2 = bytearray([self.read])         # read address
        self.readBuffer = bytearray(6)             # 6 bytes for ax,ay,az

    def readData(self):
        self.i2c.start()
        self.i2c.write(self.buf1)
        self.i2c.start()
        self.i2c.write(self.buf2)
        self.i2c.readinto(self.readBuffer)
        self.i2c.stop()

        for i in range(3):
            raw = convert_to_int16(self.readBuffer[i * 2], self.readBuffer[i * 2 + 1])
            self.acc[i] = (raw / 16384.0) * 9.80665

    def tiltAngle(self):
        # Angle from the configured up_axis; returns (90 - angle) in degrees
        self.readData()
        ax, ay, az = self.acc

        if self.up_axis == 'x':
            up = self.up_sign * ax
            horiz = (ay * ay + az * az) ** 0.5
        elif self.up_axis == 'y':
            up = self.up_sign * ay
            horiz = (ax * ax + az * az) ** 0.5
        else:  # 'z'
            up = self.up_sign * az
            horiz = (ax * ax + ay * ay) ** 0.5

        angle = math.degrees(math.atan2(horiz, up))
        return angle
        # To clamp: return max(0.0, min(180.0, 90.0 - angle))

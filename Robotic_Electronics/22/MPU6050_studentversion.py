from machine import I2C
import math

def convert_to_int16(msb, lsb):
    value = msb * 256 + lsb
    return value - 65536 if msb & 0x80 else value

class MPU6050:
    def __init__(self, i2c, address=0x68):   # fixed constructor name
        self.i2c = i2c
        self.address = address
        
        # Wake up MPU6050 (PWR_MGMT_1 = 0x6B)
        self.i2c.writeto(self.address, bytearray([0x6B, 0]))
        
        self.acc = [0.0, 0.0, 0.0]  # X, Y, Z acceleration
        
    def readData(self):
        # Read 6 bytes starting from ACCEL_XOUT_H (0x3B)
        raw_data = self.i2c.readfrom_mem(self.address, 0x3B, 6)
        
        for i in range(3):
            msb = raw_data[i*2]
            lsb = raw_data[i*2 + 1]
            raw = convert_to_int16(msb, lsb)
            self.acc[i] = raw * 9.81 / 16384  # Convert to m/s²
            
    def tiltAngle(self):
        self.readData()
        # Calculate tilt relative to X-axis
        tilt = 90 - math.degrees(math.atan2(self.acc[0], math.sqrt(self.acc[1]**2 + self.acc[2]**2)))
        return tilt    # correct indentation and valid math

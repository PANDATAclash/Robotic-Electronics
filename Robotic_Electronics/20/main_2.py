# main.py — ESP32 MicroPython
from machine import SoftI2C, Pin
from mpu6050 import MPU6050   # make sure your class is saved as mpu6050.py

# Create I2C bus (SDA=GPIO21, SCL=GPIO22, 400 kHz)
i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=400000)

# MPU6050 I2C address (default 0x68)
sensor = MPU6050(i2c, 0x68)

print("Press Enter to read acceleration and tilt angle. Ctrl+C to exit.")
try:
    while True:
        input()  # wait for Enter
        angle = sensor.tiltAngle()  # also updates sensor.acc internally
        ax, ay, az = sensor.acc
        print("ax = {:.2f} m/s^2, ay = {:.2f} m/s^2, az = {:.2f} m/s^2 | tilt = {:.2f}°"
              .format(ax, ay, az, angle))
except KeyboardInterrupt:
    print("\nBye!")

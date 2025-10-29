# boot.py — MR ELEC Q22
# Basic HW init so the board comes up in a safe state.

from machine import Pin, PWM, UART, SoftI2C
import time

# --- Pins (per course mapping) ---
PIN_LED_RED   = 19
PIN_LED_GREEN = 5
PIN_LED_BLUE  = 18   # not used in Q22
PIN_BUZZER    = 4

# --- I2C @ 400 kHz (MPU6050) ---
# Q20 requires SoftI2C on GPIO21/22 at 400 kHz.
i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=400_000)

# --- UART2 @ 115200 8N1 (to High Level Controller) ---
# RX2 = GPIO16 (given), TX2 is GPIO17 on ESP32 typical pinout.
uart2 = UART(2, baudrate=115200, bits=8, parity=None, stop=1, tx=17, rx=16)

# --- LEDs (active high) ---
led_red   = Pin(PIN_LED_RED,   Pin.OUT, value=0)
led_green = Pin(PIN_LED_GREEN, Pin.OUT, value=0)
led_blue  = Pin(PIN_LED_BLUE,  Pin.OUT, value=0)

# --- Buzzer (PWM @ 500 Hz) off by default ---
buzzer = PWM(Pin(PIN_BUZZER), freq=500, duty=0)

# Small settle delay so peripherals are ready before main starts
time.sleep_ms(50)

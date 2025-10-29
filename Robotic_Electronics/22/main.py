from machine import Pin, PWM, UART, SoftI2C
import time
from HighLevelController import HighLevelController, reg
from MPU6050_studentversion import MPU6050



def set_leds(green_on: bool, red_on: bool):
    led_green.value(1 if green_on else 0)
    led_red.value(1 if red_on else 0)

def buzzer_on(on: bool):
    # MicroPython ESP32 PWM: duty 0..1023 (classic API). 0 = off.
    buzzer.duty(512 if on else 0)

# ---- Devices ----
mpu = MPU6050(i2c)                         # wakes MPU6050 in its constructor
hlc = HighLevelController(uart2, None)     # TCP not used in Q22

# ---- Core function required by Q22 ----
def TiltAngle():
    """
    - Read accelerations and tilt from MPU6050
    - Store in reg[3..6] with correct units (Appendix E)
      acc in mm/s^2 (int), tilt in degrees (int)
    - Compare against reg[20] (warn) and reg[21] (error)
      -> LEDs & buzzer logic per spec
    """
    tilt_deg = mpu.tiltAngle()
    ax, ay, az = mpu.acc  # in m/s^2 from studentversion driver

    # Write registers (Appendix E):
    reg[3] = int(round(ax * 1000))     # mm/s^2
    reg[4] = int(round(ay * 1000))     # mm/s^2
    reg[5] = int(round(az * 1000))     # mm/s^2
    reg[6] = int(round(tilt_deg))      # degrees

    warn = reg[20]  # Tilt angle warning level [°]; 0 disables warning
    err  = reg[21]  # Tilt angle error level [°];   0 disables error

    # --- Warning LEDs ---
    if warn == 0:
        # Warning system disabled -> both LEDs off
        set_leds(green_on=False, red_on=False)
    else:
        if tilt_deg < warn:
            # OK region -> green on, red off
            set_leds(green_on=True, red_on=False)
        else:
            # Warning exceeded -> red on, green off
            set_leds(green_on=False, red_on=True)

    # --- Error (buzzer) ---
    if err == 0:
        buzzer_on(False)
    else:
        buzzer_on(tilt_deg >= err)

# ---- 100 ms scheduler loop (exact period) ----
PERIOD_MS = 100
next_tick = time.ticks_add(time.ticks_ms(), PERIOD_MS)

while True:
    # Run the tilt protection function
    TiltAngle()

    # Service UART protocol (read/write to reg[] as needed)
    hlc.readFromUart()

    # Keep exactly 100 ms period
    now = time.ticks_ms()
    # if we're late, jump to next slot to avoid drift
    if time.ticks_diff(next_tick, now) <= 0:
        next_tick = time.ticks_add(now, PERIOD_MS)
    else:
        time.sleep_ms(time.ticks_diff(next_tick, now))
        next_tick = time.ticks_add(next_tick, PERIOD_MS)

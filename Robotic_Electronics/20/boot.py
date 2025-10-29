# boot.py — minimal init for Q18 measurements (ESP32 MicroPython)
# Starts both PWM outputs at 0 duty so the motor is idle on boot.

from machine import Pin, PWM

PWM_FREQ_HZ = 20_000
PWM_PINS = (16, 17)  # pwm1, pwm2 — change if you use other pins

for p in PWM_PINS:
    try:
        PWM(Pin(p), freq=PWM_FREQ_HZ, duty=0)
    except Exception:
        pass

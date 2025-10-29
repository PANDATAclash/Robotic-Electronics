# boot.py — Q24 (cleaned)

from machine import Pin, PWM, ADC, SoftI2C, UART
import time

PIN_LED_RED   = 19
PIN_LED_GREEN = 5
PIN_LED_BLUE  = 18      # unused here
PIN_BUZZER    = 4

PIN_PWM1   = 25         # Half-bridge 1
PIN_PWM2   = 26         # Half-bridge 2
PIN_ENABLE = 13         # H-bridge enable (active high)

PIN_CS_ADC = 32         # Current-sense analog
PIN_VS_ADC = 33         # Battery/voltage analog

ENC_A  = 34
ENC_B  = 35

# --- Buttons (from reader) ---
# SW1 -> GPIO39 (input-only, NO internal pull-up; use external ~10k to 3V3)
# SW2 -> GPIO36 (input-only, NO internal pull-up; use external ~10k to 3V3)
# SW3 -> GPIO15 (has internal pull-up; WARNING: strapping pin — don't hold low at reset)
SW1 = Pin(39, Pin.IN)                       # external pull-up required
SW2 = Pin(36, Pin.IN)                       # external pull-up required
SW3 = Pin(15, Pin.IN, Pin.PULL_UP)          # internal pull-up OK

def read_buttons_active_low():
    """Returns (sw1, sw2, sw3) with 1=released, 0=pressed."""
    return (SW1.value(), SW2.value(), SW3.value())

# UART2 (HLC protocol)
uart2 = UART(2, baudrate=115200, bits=8, parity=None, stop=1)

# I2C @ 400 kHz for MPU6050
i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=400_000)

# LEDs + buzzer
LED_R = Pin(PIN_LED_RED,   Pin.OUT, value=0)
LED_G = Pin(PIN_LED_GREEN, Pin.OUT, value=0)
buzzer = PWM(Pin(PIN_BUZZER), freq=500, duty_u16=0)  # 500 Hz, off

# Motor control pins
PWM1 = PWM(Pin(PIN_PWM1), freq=50_000, duty_u16=0)
PWM2 = PWM(Pin(PIN_PWM2), freq=50_000, duty_u16=0)
EN   = Pin(PIN_ENABLE, Pin.OUT, value=0)  # disabled until safe

# ADCs
CS_ADC = ADC(Pin(PIN_CS_ADC)); CS_ADC.atten(ADC.ATTN_11DB); CS_ADC.width(ADC.WIDTH_12BIT)
VS_ADC = ADC(Pin(PIN_VS_ADC)); VS_ADC.atten(ADC.ATTN_11DB); VS_ADC.width(ADC.WIDTH_12BIT)

time.sleep_ms(50)

# boot.py
# This file is executed on every boot (including wake-boot from deepsleep)

from machine import Pin, PWM, ADC, Timer, UART
import time

# GPIO definitions
red_pin = 19
green_pin = 5
blue_pin = 18
button1_pin = 39
button2_pin = 36
button3_pin = 15
buzzer_pin = 4
PWM1_pin = 25      # GPIO25 -> pwm1
PWM2_pin = 26      # GPIO26 -> pwm2
enableMotor_pin = 13
CS_pin = 32
VS_pin = 33

# Set output pins
red_led = Pin(red_pin, Pin.OUT)
green_led = Pin(green_pin, Pin.OUT)
blue_led = Pin(blue_pin, Pin.OUT)
buzzerPin = Pin(buzzer_pin, Pin.OUT)
enableMotor = Pin(enableMotor_pin, Pin.OUT)
PWM1Pin = Pin(PWM1_pin, Pin.OUT)
PWM2Pin = Pin(PWM2_pin, Pin.OUT)

# Set input pins
button1 = Pin(button1_pin, Pin.IN)
button2 = Pin(button2_pin, Pin.IN)
button3 = Pin(button3_pin, Pin.IN)

# ADC configuration
CS_ADC = ADC(Pin(CS_pin))
CS_ADC.atten(ADC.ATTN_11DB)
CS_ADC.width(ADC.WIDTH_12BIT)

VS_ADC = ADC(Pin(VS_pin))
VS_ADC.atten(ADC.ATTN_11DB)
VS_ADC.width(ADC.WIDTH_12BIT)

# Disable motor at startup
enableMotor.value(0)

# Variables for buzzer
buzzerFrequency1 = 375
buzzerFrequency2 = 500
buzzerSwitchTime = 400
buzzerDuty = 1024

# Create PWM signals for motor control
PWM1 = PWM(PWM1Pin, freq=50000, duty_u16=0)  # 50 kHz, duty cycle 0
PWM2 = PWM(PWM2Pin, freq=50000, duty_u16=0)  # 50 kHz, duty cycle 0

# Create PWM for buzzer
buzzer = PWM(buzzerPin, freq=buzzerFrequency1, duty_u16=buzzerDuty)
buzzer.duty_u16(0)
buzzer_timer = Timer(0)

#Create UART object
uart2 = UART(2, baudrate = 115200, bits=8, parity = None, stop= 1)

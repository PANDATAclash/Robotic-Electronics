# Q14 - main.py
# All real-time work is done by interrupts/timer. Keep MCU responsive & low power.
from machine import idle
while True:
    idle()

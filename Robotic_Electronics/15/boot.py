# Q14 - boot.py (ESP32 / MicroPython)
# RGB buttons toggle R/G/B. When ALL three are ON, start a non-blocking
# 375/500 Hz siren that alternates every 400 ms. Uses IRQs + Timer (no delays).

from machine import Pin, PWM, Timer
from time import ticks_ms, ticks_diff

# ---- Pin map (per reader) ----
LED_R = Pin(19, Pin.OUT)   # Red
LED_G = Pin(5,  Pin.OUT)   # Green
LED_B = Pin(18, Pin.OUT)   # Blue

# GPIO34–39 are input-only; no internal pull resistors available.
SW1 = Pin(39, Pin.IN)                   # Button 1 (external pull on PCB)
SW2 = Pin(36, Pin.IN)                   # Button 2 (external pull on PCB)
SW3 = Pin(15, Pin.IN, Pin.PULL_UP)      # Button 3 (use internal pull-up)

BUZZER = PWM(Pin(4))                    # Active buzzer on PWM
BUZZER.duty(0)

# ---- Config ----
SIREN_TONES = (375, 500)       # Hz
SIREN_PERIOD_MS = 400
PWM_DUTY = 512                 # 50% (0..1023)
DEBOUNCE_MS = 150

# If your LEDs are wired active-low, set this True
ACTIVE_LOW_LEDS = False

# ---- State ----
state = {
    "r": False, "g": False, "b": False,
    "last_sw1": 0, "last_sw2": 0, "last_sw3": 0,
    "siren_idx": 0
}

siren_timer = Timer(-1)

def _apply_led(pin: Pin, on: bool):
    if ACTIVE_LOW_LEDS:
        pin.value(0 if on else 1)
    else:
        pin.value(1 if on else 0)

def _refresh_leds():
    _apply_led(LED_R, state["r"])
    _apply_led(LED_G, state["g"])
    _apply_led(LED_B, state["b"])

def _siren_tick(_t):
    state["siren_idx"] ^= 1
    BUZZER.freq(SIREN_TONES[state["siren_idx"]])

def _update_buzzer():
    if state["r"] and state["g"] and state["b"]:
        BUZZER.duty(PWM_DUTY)
        BUZZER.freq(SIREN_TONES[state["siren_idx"]])
        try:
            siren_timer.init(period=SIREN_PERIOD_MS, mode=Timer.PERIODIC, callback=_siren_tick)
        except Exception:
            siren_timer.deinit()
            siren_timer.init(period=SIREN_PERIOD_MS, mode=Timer.PERIODIC, callback=_siren_tick)
    else:
        siren_timer.deinit()
        BUZZER.duty(0)

def _toggle(color_key: str):
    state[color_key] = not state[color_key]
    _refresh_leds()
    _update_buzzer()

# ---- IRQ handlers with debounce ----
def _irq_sw1(_pin):
    now = ticks_ms()
    if ticks_diff(now, state["last_sw1"]) > DEBOUNCE_MS:
        state["last_sw1"] = now
        _toggle("r")

def _irq_sw2(_pin):
    now = ticks_ms()
    if ticks_diff(now, state["last_sw2"]) > DEBOUNCE_MS:
        state["last_sw2"] = now
        _toggle("g")

def _irq_sw3(_pin):
    now = ticks_ms()
    if ticks_diff(now, state["last_sw3"]) > DEBOUNCE_MS:
        state["last_sw3"] = now
        _toggle("b")

# ---- Init levels + attach interrupts ----
_refresh_leds()
SW1.irq(trigger=Pin.IRQ_FALLING, handler=_irq_sw1)
SW2.irq(trigger=Pin.IRQ_FALLING, handler=_irq_sw2)
SW3.irq(trigger=Pin.IRQ_FALLING, handler=_irq_sw3)

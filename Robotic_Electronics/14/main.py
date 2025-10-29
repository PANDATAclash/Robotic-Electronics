# main.py — ESP32 MicroPython (Thonny)
from machine import Pin, PWM, Timer
import micropython
import utime

# ---------------- Pin map (from assignment) ----------------
LED_R_PIN = 19   # Red
LED_G_PIN = 5    # Green
LED_B_PIN = 18   # Blue

SW1_PIN = 39     # Button 1 -> Red   (GPIO36/39 are input-only, no internal pulls)
SW2_PIN = 36     # Button 2 -> Green
SW3_PIN = 15     # Button 3 -> Blue  (has internal pull-up if needed)

BUZZER_PIN = 4

# ---------------- Configuration ----------------
INVERT_OUTPUT = False        # set True if LEDs are inverted on your PCB
DEBOUNCE_MS = 120

SIREN_FREQS = (375, 500)
SIREN_PERIOD_MS = 400  # ms

# ---------------- Safety for IRQs ----------------
micropython.alloc_emergency_exception_buf(100)

# ---------------- LED helpers ----------------
_led_r = Pin(LED_R_PIN, Pin.OUT)
_led_g = Pin(LED_G_PIN, Pin.OUT)
_led_b = Pin(LED_B_PIN, Pin.OUT)

def _led_write(pin_obj, on):
    # If outputs are inverted (e.g., common-anode), flip the level
    pin_obj.value(1 if (on ^ INVERT_OUTPUT) else 0)

def _led_read(pin_obj):
    raw = pin_obj.value()
    return (raw == 1) ^ INVERT_OUTPUT

# start with LEDs OFF
_led_write(_led_r, False)
_led_write(_led_g, False)
_led_write(_led_b, False)

# ---------------- Buzzer (PWM) ----------------
_buzzer_pwm = PWM(Pin(BUZZER_PIN))
_buzzer_pwm.freq(SIREN_FREQS[0])

def _buzzer_on():
    # duty compatibility: some ports have duty(), others duty_u16()
    try:
        _buzzer_pwm.duty(512)      # 0..1023 on ESP32 classic
    except AttributeError:
        _buzzer_pwm.duty_u16(32768)  # 0..65535

def _buzzer_off():
    try:
        _buzzer_pwm.duty(0)
    except AttributeError:
        _buzzer_pwm.duty_u16(0)

_buzzer_off()

_siren_timer = Timer(0)
_siren_idx = 0
_siren_active = False

def _siren_timer_cb(t):
    global _siren_idx
    _siren_idx ^= 1
    _buzzer_pwm.freq(SIREN_FREQS[_siren_idx])

def _siren_start():
    global _siren_active, _siren_idx
    if _siren_active:
        return
    _siren_idx = 0
    _buzzer_pwm.freq(SIREN_FREQS[_siren_idx])
    _buzzer_on()
    _siren_timer.init(period=SIREN_PERIOD_MS, mode=Timer.PERIODIC, callback=_siren_timer_cb)
    _siren_active = True

def _siren_stop():
    global _siren_active
    if not _siren_active:
        return
    _siren_timer.deinit()
    _buzzer_off()
    _siren_active = False

def _recompute_siren():
    if _led_read(_led_r) and _led_read(_led_g) and _led_read(_led_b):
        _siren_start()
    else:
        _siren_stop()

def _toggle_led_and_update(led_pin_obj):
    _led_write(led_pin_obj, not _led_read(led_pin_obj))
    _recompute_siren()

# ---------------- Buttons (interrupt + debounce) ----------------
# Expect the PCB to provide pulls for 36/39. We enable pull-up on 15.
_sw1 = Pin(SW1_PIN, Pin.IN)
_sw2 = Pin(SW2_PIN, Pin.IN)
_sw3 = Pin(SW3_PIN, Pin.IN, Pin.PULL_UP)

_last_sw1_ms = 0
_last_sw2_ms = 0
_last_sw3_ms = 0

def _pressed(pin_obj):
    # Treat "pressed" as LOW (active-low typical). If your PCB is active-high, change to == 1.
    return pin_obj.value() == 0

# scheduler targets (avoid lambda allocations in IRQ)
def _sched_toggle_r(_):
    _toggle_led_and_update(_led_r)

def _sched_toggle_g(_):
    _toggle_led_and_update(_led_g)

def _sched_toggle_b(_):
    _toggle_led_and_update(_led_b)

def _irq_sw1(pin):
    global _last_sw1_ms
    now = utime.ticks_ms()
    if utime.ticks_diff(now, _last_sw1_ms) < DEBOUNCE_MS:
        return
    _last_sw1_ms = now
    if _pressed(_sw1):
        micropython.schedule(_sched_toggle_r, 0)

def _irq_sw2(pin):
    global _last_sw2_ms
    now = utime.ticks_ms()
    if utime.ticks_diff(now, _last_sw2_ms) < DEBOUNCE_MS:
        return
    _last_sw2_ms = now
    if _pressed(_sw2):
        micropython.schedule(_sched_toggle_g, 0)

def _irq_sw3(pin):
    global _last_sw3_ms
    now = utime.ticks_ms()
    if utime.ticks_diff(now, _last_sw3_ms) < DEBOUNCE_MS:
        return
    _last_sw3_ms = now
    if _pressed(_sw3):
        micropython.schedule(_sched_toggle_b, 0)

EDGE = Pin.IRQ_FALLING | Pin.IRQ_RISING
_sw1.irq(trigger=EDGE, handler=_irq_sw1)
_sw2.irq(trigger=EDGE, handler=_irq_sw2)
_sw3.irq(trigger=EDGE, handler=_irq_sw3)

print("main.py: running — press SW1/SW2/SW3 to toggle R/G/B; all ON => siren")

# keep alive; work happens in IRQs
while True:
    utime.sleep_ms(250)

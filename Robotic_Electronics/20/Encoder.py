from machine import Pin
import micropython, time

micropython.alloc_emergency_exception_buf(128)

# internal state (module-level per Appendix C’s simple API)
_A = None
_B = None
t_a_rise = 0          # timestamp of last A rising edge [us]
t_b_fall = 0          # timestamp of last B falling edge [us]
dt_ar_bf = None       # A↑ -> B↓ interval [us]
dt_bf_ar = None       # B↓ -> A↑ interval [us]
_period_us = None     # last full period estimate [us]
_dir = 0              # +1 or -1 (internal)

def _on_a(pin):
    global t_a_rise, dt_bf_ar, _period_us, _dir
    if pin.value():  # rising edge on A
        now = time.ticks_us()
        # compute Bfall -> Arise if we have a recent B fall
        if t_b_fall:
            dt_bf_ar = time.ticks_diff(now, t_b_fall)
            # if both halves known, update full period & direction
            if dt_ar_bf is not None and dt_bf_ar is not None:
                _period_us = dt_ar_bf + dt_bf_ar
                _dir = +1 if dt_ar_bf < dt_bf_ar else -1
        t_a_rise = now

def _on_b(pin):
    global t_b_fall, dt_ar_bf, _period_us, _dir
    if not pin.value():  # falling edge on B
        now = time.ticks_us()
        # compute Arise -> Bfall if we have a recent A rise
        if t_a_rise:
            dt_ar_bf = time.ticks_diff(now, t_a_rise)
            if dt_ar_bf is not None and dt_bf_ar is not None:
                _period_us = dt_ar_bf + dt_bf_ar
                _dir = +1 if dt_ar_bf < dt_bf_ar else -1
        t_b_fall = now

def InitEncoder(gpio1, gpio2):
    """Configure interrupts on encA (gpio1) and encB (gpio2)."""
    global _A, _B, t_a_rise, t_b_fall, dt_ar_bf, dt_bf_ar, _period_us, _dir
    _A = Pin(gpio1, Pin.IN)   # GPIO34/35: no internal pullups
    _B = Pin(gpio2, Pin.IN)
    # reset state
    t_a_rise = 0
    t_b_fall = 0
    dt_ar_bf = None
    dt_bf_ar = None
    _period_us = None
    _dir = 0
    # IRQs: A rising, B falling (per Appendix C)
    _A.irq(trigger=Pin.IRQ_RISING, handler=_on_a)
    _B.irq(trigger=Pin.IRQ_FALLING, handler=_on_b)

def GetFrequency():
    """Return encoder frequency in Hz (float). 0.0 if not yet measurable."""
    if _period_us is None or _period_us <= 0:
        return 0.0
    # simple exponential smoothing to reduce jitter could be added if needed
    return 1_000_000.0 / float(_period_us)

# Optional helper if you want direction later:
def GetDirection():
    """Return +1 / -1 direction hint (based on which half-interval is longer)."""
    return _dir

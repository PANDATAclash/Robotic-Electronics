# Q16 - boot.py (ESP32 / MicroPython)
# Sets up the ADCs and keeps the half-bridges disabled for sensor measurements.

from machine import Pin, ADC

# --- Half-bridge enable (keep DISABLED during measurements) ---
HB_ENABLE = Pin(13, Pin.OUT)
HB_ENABLE.value(0)   # 0 = disabled; 1 = enabled

# --- ADC setup ---
# VS (voltage sensor) on GPIO33, CS (current sensor) on GPIO32
adc_vs = ADC(Pin(33))
adc_cs = ADC(Pin(32))

# Use 11 dB attenuation and 12-bit width as required
adc_vs.atten(ADC.ATTN_11DB)
adc_cs.atten(ADC.ATTN_11DB)
adc_vs.width(ADC.WIDTH_12BIT)
adc_cs.width(ADC.WIDTH_12BIT)

# --- Optional: tiny helper to read microvolts with fallback if read_uv() is missing ---
def _read_uv(adc):
    try:
        return adc.read_uv()  # preferred: calibrated microvolts
    except AttributeError:
        # Fallback: approximate scaling from raw counts (0..4095) to microvolts.
        # 11 dB range tops out around ~3.2–3.3 V depending on chip; 3.3 V used as a sane default.
        raw = adc.read()
        return int(raw * (3_300_000 / 4095))

# Expose helpers so main.py can import them from the global namespace
READ_UV = _read_uv

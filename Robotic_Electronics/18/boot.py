from machine import Pin, PWM, ADC
from Encoder import InitEncoder, GetFrequency  # encoder support

# --- Half-bridge enable & ADCs from Q15/16 ---
HB_ENABLE = Pin(13, Pin.OUT)        # LED2 mirrors this state
adc_cs = ADC(Pin(32)); adc_cs.atten(ADC.ATTN_11DB); adc_cs.width(ADC.WIDTH_12BIT)

# --- PWMs from Q17 (50 kHz) ---
PWM_FREQ = 50_000
pwm1 = PWM(Pin(25), freq=PWM_FREQ, duty=0)  # half-bridge 1
pwm2 = PWM(Pin(26), freq=PWM_FREQ, duty=0)  # half-bridge 2

# --- Encoder init for encA=34, encB=35 (run once at boot) ---
InitEncoder(34, 35)  # Appendix C says encA/encB on GPIO34/35. :contentReference[oaicite:2]{index=2}

# Start disabled for safety; main.py will enable
HB_ENABLE.value(0)

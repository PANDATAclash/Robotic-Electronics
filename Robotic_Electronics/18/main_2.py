# Q18: PWMs + on-demand encoder & current readout
from time import sleep_ms
from Encoder import GetFrequency

# Objects created in boot.py: HB_ENABLE, pwm1, pwm2, adc_cs

# --- Use YOUR measured model from Q16c ---
I0 = -3.8736    # [A]
c2 =  2.3182    # [A/V]

def clamp(n, lo, hi):
    return lo if n < lo else hi if n > hi else n

def set_pwm(dc):
    """Map dc (-950..950) to the two legs."""
    if dc >= 0:
        pwm1.duty(dc); pwm2.duty(0)
    else:
        pwm1.duty(0);  pwm2.duty(-dc)

def read_current_A():
    # Use accurate ADC: read_uv -> volts
    vcs_V = adc_cs.read_uv() / 1_000_000
    return I0 + c2 * vcs_V, vcs_V

print("Q18 ready. Enter duty -950..950. Press ENTER (empty) to read encoder and current.")
HB_ENABLE.value(1)  # enable half-bridges (LED2 ON). :contentReference[oaicite:3]{index=3}

while True:
    try:
        s = input("duty or ENTER to read: ").strip()

        # Empty input -> show encoder freq and motor current
        if s == "":
            f_hz = GetFrequency()                # encoder freq (signed by direction) :contentReference[oaicite:4]{index=4}
            I_A, vcs_V = read_current_A()        # current via I = I0 + c2*VCS  :contentReference[oaicite:5]{index=5}
            print("Encoder = {:>6.1f} Hz | VCS = {:>5.3f} V | I = {:>+6.3f} A".format(f_hz, vcs_V, I_A))
            continue

        # Number entered -> set duty
        dc = int(s)
        dc = clamp(dc, -950, 950)
        set_pwm(dc)
        # small pause so terminal stays readable
        sleep_ms(30)

    except KeyboardInterrupt:
        print("\nStopping.")
        pwm1.duty(0); pwm2.duty(0); HB_ENABLE.value(0)
        break
    except Exception as e:
        print("Please enter an integer in -950..950 (or just press ENTER). Error:", e)

# Q16 - main.py
# Prints the current-sensor voltage (CS on GPIO32) every 500 ms in mV.
# Use Ctrl+C to stop. Half-bridges remain disabled (HB_ENABLE=0) while testing.

from time import sleep_ms

# The following objects are created in boot.py and available globally:
# - HB_ENABLE  (Pin(13, OUT))
# - adc_cs, adc_vs  (ADC instances on GPIO32/33)
# - READ_UV(adc)    (helper that returns microvolts with a safe fallback)

PRINT_VS_TOO = False   # set True if you also want to see VS lines

print("Q16: Reading CS (GPIO32) every 500 ms in mV. Ctrl+C to stop.")
print("Half-bridges are DISABLED (HB_ENABLE=0).")

while True:
    try:
        cs_uv = READ_UV(adc_cs)      # microvolts
        cs_mv = cs_uv // 1000        # millivolts
        line = "VCS = {} mV".format(cs_mv)

        if PRINT_VS_TOO:
            vs_uv = READ_UV(adc_vs)
            vs_mv = vs_uv // 1000
            line += " | VS = {} mV".format(vs_mv)

        print(line)
        sleep_ms(500)

    except KeyboardInterrupt:
        # Safety: keep drivers disabled on exit
        try:
            HB_ENABLE.value(0)
        except:
            pass
        print("\nStopped.")
        break

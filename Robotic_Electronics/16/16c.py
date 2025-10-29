import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Measured data from 16b
# -----------------------------
# Current sweep [A] and VCS [V] for +I
I_pos = np.array([0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.06])
V_pos = np.array([1.672, 1.8, 1.93, 2.06, 2.19, 2.32, 2.45, 2.55])

# Current sweep [A] and VCS [V] for –I (currents negated)
I_neg = -np.array([0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.06])
V_neg = np.array([1.672, 1.544, 1.417, 1.283, 1.153, 1.024, 0.89, 0.78])

# Combine both polarities
V = np.concatenate([V_pos, V_neg])     # VCS [V]
I = np.concatenate([I_pos, I_neg])     # Current [A]

# -----------------------------
# Linear regression: I = I0 + c2 * VCS
# -----------------------------
c2, I0 = np.polyfit(V, I, 1)  # slope, intercept

# -----------------------------
# Theoretical expectations
# -----------------------------
# INA181A1 gain and shunt; 3.3 V rail with 10k:10k divider -> Vref = ~1.65 V
G = 20.0          # INA181A1 gain (change if your variant differs)
R1 = 25e-3        # 25 mΩ shunt
Vcc = 3.3         # ESP32/board supply
R_top = 10e3      # divider top resistor (e.g., R37)
R_bottom = 10e3   # divider bottom resistor (R38 = 10 kΩ)
Vref = Vcc * (R_bottom / (R_top + R_bottom))

c2_th = 1.0 / (G * R1)          # A/V
I0_th = -Vref / (G * R1)        # A

# Percentage differences
def pct(meas, th):
    return 100.0 * (meas - th) / th

c2_diff_pct = pct(c2, c2_th)
I0_diff_pct = pct(I0, I0_th)

# -----------------------------
# Print results
# -----------------------------
print(f"Fitted model: I = I0 + c2*VCS")
print(f"  I0 (fit)  = {I0:.4f} A")
print(f"  c2 (fit)  = {c2:.4f} A/V")
print()
print(f"Theory (G={G:.0f}, R1={R1*1e3:.1f} mΩ, Vref={Vref:.3f} V):")
print(f"  I0_th     = {I0_th:.4f} A")
print(f"  c2_th     = {c2_th:.4f} A/V")
print()
print(f"Percent difference:")
print(f"  ΔI0  = {c2_diff_pct:.2f}%")
print(f"  Δc2  = {I0_diff_pct:.2f}%")

# -----------------------------
# Plot in requested units (mA vs mV)
# -----------------------------
plt.figure()
plt.scatter(V*1000, I*1000, marker='x', label='Measured points')
V_line = np.linspace(V.min()-0.1, V.max()+0.1, 200)
I_fit = I0 + c2 * V_line
plt.plot(V_line*1000, I_fit*1000, label='Linear fit')

plt.xlabel('VCS [mV]')
plt.ylabel('Current [mA]')
plt.title('I = I0 + c2·VCS (linear regression)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# -----------------------------
# Interpretation guideline for your report:
# If |Δ| > 3%, discuss: ADC absolute error/nonlinearity, exact INA181 gain code,
# Vref not exactly Vcc/2, shunt tolerance & self-heating, and wiring/lead resistance.
# -----------------------------

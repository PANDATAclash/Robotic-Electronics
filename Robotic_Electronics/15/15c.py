# q15_plot_and_ratio.py
# Requirements: numpy, matplotlib

import numpy as np
import matplotlib.pyplot as plt

# ---- Your measurements (from the table) ----
Vin_V  = np.array([6, 7, 8, 9, 10, 11, 12, 13, 14, 15], dtype=float)
VS_mV  = np.array([1240, 1450, 1660, 1862, 2068, 2278, 2485, 2676, 2880, 3083], dtype=float)

# ---- (b) Plot VS vs Vin ----
plt.figure()
plt.plot(Vin_V, VS_mV, marker='o')
plt.xlabel('Vin [V]')
plt.ylabel('VS [mV]')
plt.title('VS vs Vin')
plt.grid(True)
plt.tight_layout()
plt.show()

# ---- (c) Compute c1 from measurements and from theory ----
VS_V = VS_mV / 1000.0

# Average proportional factor (measured): mean(Vin / VS)
c1_meas = np.mean(Vin_V / VS_V)

# Theoretical c1 from divider:
#   VS = Vin * R_bottom / (R_top + R_bottom)
#   => Vin/VS = (R_top + R_bottom) / R_bottom
# Use the orientation that matches your board: top=39k, bottom=10k
R_top    = 39_000.0   # ohms  (R23 physically at Vin side)
R_bottom = 10_000.0   # ohms  (R44 to GND)
c1_theory = (R_top + R_bottom) / R_bottom

# Percent difference
diff_pct = 100.0 * (c1_meas - c1_theory) / c1_theory

print(f"Measured c1 (mean Vin/VS): {c1_meas:.6f}")
print(f"Theoretical c1 from R_top={R_top/1000:.0f}k, R_bottom={R_bottom/1000:.0f}k: {c1_theory:.6f}")
print(f"Difference (meas - theory) / theory: {diff_pct:+.2f}%")

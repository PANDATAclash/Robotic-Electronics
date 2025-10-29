# V_CS [mV] vs I [mA] — based on the tables in Q16

import matplotlib.pyplot as plt

# --- Data from the prompt ---
# Positive currents (A) and V_CS (V)
I_pos_A = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.06]
Vcs_pos_V = [1.672, 1.8, 1.93, 2.06, 2.19, 2.32, 2.45, 2.55]

# Negative currents (A) and V_CS (V)
I_neg_A = [-0.3, -0.6, -0.9, -1.2, -1.5, -1.8, -2.06]
Vcs_neg_V = [1.544, 1.417, 1.283, 1.153, 1.024, 0.89, 0.78]

# --- Convert units to match assignment (mA, mV) ---
I_pos_mA = [i * 1000 for i in I_pos_A]
I_neg_mA = [i * 1000 for i in I_neg_A]
Vcs_pos_mV = [v * 1000 for v in Vcs_pos_V]
Vcs_neg_mV = [v * 1000 for v in Vcs_neg_V]

# --- Plot ---
plt.figure()
plt.plot(I_neg_mA, Vcs_neg_mV, marker='o', label='Negative current')
plt.plot(I_pos_mA, Vcs_pos_mV, marker='o', label='Positive current')

plt.grid(True)
plt.xlabel('Current I [mA]')
plt.ylabel('Voltage V_CS [mV]')
plt.title('V_CS vs Current')
plt.legend()
plt.tight_layout()
plt.show()

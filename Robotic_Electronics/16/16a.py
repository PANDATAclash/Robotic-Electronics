# Plot VCS [mV] vs current [mA] for positive and negative current datasets.
import matplotlib.pyplot as plt

# Given data from the tables (currents in A, Vcs in V)
curr_A = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.06]

vcs_pos_V = [1.672, 1.8, 1.93, 2.06, 2.19, 2.32, 2.45, 2.55]   # positive current sweep
vcs_neg_V = [1.672, 1.544, 1.417, 1.283, 1.153, 1.024, 0.89, 0.78]  # negative current sweep

# Convert to requested units
curr_mA = [x * 1000 for x in curr_A]     # A -> mA
vcs_pos_mV = [x * 1000 for x in vcs_pos_V]  # V -> mV
vcs_neg_mV = [x * 1000 for x in vcs_neg_V]  # V -> mV

# Create plot
plt.figure()
plt.plot(curr_mA, vcs_pos_mV, marker='o', label='Positive current')
plt.plot(curr_mA, vcs_neg_mV, marker='s', label='Negative current')
plt.xlabel('Current [mA]')
plt.ylabel('VCS [mV]')
plt.title('VCS vs Current')
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.show()



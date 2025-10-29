import matplotlib.pyplot as plt

# Data from the photo — LOADED situation
vin = [5.86, 6.86, 7.86, 8.86, 9.86, 10.86, 11.86, 12.86, 13.86, 14.86]  # Input voltage [V]
vj5 = [4.976, 4.978, 4.978, 4.978, 4.982, 4.98, 4.978, 4.979, 4.98, 4.982]  # J5V voltage [V]
iin_mA = [540, 467, 412, 369, 334, 305, 280, 260, 244, 228]  # Supply current (loaded) [mA] — from photo

# --- Efficiency ---

iin = [x/1000 for x in iin_mA]  # A
iout = [v/10.0 for v in vj5]    # A, from 10Ω load
efficiency = [(v*io)/(vi*ii) if (vi*ii)!=0 else 0.0 for v, io, vi, ii in zip(vj5, iout, vin, iin)]

# Plot J5V voltage vs input voltage
plt.figure()
plt.plot(vin, vj5, marker='o')
plt.xlabel('Input voltage Vin [V]')
plt.ylabel('Voltage on J5V [V]')
plt.title('J5V Voltage vs Input Voltage (Loaded, 10Ω)')
plt.grid(True)
plt.show()

# Plot Efficiency vs input voltage
plt.figure()
plt.plot(vin, [e*100 for e in efficiency], marker='o')
plt.xlabel('Input voltage Vin [V]')
plt.ylabel('Efficiency [%]')
plt.title('5V Supply Efficiency vs Vin (Loaded, 10Ω)')
plt.grid(True)
plt.show()

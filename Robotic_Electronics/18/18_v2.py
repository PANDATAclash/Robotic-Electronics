import numpy as np

# === Your measured table ===
# Half bridge 1 (pwm2 = 0) — positive direction
duty_p = np.array([0, 100, 200, 300, 400, 500, 600, 700, 800, 900])
I_p    = np.array([0, 0.109, 0.130, 0.137, 0.160, 0.167, 0.188, 0.195, 0.195, 0.197])
f_p    = np.array([0, 251.6, 525.8, 775.8, 1002, 1434.7, 1666, 1838.2, 1930, 2105])    # [Hz]

# Half bridge 2 (pwm1 = 0) — negative direction
duty_n = np.array([0, -100, -200, -300, -400, -500, -600, -700, -800, -900])
I_n    = np.array([0.000, -0.102, -0.132, -0.139, -0.139, -0.153, -0.153, -0.176, -0.171, -0.183])
f_n    = np.array([0, -230, -510, -800, -1080, -1380, -1670, -1950, -2200, -2550])

# === Build dataset (drop f=0 points) ===
duty = np.concatenate([duty_p, duty_n])
I    = np.concatenate([I_p,    I_n   ])
f    = np.concatenate([f_p,    f_n   ])
mask = (f != 0)
duty, I, f = duty[mask], I[mask], f[mask]

# === Motor voltage and angular speed ===
Vin = 12.2
U = Vin * (duty / 1023.0)        # Umotor [V]
PPR_MOTOR = 16.0                 # encoder counts per revolution on motor shaft
omega = 2.0 * np.pi * (f / PPR_MOTOR)  # rad/s (signed)

# === Linear regression: y = km + Ra * x, with x = I/omega, y = U/omega ===
x = I / omega
y = U / omega
Ra, km = np.polyfit(x, y, 1)     # returns [slope, intercept]

print(f"Ra = {Ra:.6f} ohm")
print(f"km = {km:.6f} V·s/rad  (== {km:.6f} N·m/A)")

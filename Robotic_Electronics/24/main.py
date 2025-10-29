#Q24
from machine import Pin, PWM, ADC, SoftI2C, UART
from time import ticks_ms, ticks_add, ticks_diff, sleep_ms
from HighLevelController1 import HighLevelController, reg
from TCPHandler import TCPHandler
from MPU6050_studentversion import MPU6050
import Encoder
import time


SSID = "Kurwa"          
PASSWORD = "12345679"   
PORT = 5000

tcp = TCPHandler(SSID, PASSWORD, PORT, led=Pin(2))
hlc = HighLevelController(uart2, tcp)

# Wait for TCP server to be ready (so port 5000 opens)
print("Waiting for TCP server to start...")
tcp_state = 0
for _ in range(150):                 # ~15 s max
    tcp_state = tcp.getState()       # 0: Wi-Fi connecting, 1: listening, 2: client connected
    if tcp_state >= 1:
        break
    time.sleep_ms(100)
print("TCP state at start:", tcp_state)

# ----------- Sensors -----------
mpu = MPU6050(i2c, addr=0x68, up_axis='x', up_sign=+1)
Encoder.InitEncoder(ENC_A, ENC_B)

# ----------- Motor / model constants (regs 13..19) -----------
# Encoder is 16 CPR (cycles/rev) on the MOTOR shaft; gearbox 48:1.
reg[13] = 16       # CPR at motor shaft (match Q18)
reg[14] = 21       # gear ratio ×1000 => (1/48)*1000 ≈ 20.833 -> 21
reg[15] = 250      # rated shaft speed [rpm] (placeholder; update if you have datasheet value)

# Torque model (from your Q18b): km ≈ 10.963 mNm/A, choose Irated=200 mA -> Trated ≈ 2.193 mNm
reg[16] = 2193     # rated torque [mNm]
reg[17] = 200      # rated current [mA]

# From Q18 / Q18b
reg[18] = 160      # no-load current @ ~½ speed [mA]
reg[19] = 1460     # armature resistance [mΩ] (1.460445 Ω)

# Tilt thresholds (modifiable over HLC)
reg[20] = 15       # warn [°]
reg[21] = 30       # error [°]


# VS path: ~5:1 divider to 0..3.0 V range
VS_MV_PER_COUNT = (3000 * 5) / 4095.0  # ≈ 3.663 mV/count at battery

# Current sensor linear fit from ±I tables:
# I[mA] = I0_MA + C2_MA_PER_MV * Vcs[mV]
I0_MA = -3896.0
C2_MA_PER_MV = 2.33

def read_battery_mV():
    return int(VS_ADC.read() * VS_MV_PER_COUNT)

def read_current_mA():
    vcs_mV = (CS_ADC.read() * 3000.0) / 4095.0
    return int(round(I0_MA + C2_MA_PER_MV * vcs_mV))


def apply_pwm_from_reg(pwm_obj, cmd_0_1023):
    """Accept 0..1023 in reg and apply to PWM (works on duty_u16 or duty)."""
    if cmd_0_1023 < 0: cmd_0_1023 = 0
    if cmd_0_1023 > 1023: cmd_0_1023 = 1023
    if hasattr(pwm_obj, "duty_u16"):
        pwm_obj.duty_u16(int((cmd_0_1023 / 1023.0) * 65535))
    else:
        pwm_obj.duty(cmd_0_1023)

def tilt_protection():
    """LEDs + buzzer + EN control based on tilt thresholds; publish acc/tilt regs."""
    tilt = mpu.tiltAngle()
    ax, ay, az = mpu.acc  # m/s^2

    # publish to registers
    reg[3] = int(round(ax * 1000))  # mm/s^2
    reg[4] = int(round(ay * 1000))
    reg[5] = int(round(az * 1000))
    reg[6] = int(round(tilt))       # deg

    warn = reg[20]
    err  = reg[21]

    # LEDs
    if warn == 0:
        LED_G.value(0); LED_R.value(0)
    else:
        if tilt < warn:
            LED_G.value(1); LED_R.value(0)
        else:
            LED_G.value(0); LED_R.value(1)

    # Error: buzzer + disable H-bridge
    if err and tilt >= err:
        buzzer.duty_u16(20000)  # on
        EN.value(0)             # disable power stage
    else:
        buzzer.duty_u16(0)
        EN.value(1)             # allowed

def compute_speed_torque(current_mA, freq_Hz):
    """
    freq_Hz: signed cycles/sec from a 16-CPR encoder on the MOTOR shaft (like Q18)
    motor_rpm = (freq * 60) / 16
    shaft_rpm = motor_rpm * (reg[14]/1000)  ~ divide by 48
    """
    motor_rpm = (freq_Hz * 60.0) / 16.0
    shaft_rpm = motor_rpm * (reg[14] / 1000.0)

    # no-load current sign follows direction
    I0 = reg[18] if shaft_rpm >= 0 else -reg[18]
    kT = (reg[16] / float(reg[17])) if reg[17] else 0.0  # mNm/mA
    Tshaft_mNm = kT * (current_mA - I0)

    return int(round(shaft_rpm)), int(round(Tshaft_mNm))


PERIOD_MS = 100
reg[22] = 0   # pwm1 command 0..1023
reg[23] = 0   # pwm2 command 0..1023

print("Q24 starting...")

next_t = ticks_add(ticks_ms(), PERIOD_MS)
while True:
    # Keep TCP state machine alive (listening / accept clients)
    tcp.getState()

    # sensors → regs[0..2]
    reg[0] = read_battery_mV()          # mV
    reg[1] = read_current_mA()          # mA
    freq = Encoder.GetFrequency()       # Hz (signed; same as Q18)
    reg[2] = int(freq)

    # tilt protection (also sets EN/buzzer), publish acc/tilt regs
    tilt_protection()

    # derive speed & torque → regs[7..8]
    rpm_shaft, Tshaft = compute_speed_torque(reg[1], freq)
    reg[7] = rpm_shaft
    reg[8] = Tshaft

    # serve protocols
    hlc.readFromTCP()   # TCP client (KiTTY)
    hlc.readFromUart()  # UART2 client, if used

    # apply PWM commands from reg[22]/[23] (0..1023)
    apply_pwm_from_reg(PWM1, reg[22])
    apply_pwm_from_reg(PWM2, reg[23])

    # exact 100 ms cadence
    now = ticks_ms()
    dt = ticks_diff(next_t, now)
    if dt > 0:
        sleep_ms(dt)
        next_t = ticks_add(next_t, PERIOD_MS)
    else:
        next_t = ticks_add(now, PERIOD_MS)

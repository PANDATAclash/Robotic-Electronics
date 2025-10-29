from boot import *  # pins, PWM1/PWM2, EN, LEDs, ADCs, uart2, i2c, SW1/SW2/SW3, etc.

from machine import Pin
from time import ticks_ms, ticks_add, ticks_diff, sleep_ms
from HighLevelController1 import HighLevelController, reg
from TCPHandler import TCPHandler
from MPU6050_studentversion import MPU6050
import Encoder
import time

# ---------------- Network / protocol ----------------
SSID = "Kurwa"
PASSWORD = "12345679"
PORT = 5000

tcp = TCPHandler(SSID, PASSWORD, PORT, led=Pin(2))
hlc = HighLevelController(uart2, tcp)

print("Waiting for TCP server to start.")
for _ in range(150):                 # about 15 seconds
    if tcp.getState() >= 1:          # 0: Wi-Fi, 1: listening, 2: client connected
        break
    time.sleep_ms(100)

# ---------------- Sensors ----------------
mpu = MPU6050(i2c, addr=0x68, up_axis='x', up_sign=+1)
Encoder.InitEncoder(ENC_A, ENC_B)    # returns signed hertz on GetFrequency()

# ---------------- Motor / model constants ----------------
reg[13] = 16      # cycles per motor revolution at motor
reg[14] = 23      # gear ratio ×1000 (= 1/43.7)
reg[15] = 250     # rated shaft speed [rpm]
reg[16] = 318     # rated torque [mN·m]
reg[17] = 2900    # rated current [mA]
reg[18] = 100     # no-load current [mA]
reg[19] = 1460    # armature resistance [mΩ] (1.460 Ω)

# ---------------- Tilt thresholds ----------------
reg[20], reg[21] = 30, 45

# ---------------- ADC calibration ----------------
VS_MV_PER_COUNT = (3300 * 5) / 4095.0    # battery divider about five to one

# Current sensor calibration
C2_MA_PER_MV = 2.33
CURR_SIGN     = +1
CURR_VREF_MV  = None
current_filt  = 0.0

def calibrate_current_zero(samples=64):
    global CURR_VREF_MV
    acc = 0
    for _ in range(samples):
        acc += CS_ADC.read()
        time.sleep_ms(5)
    CURR_VREF_MV = (acc / samples) * (3300.0 / 4095.0)

def read_battery_mV():
    return int(VS_ADC.read() * VS_MV_PER_COUNT)

def read_current_mA():
    """Read current sensor and filter."""
    global CURR_VREF_MV, current_filt
    if CURR_VREF_MV is None:
        calibrate_current_zero()
    vcs_mV = (CS_ADC.read() * 3300.0) / 4095.0
    i_now = CURR_SIGN * C2_MA_PER_MV * (vcs_mV - CURR_VREF_MV)
    current_filt = 0.9 * current_filt + 0.1 * i_now
    return int(round(current_filt))

# ---------------- Helpers ----------------
def apply_pwm_from_reg(pwm_obj, cmd_0_1023):
    cmd_0_1023 = max(0, min(1023, cmd_0_1023))
    if hasattr(pwm_obj, "duty_u16"):
        pwm_obj.duty_u16(int((cmd_0_1023 / 1023.0) * 65535))
    else:
        pwm_obj.duty(cmd_0_1023)

def set_duty_fraction(duty):
    duty = max(-0.98, min(0.98, duty))
    if duty >= 0:
        reg[23] = 0
        reg[22] = int(round(duty * 1023))
    else:
        reg[22] = 0
        reg[23] = int(round((-duty) * 1023))

def get_duty_fraction():
    if reg[22] > 0 and reg[23] == 0:
        return min(0.98, reg[22] / 1023.0)
    if reg[23] > 0 and reg[22] == 0:
        return -min(0.98, reg[23] / 1023.0)
    return 0.0

# --- Tilt protection ---
tilt_in_fault = False

def tilt_protection():
    """Read tilt, drive LEDs, buzzer, and disable on fault."""
    global tilt_in_fault

    tilt = mpu.tiltAngle()
    ax, ay, az = mpu.acc
    reg[3] = int(round(ax * 1000))
    reg[4] = int(round(ay * 1000))
    reg[5] = int(round(az * 1000))
    reg[6] = int(round(tilt))

    warn, err = reg[20], reg[21]

    if warn == 0:
        LED_G.value(0); LED_R.value(0)
    else:
        if tilt < warn: LED_G.value(1); LED_R.value(0)
        else:           LED_G.value(0); LED_R.value(1)

    # Fault latch with hysteresis
    if err:
        if tilt_in_fault:
            if tilt < (err - 2):
                tilt_in_fault = False
        else:
            if tilt >= err:
                tilt_in_fault = True
    else:
        tilt_in_fault = False

    if tilt_in_fault:
        buzzer.duty_u16(20000)
        EN.value(0)
    else:
        buzzer.duty_u16(0)

    return tilt_in_fault

def cycles_to_rpm_shaft(cycles_hz):
    cpr = reg[13] if reg[13] else 16
    motor_rpm = (cycles_hz * 60.0) / float(cpr)
    return int(round(motor_rpm * (reg[14] / 1000.0)))

def compute_torque_mNm(current_mA, rpm_shaft):
    """Compute signed shaft torque."""
    kT = (reg[16] / float(reg[17])) if reg[17] else 0.0
    s_omega = 1 if rpm_shaft >= 0 else -1
    I0 = s_omega * reg[18]
    tau = kT * (current_mA - I0)
    tau_signed = s_omega * tau
    if abs(tau_signed) < 5:
        return 0
    return int(round(tau_signed))

# ---------------- Q25 controller ----------------
MODE_OFF  = 0
MODE_TEST = 1
MODE_CTRL = 2

reg[9]  = MODE_OFF
reg[10] = 50
reg[11] = reg[16]
reg[26] = 1

TS = 0.1
KI = 0.016
RPM_ALPHA = 0.2
rpm_filt = 0.0
duty_int = 0.0

def i_controller_step(rpm_set, rpm_meas, vin_mV, i_meas_mA, enabled=True):
    global duty_int

    if not enabled:
        duty_int *= 0.9
        if rpm_set >= 0 and duty_int < 0: duty_int = 0.0
        if rpm_set <  0 and duty_int > 0: duty_int = 0.0
        return duty_int

    error = rpm_set - rpm_meas
    delta_dc = KI * error * TS

    kT = (reg[16] / float(reg[17])) if reg[17] else 1e-6
    I0 = reg[18] if rpm_set >= 0 else -reg[18]
    Imax = (reg[11] / kT) + I0
    Ra  = reg[19] / 1000.0
    Uin = max(1.0, vin_mV / 1000.0)

    if rpm_set >= 0:
        delta_dc = min(delta_dc, (Ra * (Imax - i_meas_mA)) / Uin)
    else:
        delta_dc = max(delta_dc, -(Ra * (Imax + i_meas_mA)) / Uin)

    duty_int += delta_dc

    if rpm_set >= 0:
        duty_int = min(0.98, max(0.0, duty_int))
    else:
        duty_int = max(-0.98, min(0.0, duty_int))

    return duty_int

# -------------- Encoder sign stabilization --------------
last_sign = +1
sign_streak = 0
RPM_SIGN_MIN = 20
SIGN_HOLD_SAMPLES = 3

# -------------- Buttons (use SW1/SW2/SW3 from boot.py) --------------
# Active-low convention: 1 = released, 0 = pressed
prev_sw1 = SW1.value()
prev_sw2 = SW2.value()
prev_sw3 = SW3.value()

DEBOUNCE_MS = 30
last_sw1_time = 0
last_sw2_time = 0
last_sw3_time = 0

def _pressed(prev, now):
    return (prev == 1) and (now == 0)   # falling edge on active-low

last_active_mode = MODE_TEST  # remembered for SW1 toggle

# ---------------- Main loop ----------------
PERIOD_MS = 100
reg[22] = 0
reg[23] = 0

calibrate_current_zero()
print("Q25 starting.")
next_t = ticks_add(ticks_ms(), PERIOD_MS)

while True:
    tcp.getState()

    # Sensors
    reg[0] = read_battery_mV()

    CURR_SIGN = 1 if reg[26] >= 0 else -1
    reg[1] = read_current_mA()

    freq_hz = Encoder.GetFrequency()
    reg[2] = int(freq_hz)

    tilt_fault = tilt_protection()

    rpm_mag = cycles_to_rpm_shaft(abs(freq_hz))
    cand_sign = +1 if freq_hz >= 0 else -1

    if rpm_mag >= RPM_SIGN_MIN and cand_sign != last_sign:
        sign_streak += 1
        if sign_streak >= SIGN_HOLD_SAMPLES:
            last_sign = cand_sign
            sign_streak = 0
    else:
        sign_streak = 0

    rpm_signed = rpm_mag if last_sign > 0 else -rpm_mag
    reg[7] = int(rpm_signed)
    reg[8] = compute_torque_mNm(reg[1], reg[7])

    rpm_filt += RPM_ALPHA * (reg[7] - rpm_filt)

    # ---------------- Button handling (debounced) ----------------
    now_ms = ticks_ms()

    sw1 = SW1.value()
    sw2 = SW2.value()
    sw3 = SW3.value()

    # SW1: toggle OFF <-> last_active_mode (if no tilt fault)
    if _pressed(prev_sw1, sw1) and ticks_diff(now_ms, last_sw1_time) > DEBOUNCE_MS:
        last_sw1_time = now_ms
        if reg[9] == MODE_OFF and not tilt_fault:
            reg[9] = last_active_mode
        else:
            if reg[9] != MODE_OFF:
                last_active_mode = reg[9]
            reg[9] = MODE_OFF
            reg[22] = 0; reg[23] = 0
            EN.value(0)

    # SW2: speed up / bias forward
    if _pressed(prev_sw2, sw2) and ticks_diff(now_ms, last_sw2_time) > DEBOUNCE_MS:
        last_sw2_time = now_ms
        if reg[9] == MODE_TEST:
            if reg[23] == 0:
                reg[22] = min(970, reg[22] + 50)
            else:
                reg[23] = max(0, reg[23] - 50)
        elif reg[9] == MODE_CTRL:
            step = max(1, int(round(0.05 * reg[15])))   # +5% of rated
            max_sp = int(round(1.20 * reg[15]))         # +120% cap
            reg[10] = min(max_sp, reg[10] + step)

    # SW3: speed down / bias reverse
    if _pressed(prev_sw3, sw3) and ticks_diff(now_ms, last_sw3_time) > DEBOUNCE_MS:
        last_sw3_time = now_ms
        if reg[9] == MODE_TEST:
            if reg[22] == 0:
                reg[23] = min(970, reg[23] + 50)
            else:
                reg[22] = max(0, reg[22] - 50)
        elif reg[9] == MODE_CTRL:
            step = max(1, int(round(0.05 * reg[15])))   # -5% of rated
            min_sp = -int(round(1.20 * reg[15]))        # -120% cap
            reg[10] = max(min_sp, reg[10] - step)

    prev_sw1, prev_sw2, prev_sw3 = sw1, sw2, sw3
    # -------------------------------------------------------------

    # Commands from host
    hlc.readFromTCP()
    hlc.readFromUart()

    # Modes
    m = reg[9]
    if m == MODE_OFF:
        duty_int = 0.0
        reg[22] = 0
        reg[23] = 0
        EN.value(0)

    elif m == MODE_TEST:
        if reg[22] > 0: reg[23] = 0
        if reg[23] > 0: reg[22] = 0
        duty_int = get_duty_fraction()
        EN.value(0 if tilt_fault else 1)

    elif m == MODE_CTRL:
        duty = i_controller_step(
            rpm_set   = reg[10],
            rpm_meas  = rpm_filt,
            vin_mV    = reg[0],
            i_meas_mA = reg[1],
            enabled   = (not tilt_fault)
        )
        set_duty_fraction(duty)
        EN.value(0 if tilt_fault else 1)

    else:
        duty_int = 0.0
        reg[22] = 0
        reg[23] = 0
        EN.value(0)

    apply_pwm_from_reg(PWM1, reg[22])
    apply_pwm_from_reg(PWM2, reg[23])

    now = ticks_ms()
    dt = ticks_diff(next_t, now)
    if dt > 0:
        sleep_ms(dt)
        next_t = ticks_add(next_t, PERIOD_MS)
    else:
        next_t = ticks_add(now, PERIOD_MS)

from boot import *  # pins, PWM1/PWM2, EN, LEDs, ADCs, uart2, i2c, etc.

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
reg[14] = 23      # gear ratio times one thousand (= 1/43.7)
reg[15] = 250     # rated shaft speed [rpm]
reg[16] = 318     # rated torque [mN·m]
reg[17] = 2900    # rated current [mA]
reg[18] = 100     # no-load current near half speed [mA]
reg[19] = 1460    # armature resistance [mΩ] (1.460 Ω)

# ---------------- Tilt thresholds ----------------
reg[20], reg[21] = 15, 30

# ---------------- ADC calibration ----------------
VS_MV_PER_COUNT = (3300 * 5) / 4095.0    # battery divider about five to one

# Current sensor calibration (auto-zero + selectable polarity)
C2_MA_PER_MV = 2.33       # slope magnitude
CURR_SIGN     = +1        # set each loop from reg[26]
CURR_VREF_MV  = None      # learned zero (idle) voltage
current_filt  = 0.0       # filtered current (mA) for torque

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
    """Read raw sensor, convert to milliampere, then apply a simple low-pass filter."""
    global CURR_VREF_MV, current_filt
    if CURR_VREF_MV is None:
        calibrate_current_zero()
    vcs_mV = (CS_ADC.read() * 3300.0) / 4095.0
    i_now = CURR_SIGN * C2_MA_PER_MV * (vcs_mV - CURR_VREF_MV)
    # Low-pass: 90 percent previous + 10 percent new
    current_filt = 0.9 * current_filt + 0.1 * i_now
    return int(round(current_filt))

# ---------------- Helpers ----------------
def apply_pwm_from_reg(pwm_obj, cmd_0_1023):
    if cmd_0_1023 < 0: cmd_0_1023 = 0
    if cmd_0_1023 > 1023: cmd_0_1023 = 1023
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

# --- Tilt protection with small hysteresis; only DISABLES the bridge
tilt_in_fault = False  # latched fault with hysteresis

def tilt_protection():
    """Update tilt registers, light indicators, and buzzer.
       Return True when fault (bridge must be disabled), False otherwise.
       This function never enables the bridge; it may only force-disable it."""
    global tilt_in_fault

    tilt = mpu.tiltAngle()
    ax, ay, az = mpu.acc
    reg[3] = int(round(ax * 1000))
    reg[4] = int(round(ay * 1000))
    reg[5] = int(round(az * 1000))
    reg[6] = int(round(tilt))

    warn, err = reg[20], reg[21]

    # Warning indicators
    if warn == 0:
        LED_G.value(0); LED_R.value(0)
    else:
        if tilt < warn: LED_G.value(1); LED_R.value(0)
        else:           LED_G.value(0); LED_R.value(1)

    # Fault latch with two degree hysteresis
    if err:
        if tilt_in_fault:
            if tilt < (err - 2):
                tilt_in_fault = False
        else:
            if tilt >= err:
                tilt_in_fault = True
    else:
        tilt_in_fault = False

    # Buzzer and enforced disable on fault
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
    """
    Return SIGNED shaft torque.

    Convention used here:
      - Positive torque when it assists the current direction of rotation.
      - Negative torque when it opposes the rotation (braking).
    Implementation:
      τ = kT * (I - sgn(ω)*I0), then we align sign with rotation: τ_signed = sgn(ω) * τ
    """
    # motor torque constant in mN·m per mA
    kT = (reg[16] / float(reg[17])) if reg[17] else 0.0

    # sign of rotation (+1 for ≥0 rpm, -1 for <0 rpm)
    s_omega = 1 if rpm_shaft >= 0 else -1

    # direction-aware no-load current
    I0 = s_omega * reg[18]

    # raw torque (can be positive or negative)
    tau = kT * (current_mA - I0)

    # torque signed with rotation direction
    tau_signed = s_omega * tau

    # small symmetric deadband to kill tiny noise around zero
    if abs(tau_signed) < 5:
        return 0

    return int(round(tau_signed))


# ---------------- Q25 controller ----------------
MODE_OFF  = 0
MODE_TEST = 1
MODE_CTRL = 2

# defaults
reg[9]  = MODE_OFF     # start in off mode
reg[10] = 50           # speed setpoint [rpm]
reg[11] = reg[16]      # torque limit [mN·m] (start at rated torque)
reg[26] = 1            # current polarity control: >0 => positive, <0 => negative

TS = 0.1               # loop period [s]
KI = 0.016             # integral gain tuned for the required ramp
RPM_ALPHA = 0.2        # speed low-pass weight
rpm_filt = 0.0
duty_int = 0.0



def i_controller_step(rpm_set, rpm_meas, vin_mV, i_meas_mA, enabled=True):
    """
    Integral-only controller with current-based duty limiting.
    Direction guard: duty sign follows setpoint sign; never crosses zero.
    """
    global duty_int

    # When disabled, bleed integrator and keep sign guard
    if not enabled:
        duty_int *= 0.9
        # guard against tiny sign creep
        if rpm_set >= 0 and duty_int < 0: duty_int = 0.0
        if rpm_set <  0 and duty_int > 0: duty_int = 0.0
        return duty_int

    # 1) Integral step
    error = rpm_set - rpm_meas
    delta_dc = KI * error * TS

    # 2) Current/torque limiter (caps the permitted delta toward limit)
    kT = (reg[16] / float(reg[17])) if reg[17] else 1e-6
    I0 = reg[18] if rpm_set >= 0 else -reg[18]
    Imax = (reg[11] / kT) + I0                 # [mA]
    Ra  = reg[19] / 1000.0                     # [Ohm]
    Uin = max(1.0, vin_mV / 1000.0)            # [V]

    if rpm_set >= 0:
        # When over limit (i_meas > Imax), limiter allows negative delta to back off,
        # but the direction guard (below) will stop at zero.
        delta_dc = min(delta_dc, (Ra * (Imax - i_meas_mA)) / Uin)
    else:
        delta_dc = max(delta_dc, -(Ra * (Imax + i_meas_mA)) / Uin)

    # 3) Accumulate
    duty_int += delta_dc

    # 4) Direction guard + anti-windup at bound
    if rpm_set >= 0:
        if duty_int < 0.0:
            duty_int = 0.0        # do not cross into negative
        if duty_int > 0.98:
            duty_int = 0.98       # top clamp
    else:
        if duty_int > 0.0:
            duty_int = 0.0        # do not cross into positive
        if duty_int < -0.98:
            duty_int = -0.98      # bottom clamp

    return duty_int


# ---------------- Telemetry (UART2) ----------------
reg[27] = 1                 # 1 = telemetry on, 0 = off
TELEMETRY_EVERY_N = 5       # one line every 5 loops (~2 Hz at 100 ms)
telemetry_counter = 0

def send_telemetry(timestamp_ms):
    """CSV to uart2: tel,<seconds>,<rpm>,<mN·m>,<mA>"""
    try:
        s = "tel,{:.3f},{},{},{}\n".format(
            timestamp_ms / 1000.0,
            int(reg[7]),   # speed (rpm)
            int(reg[8]),   # torque (mN·m)
            int(reg[1])    # current (mA)  <-- added
        )
        uart2.write(s)
    except Exception:
        pass


# -------------- Encoder sign stabilization --------------
last_sign = +1
sign_streak = 0
RPM_SIGN_MIN = 20        # do not accept sign changes below this speed (rpm)
SIGN_HOLD_SAMPLES = 3    # need this many consecutive samples to flip sign

# ---------------- Main loop ----------------
PERIOD_MS = 100
reg[22] = 0
reg[23] = 0

# Ensure auto-zero happens at rest
calibrate_current_zero()

print("Q25 starting.")
next_t = ticks_add(ticks_ms(), PERIOD_MS)

while True:
    tcp.getState()  # keep server alive

    # Sensors -> registers
    reg[0] = read_battery_mV()

    # current polarity from reg[26]
    CURR_SIGN = 1 if reg[26] >= 0 else -1
    reg[1] = read_current_mA()

    # Encoder frequency (signed hertz)
    freq_hz = Encoder.GetFrequency()
    reg[2] = int(freq_hz)  # signed frequency

    # Tilt protection (may force-disable)
    tilt_fault = tilt_protection()

    # ---------- Derived signals with sign stabilization ----------
    rpm_mag = cycles_to_rpm_shaft(abs(freq_hz))  # non-negative magnitude
    cand_sign = +1 if freq_hz >= 0 else -1

    # Only allow sign change if speed is clearly nonzero and sustained
    if rpm_mag >= RPM_SIGN_MIN and cand_sign != last_sign:
        sign_streak += 1
        if sign_streak >= SIGN_HOLD_SAMPLES:
            last_sign = cand_sign
            sign_streak = 0
    else:
        sign_streak = 0

    rpm_signed = rpm_mag if last_sign > 0 else -rpm_mag

    reg[7] = int(rpm_signed)                         # signed shaft rpm
    reg[8] = compute_torque_mNm(reg[1], reg[7])      # uses filtered current
    # ------------------------------------------------------------

    # Telemetry at limited rate
    if reg[27]:
        telemetry_counter = (telemetry_counter + 1) % TELEMETRY_EVERY_N
    if telemetry_counter == 0:
        send_telemetry(ticks_ms())


    # Filtered rpm for controller stability (on signed rpm)
    rpm_filt += RPM_ALPHA * (reg[7] - rpm_filt)

    # Commands from host
    hlc.readFromTCP()
    hlc.readFromUart()

    # -------- Modes --------
    m = reg[9]
    if m == MODE_OFF:
        duty_int = 0.0
        reg[22] = 0; reg[23] = 0
        EN.value(0)   # keep disabled in OFF

    elif m == MODE_TEST:
        # manual pulse width, ensure exclusivity
        if reg[22] > 0: reg[23] = 0
        if reg[23] > 0: reg[22] = 0
        duty_int = get_duty_fraction()  # track for smooth switching
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
        # unknown mode -> safe
        duty_int = 0.0
        reg[22] = 0; reg[23] = 0
        EN.value(0)

    # Apply to hardware
    apply_pwm_from_reg(PWM1, reg[22])
    apply_pwm_from_reg(PWM2, reg[23])

    # 100 ms cadence
    now = ticks_ms()
    dt = ticks_diff(next_t, now)
    if dt > 0:
        sleep_ms(dt); next_t = ticks_add(next_t, PERIOD_MS)
    else:
        next_t = ticks_add(now, PERIOD_MS)

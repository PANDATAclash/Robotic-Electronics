from micropython import const
from machine import mem32
import array

PERIP_CLK_EN_REG = const(0x3FF000C0)
PERIP_RST_EN_REG = const(0x3FF000C4)
PWM0_CAP_TIMER_CFG_REG = const(0x3FF5E0E8)
PWM0_CAP_TIMER_PHASE_REG = const(0x3FF5E0EC)
PWM0_CAP_CH0_CFG_REG = const(0x3FF5E0F0)
PWM0_CAP_CH1_CFG_REG = const(0x3FF5E0F4)
PWM0_CAP_CH0_REG = const(0x3FF5E0FC)
PWM0_CAP_CH1_REG = const(0x3FF5E100)
PWM1_CAP_TIMER_CFG_REG = const(0x3FF6C0E8)
PWM1_CAP_TIMER_PHASE_REG = const(0x3FF6C0EC)
PWM1_CAP_CH0_CFG_REG = const(0x3FF6C0F0)
PWM1_CAP_CH1_CFG_REG = const(0x3FF6C0F4)
PWM1_CAP_CH0_REG = const(0x3FF6C0FC)
PWM1_CAP_CH1_REG = const(0x3FF6C100)
GPIO_FUNC31_IN_SEL_CFG_REG = const(0x3FF441AC)	# function 31:  pwm0_sync0_in
GPIO_FUNC109_IN_SEL_CFG_REG = const(0x3FF442E4)	# function 109: pwm0_cap0_in
GPIO_FUNC103_IN_SEL_CFG_REG = const(0x3FF442CC)	# function 103: pwm1_sync0_in
GPIO_FUNC112_IN_SEL_CFG_REG = const(0x3FF442F0)	# function 112: pwm1_cap0_in

singleChannel = False	# True when a single channel encoder is used, False is a quadrature encoder is used

"""
The function GetIOMuxAddress(gpio) returns the IOMuxaddress of a pin (page 60-61 datasheets ESP32)
Argument gpio: gpio pin number (integer)
"""
iomux = array.array('B', [0x44,0x88,0x40,0x84,0x48,0x6C,0x60,0x64,0x68,0x54,0x58,0x5C,0x34,0x38,0x30,0x3C,0x4C,0x50,0x70,0x74,0x78,0x7C,0x80,0x8C,0x90,0x24,0x28,0x2C,0x00,0x00,0x00,0x00,0x1C,0x20,0x14,0x18,0x04,0x08,0x0C,0x10]);
def GetIOMuxAddress(gpio):
    if (gpio < 0) or (gpio > 39):
        return 0
    return 0 if iomux[gpio] == 0 else 0x3FF49000 + iomux[gpio]


"""
The function InitEncoder(gpio1, gpio2) initializes 2 timer/capture modules
Arguments gpio1: gpio pin number of encoder A input (integer)
Arguments gpio2: gpio pin number of encoder B input (input)
For single channel encoders, gpio1 and gpio2 must be the same
"""
def InitEncoder(gpio1, gpio2):
    global singleChannel
    iomux1 = GetIOMuxAddress(gpio1)
    iomux2 = GetIOMuxAddress(gpio2)
    if (iomux1 == 0) or (iomux2 == 0):
        print("Illegal PIN number")
        return
    singleChannel = gpio1 == gpio2
    mem32[PERIP_CLK_EN_REG] |=  (9 << 17)	# Enable peripheral clock for MCPWM0 and MCPWM1 (bit 17 and 20, page 93)
    mem32[PERIP_RST_EN_REG] &=  ~(9 << 17)	# Disable reset state of peripheral clock (see page 93)
    mem32[GPIO_FUNC31_IN_SEL_CFG_REG] = 0x80 + gpio1	# Positive edge of gpio1 is pwm0_sync0_in source (page 53)
    mem32[GPIO_FUNC109_IN_SEL_CFG_REG] = 0x80 + gpio2	# Negative edge of gpio2 is pwm0_cap0_in source (page 54)
    mem32[GPIO_FUNC103_IN_SEL_CFG_REG] = 0xC0 + gpio2	# Negative edge of gpio2 is pwm1_sync0_in source (page 54)
    mem32[GPIO_FUNC112_IN_SEL_CFG_REG] = 0xC0 + gpio1	# Positive edge of gpio1 is pwm1_cap0_in source (page 54)
    mem32[iomux1] = 0x2A00							# FUN_IE=1, FUN_DRV=2 (default value), MCU_SEL=2 (Function = 2 (GPIO)) (page 71)
    mem32[iomux2] = 0x2A00							# FUN_IE=1, FUN_DRV=2 (default value), MCU_SEL=2 (Function = 2 (GPIO)) (page 71)
    mem32[PWM0_CAP_TIMER_CFG_REG] = 3 + (4 << 2)	# Capture timer runs on APB_clk, PWM_CAP_SYNCI_SEL is enabled, PWM_CAP_SYNCI_SEL=SYNC0 from GPIO matrix (page 477)
    mem32[PWM1_CAP_TIMER_CFG_REG] = 3 + (4 << 2)	# Capture timer runs on APB_clk, PWM_CAP_SYNCI_SEL is enabled, PWM_CAP_SYNCI_SEL=SYNC0 from GPIO matrix (page 477)
    mem32[PWM0_CAP_CH0_CFG_REG] = 3					# Capture on negative edge, enable channel 0 (page 478)
    mem32[PWM1_CAP_CH0_CFG_REG] = 3					# Capture on negative edge, enable channel 0 (page 478)
    mem32[PWM0_CAP_CH1_CFG_REG] = 1					# Enable capture channel 1 of PWM0 (for reading the actual counter value)
    mem32[PWM1_CAP_CH1_CFG_REG] = 1					# Enable capture channel 1 of PWM1 (for reading the actual counter value)

"""
The function GetFrequency() returns the frequency as an integer
"""
def GetFrequency():
    mem32[PWM0_CAP_CH1_CFG_REG] = 0x1001			# Generate software trigger to capture timer value (see page 478)
    mem32[PWM1_CAP_CH1_CFG_REG] = 0x1001			# Generate software trigger to capture timer value (see page 478)
    cap0 = mem32[PWM0_CAP_CH0_REG]					# Number of 80MHz counts from positive edge of gpio1 to negative edge of gpio2
    cap1 = mem32[PWM1_CAP_CH0_REG]					# Number of 80MHz counts from negative edge of gpio2 to positive edge of gpio2
    tim0 = mem32[PWM0_CAP_CH1_REG]					# tim0 is timer value of PWM0 module
    tim1 = mem32[PWM1_CAP_CH1_REG]					# tim1 is timer value of PWM1 module
    if (tim0 > 160000000) and (cap0 < 160000000):
        mem32[PWM0_CAP_CH0_CFG_REG] = 0x1003		# Software trigger event prevents frequency falling back to the last non-zero value after timer overflow at 0Hz
    cnt = max(tim0, tim1, cap0 + cap1)				# Total number of 80MHz counts of one period
    if (cnt < 2):
        return 0
    f = round(80000000 / (cnt - 2))					# The capture is 1 pulse after the edge
    return f if (cap0 >= cap1) or singleChannel else -f

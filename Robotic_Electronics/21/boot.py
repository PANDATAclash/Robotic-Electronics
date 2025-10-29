from machine import UART, Pin

# =========================
# Pin mapping (shared)
# =========================
# LEDs
PIN_RED    = 19
PIN_GREEN  = 5
PIN_BLUE   = 18

# Buttons
PIN_SW1    = 39  # input-only, external pulls on PCB
PIN_SW2    = 36  # input-only, external pulls on PCB
PIN_SW3    = 15  # supports internal pull-up

# Buzzer
PIN_BUZZER = 4

# UART2 (ESP32 default pins)
PIN_RX2    = 16
PIN_TX2    = 17

# =========================
# UART configuration
# =========================
UART_ID        = 2
UART_BAUDRATE  = 115200
UART_BITS      = 8
UART_PARITY    = None
UART_STOP      = 1
UART_TIMEOUT   = 0   # non-blocking reads; main.py will poll

DEBOUNCE_MS    = 50
LOOP_SLEEP_MS  = 10

BUZZER_FREQ_HZ = 500
BUZZER_DUTY_ON = 512


def init_uart2() -> UART:
    """Create and return a configured non-blocking UART2 instance."""
    return UART(
        UART_ID,
        baudrate=UART_BAUDRATE,
        bits=UART_BITS,
        parity=UART_PARITY,
        stop=UART_STOP,
        rx=PIN_RX2,
        tx=PIN_TX2,
        timeout=UART_TIMEOUT,
    )


def init_pins():
    """
    Put hardware in a safe default state before main.py executes.
    Safe to re-initialize in main.py.
    """
    # LEDs off
    red   = Pin(PIN_RED,   Pin.OUT, value=0)
    green = Pin(PIN_GREEN, Pin.OUT, value=0)
    blue  = Pin(PIN_BLUE,  Pin.OUT, value=0)

    # Buttons (active-low)
    sw1 = Pin(PIN_SW1, Pin.IN)                 # external pull network on PCB
    sw2 = Pin(PIN_SW2, Pin.IN)                 # external pull network on PCB
    sw3 = Pin(PIN_SW3, Pin.IN, Pin.PULL_UP)    # safe internal pull-up

    # Return handles if main.py wants to import them (optional)
    return {
        "red": red, "green": green, "blue": blue,
        "sw1": sw1, "sw2": sw2, "sw3": sw3,
    }


# =========================
# Entry (executed at boot)
# =========================
try:
    uart2 = init_uart2()   # Expose as a global if needed
    pins  = init_pins()    # Optional; safe pre-init
except Exception as e:
    # Avoid halting boot on non-critical init errors
    # (main.py can still attempt its own initialization)
    print("boot.py init warning:", e)

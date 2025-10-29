from machine import UART, Pin, PWM
import time


def now_ms() -> int:
    return time.ticks_ms()


class UartConsole:
    """Minimal wrapper around UART for CRLF lines and non-blocking char reads."""
    def __init__(self, uart: UART):
        self.uart = uart
        self._buf1 = bytearray(1)

    def read_char(self):
        """Return one byte (int 0..255) if available, else None (non-blocking)."""
        n = self.uart.readinto(self._buf1)
        if n:
            return self._buf1[0]
        return None

    def send_line(self, text: str):
        self.uart.write(text + "\r\n")


class DebouncedButton:
    """
    Active-low button with simple debounce.
    Use update(ts) on each loop; when it returns True you got a 'pressed' event (rising edge).
    """
    def __init__(self, pin: Pin, debounce_ms: int):
        self.pin = pin
        self.debounce_ms = debounce_ms
        self._last_raw = self._pressed_raw()
        self._next_ok_ts = 0

    def _pressed_raw(self) -> bool:
        # Active-low: logic 0 means pressed. Adjust here if hardware differs.
        return self.pin.value() == 0

    def update(self, ts_ms: int) -> bool:
        raw = self._pressed_raw()
        if raw != self._last_raw:
            # Edge detected
            if raw and time.ticks_diff(ts_ms, self._next_ok_ts) >= 0:
                # Rising edge into 'pressed'
                self._next_ok_ts = time.ticks_add(ts_ms, self.debounce_ms)
                self._last_raw = raw
                return True
            self._next_ok_ts = time.ticks_add(ts_ms, self.debounce_ms)
            self._last_raw = raw
        return False


class Led:
    def __init__(self, pin_no: int, initial=False):
        self.pin = Pin(pin_no, Pin.OUT, value=1 if initial else 0)
        self.state = bool(initial)

    def set(self, on: bool):
        self.state = bool(on)
        self.pin.value(1 if self.state else 0)

    def toggle(self):
        self.set(not self.state)


class Buzzer:
    def __init__(self, pin_no: int, freq_hz: int, duty_on: int):
        self.pwm = PWM(Pin(pin_no))
        self.freq_hz = freq_hz
        self.duty_on = duty_on
        self._on = False
        self.off()

    def on(self):
        self.pwm.freq(self.freq_hz)
        self.pwm.duty(self.duty_on)
        self._on = True

    def off(self):
        self.pwm.duty(0)
        self._on = False

    def toggle(self):
        if self._on:
            self.off()
        else:
            self.on()



# Application
class App:
    def __init__(self):
        # LEDs
        self.led_R = Led(PIN_RED,   initial=False)
        self.led_G = Led(PIN_GREEN, initial=False)
        self.led_B = Led(PIN_BLUE,  initial=False)

        # Buttons
        # Note: 36/39 are input-only, external pulls; 15 supports internal pull-up.
        self.btn1 = DebouncedButton(Pin(PIN_SW1, Pin.IN),                     DEBOUNCE_MS)
        self.btn2 = DebouncedButton(Pin(PIN_SW2, Pin.IN),                     DEBOUNCE_MS)
        self.btn3 = DebouncedButton(Pin(PIN_SW3, Pin.IN, Pin.PULL_UP),        DEBOUNCE_MS)

        # Buzzer
        self.buzzer = Buzzer(PIN_BUZZER, BUZZER_FREQ_HZ, BUZZER_DUTY_ON)

        # UART
        self.uart = UartConsole(UART(
            UART_ID,
            baudrate=UART_BAUDRATE,
            bits=UART_BITS,
            parity=UART_PARITY,
            stop=UART_STOP,
            rx=PIN_RX2,
            tx=PIN_TX2,
            timeout=UART_TIMEOUT,  # non-blocking
        ))

        # Command map for UART single-char commands
        self._cmd_map = {
            ord('R'): self.led_R.toggle,
            ord('G'): self.led_G.toggle,
            ord('B'): self.led_B.toggle,
            ord('A'): self.buzzer.toggle,
        }

  
    def _handle_uart(self):
        ch = self.uart.read_char()
        if ch is None:
            return
        action = self._cmd_map.get(ch, None)
        if action:
            action()
        # else: ignore unknown chars

    def _handle_buttons(self, ts_ms: int):
        if self.btn1.update(ts_ms):
            self.uart.send_line("switch 1 is pressed")
        if self.btn2.update(ts_ms):
            self.uart.send_line("switch 2 is pressed")
        if self.btn3.update(ts_ms):
            self.uart.send_line("switch 3 is pressed")


    def run(self):
        # Reuse a single loop delay to stay responsive yet lightweight
        while True:
            ts = now_ms()
            self._handle_uart()
            self._handle_buttons(ts)
            time.sleep_ms(LOOP_SLEEP_MS)



# Entry point
if __name__ == "__main__":
    App().run()

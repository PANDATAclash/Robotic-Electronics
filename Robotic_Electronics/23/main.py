from machine import Pin
from time import sleep_ms
from TCPHandler import TCPHandler

# WiFi network credentials
SSID = 'Kurwa'          # <-- your WiFi SSID
PASSWORD = '12345679'   # <-- your WiFi password
PORT = 5000

# LED on Pin 2 turns on when a client connects
tcp = TCPHandler(SSID, PASSWORD, PORT, led=Pin(2))

print("TCP square server started. Waiting for client connection...")

while True:
    state = tcp.getState()

    if state == 0:              # not connected to WiFi
        sleep_ms(100)
        continue
    elif state == 1:            # WiFi connected (no client)
        sleep_ms(100)
        continue
    elif state == 2:            # client connected
        data = tcp.receive()    # <-- correct method name
        if not data:
            sleep_ms(50)
            continue

        # Convert bytes -> str
        try:
            text = data.decode().strip()
        except Exception:
            text = ""

        # Try to parse number and respond
        try:
            value = float(text)
            square = value ** 2
            tcp.transmit(str(square))   # <-- correct method name
        except ValueError:
            tcp.transmit("Error: invalid number")

        sleep_ms(50)

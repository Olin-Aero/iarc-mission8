import urequests
import network
import utime
from machine import Pin, UART
import machine

from config import DRONE_ID

SERVER = "192.168.16.96:8080"

sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect("OLIN-ROBOTICS", "R0B0TS-RULE")


# response = urequests.post("http://jsonplaceholder.typicode.com/posts", data = "some dummy content")

# Pinout for Ultrasonics:
# Sensor | PW pin | RX pin
# Left   |   D1   |   D2
# Center |   D3   |   D4
# Right  |   D5   |   D6
sonics = [
    (Pin(rx, Pin.IN), Pin(tx, Pin.OUT))
    for rx, tx in [
        (5, 4),
        (0, 2),
        # (14, 12),
    ]
]
LED = Pin(16, Pin.OUT)


def get_readings():
    results = []
    for i, (rx, tx) in enumerate(sonics):
        tx.on()
        utime.sleep_us(30)
        tx.off()

        dt = machine.time_pulse_us(rx, 1)

        results.append(dt)

        utime.sleep_us(50000-dt)

    return results


PERIOD = 500  # ms

while True:
    last_time = utime.ticks_ms()
    LED.off()  # Turn on the LED
    values = get_readings()
    try:
        response = urequests.get(
            "http://{}/{};{}".format(
                SERVER, DRONE_ID, ";".join([str(x) for x in values])
            )
        )
        print("requested, {}".format(response.status_code))
        response.close()
    except OSError as e:
        print("Error", e)

    LED.on()  # Turn off the LED
    utime.sleep_ms(max(last_time + PERIOD - utime.ticks_ms(), 0))

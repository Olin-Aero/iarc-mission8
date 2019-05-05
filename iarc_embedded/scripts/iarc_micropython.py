import urequests
import network
import utime
from machine import Pin, UART

from config import DRONE_ID

SERVER = "192.168.16.96:8080"

sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect("OLIN-ROBOTICS", "R0B0TS-RULE")

# response = urequests.post("http://jsonplaceholder.typicode.com/posts", data = "some dummy content")

uart = UART(1)
enables = [Pin(i, Pin.OUT, None) for i in [16,5,4]]

def get_readings():
    if uart.any():
        return uart.read()
    return [1,2,3]


PERIOD = 500  # ms

while True:
    last_time = utime.ticks_ms()
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

    utime.sleep_ms(max(last_time + PERIOD - utime.ticks_ms(), 0))

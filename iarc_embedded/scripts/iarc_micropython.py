import urequests
import network
import utime
import machine

from config import DRONE_ID

SERVER = "192.168.16.96:8080"

sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect("OLIN-ROBOTICS", "R0B0TS-RULE")

# response = urequests.post("http://jsonplaceholder.typicode.com/posts", data = "some dummy content")

PERIOD = 500  # ms

while True:
    last_time = utime.ticks_ms()
    values = [42, 42, 42]
    try:
        response = urequests.get(
            "http://{}/{};{}".format(
                SERVER, DRONE_ID, ";".join([str(x) for x in values])
            )
        )
        print("requested, {}".format(response.status_code))
        response.close()
    except OSError as e:
        print("Error ", e)

    utime.sleep_ms(max(last_time + PERIOD - utime.ticks_ms(), 0))

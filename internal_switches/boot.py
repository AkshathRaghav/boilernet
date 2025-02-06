from machine import Pin
from neopixel import NeoPixel
from time import sleep

NP_PIN = 8

np_pin = Pin(NP_PIN, Pin.OUT)
np = NeoPixel(np_pin, 1)
for _ in range(3):
    np[0] = (0, 0, 0)
    np.write()
    sleep(0.2)
    np[0] = (255, 0, 0)
    np.write()
    sleep(0.2)

# np[0] = (0, 0, 255)
# np.write()
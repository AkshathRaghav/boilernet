from machine import Pin, UART
from neopixel import NeoPixel
import time
from time import sleep

NP_PIN = 8

np_pin = Pin(NP_PIN, Pin.OUT)
np = NeoPixel(np_pin, 1)
for _ in range(3):


TX = 6
RX = 7
LED_PIN = 9  

uart = UART(2, baudrate=115200, tx=TX, rx=RX)


def validate_message(data):
    if data.startswith("<START>") and data.endswith("<END>"):
        return data[7:-5]  
    return None 

while True:
    if uart.any():
        raw_data = uart.read().decode().strip()  
        validated_data = validate_message(raw_data)

        if validated_data:
            print(f"Valid Data Received: {validated_data}")
            np[0] = (0, 0, 0)
            np.write()
            sleep(0.1)
            np[0] = (255, 0, 0)
            np.write()
            sleep(0.1)
        else:
            print(f"Invalid Data Received: {raw_data}")

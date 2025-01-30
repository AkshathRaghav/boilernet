# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

from machine import Pin
from time import sleep

LED_PIN = 15

led = Pin(LED_PIN, Pin.OUT)

for i in range(6):
    led.value(not led.value())
    sleep(0.2)
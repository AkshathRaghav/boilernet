from machine import Pin, UART
import time
TX = 19
RX = 18
LED_PIN = 15
SWITCH_PIN = 33

# Configure UART
uart = UART(2, baudrate=12000000, tx=TX, rx=RX)  # TX=17, RX=16

# Configure LED
led = Pin(LED_PIN, Pin.OUT)
led.value(0)  # Initially OFF

# Configure Push Button
button = Pin(SWITCH_PIN, Pin.IN, Pin.PULL_UP)

# Debounce variables
last_button_state = button.value()
last_time = time.ticks_ms()
debounce_delay = 200  # ms

while True:
    # Read button state
    current_time = time.ticks_ms()
    button_state = button.value()

    # Check for button press with debounce
    if button_state == 0 and last_button_state == 1:
        if time.ticks_diff(current_time, last_time) > debounce_delay:
            uart.write("BUTTON_PRESSED\n")  # Send signal to the other board
            last_time = current_time
            print("Sent message!")

    last_button_state = button_state

    # Check if data is received
    if uart.any():
        #time.sleep(0.000001) (Sometimes necesary with a longer message (Will definitely need for data transfer))
        data = uart.read().decode().strip()
        if data == "BUTTON_PRESSED":
            print("Received Message")
            led.value(1)  # Turn ON LED
            time.sleep(0.5)  # Keep LED on for visibility
            led.value(0)  # Turn OFF LED
        else:
            print(f"Wrong data received: {data}")
            

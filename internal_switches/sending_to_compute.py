from machine import Pin, UART
import time

TX = 7
RX = 6
SWITCH_PIN = 9  
sd_file_path = "/sd/QualificationsScreenshot.webp" 
CHUNK_SIZE = 32  

uart = UART(2, baudrate=115200, tx=TX, rx=RX)

button = Pin(SWITCH_PIN, Pin.IN, Pin.PULL_UP)

last_button_state = button.value()
last_time = time.ticks_ms()
debounce_delay = 200  # ms

def send_chunk(data_chunk):
    formatted_chunk = f"<START>{data_chunk}<END>".encode()
    uart.write(formatted_chunk)
    print(f"Sent Chunk: {formatted_chunk}")

while True:
    current_time = time.ticks_ms()
    button_state = button.value()

    if button_state == 0 and last_button_state == 1:
        if time.ticks_diff(current_time, last_time) > debounce_delay:
            print("Button Pressed! Starting Data Transfer...")

            with open(sd_file_path, "rb") as sd_file:
                while True:
                    data_chunk = sd_file.read(CHUNK_SIZE)
                    if not data_chunk:  
                        break
                    send_chunk(data_chunk)
                    time.sleep(0.01)  

            last_time = current_time

    last_button_state = button_state

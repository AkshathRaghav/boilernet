import machine
from machine import Pin, SPI, SoftSPI
import sdcard
import os
import time

toggle = 0

led = machine.Pin(2, machine.Pin.OUT)

# SPI + SD Card init
spi = SoftSPI(1, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
sd = sdcard.SDCard(spi, Pin(5))

# Virtual FS (os takes care)
vfs = os.VfsFat(sd)
os.mount(sd, '/sd')

def BlinkLED(timer_one):
    global toggle
    if toggle == 1:
        led.value(0)
        toggle = 0
    else:
        led.value(1)
        toggle = 1
        
timer_one = machine.Timer(0)
timer_one.init(freq=5, mode=machine.Timer.PERIODIC, callback=BlinkLED)

def list_files_on_sd():
    try:
        files = os.listdir("/sd")
        print("\nFiles on SD card:")
        for file in files:
            print(f"- {file}")
    except Exception as e:
        print(f"Error listing files: {e}")


def transfer_file_to_sd(source_path, dest_path):
    try:
        with open(source_path, "rb") as src:
            data = src.read()

        with open(dest_path, "wb") as dest:
            dest.write(data) 

        print(f"File {source_path} successfully transferred to {dest_path}")

    except Exception as e:
        print(f"Error transferring file: {e}")

def read_file_and_write_to_flash(sd_file_path, flash_file_path):
    try:
        with open(sd_file_path, "rb") as sd_file:
            data = sd_file.read()  

        with open(flash_file_path, "wb") as flash_file:
            flash_file.write(data)  

        print(f"File successfully copied from SD to Flash as {flash_file_path}")

    except Exception as e:
        print(f"Error in reading/writing file: {e}")


source_path = "QualificationsScreenshot.webp"
dest_path = "/sd/QualificationsScreenshot.webp" 

list_files_on_sd()

transfer_file_to_sd(source_path, dest_path)

list_files_on_sd()

read_file_and_write_to_flash(dest_path, source_path)


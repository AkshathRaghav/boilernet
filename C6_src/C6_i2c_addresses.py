'''
THIS CODE IS NOT FUNCTIONAL DUE TO COMPETING MASTERS ON I2C
'''

from machine import Pin, I2C, Timer
import ujson
from neopixel import NeoPixel
from time import sleep

# Unique device ID (Set this manually for each device)
DEVICE_ID = 1  # Change this for each device
I2C_DATA_PIN = 19
I2C_CLOCK_PIN = 18
NP_PIN = 8

# Setup Neopixel
np_pin = Pin(NP_PIN, Pin.OUT)
np = NeoPixel(np_pin, 1)

# Configure I2C
i2c = I2C(0, scl=Pin(I2C_CLOCK_PIN), sda=Pin(I2C_DATA_PIN), freq=100000)  # Adjust pins as needed

# Mapping of I2C addresses to unique device IDs
address_id_map = {}

# Timer for periodic scanning
scan_timer = Timer(-1)

def scan_i2c_bus(timer):
    """Scans the I2C bus and updates the address-to-device ID map."""
    global address_id_map
    np[0] = (0, 100, 0)
    np.write()
    try:
        # Scan for devices
        addresses = i2c.scan()

        for addr in addresses:
            if addr not in address_id_map:
                print(f"New device detected at address {addr}. Requesting ID...")

                # Request device ID from the new device
                try:
                    i2c.writeto(addr, b"REQ_ID")  # Request ID from the device
                    device_id = i2c.readfrom(addr, 2).decode()  # Read 2-byte ID
                    address_id_map[addr] = int(device_id)
                    print(f"Device at address {addr} has ID {device_id}")
                except Exception as e:
                    print(f"Could not communicate with device at {addr}: {e}")

                # Send this device's ID to the new device
                try:
                    i2c.writeto(addr, f"ID:{DEVICE_ID}".encode())
                except Exception as e:
                    print(f"Could not send ID to device at {addr}: {e}")

        # Remove devices that disappeared from the bus
        existing_addresses = set(address_id_map.keys())
        for addr in existing_addresses:
            if addr not in addresses:
                print(f"Device at address {addr} removed")
                del address_id_map[addr]

        print(f"Updated address-ID map: {ujson.dumps(address_id_map)}")

    except Exception as e:
        print(f"Error scanning I2C bus: {e}")
    np[0] = (255, 0, 0)
    np.write()


# Function to respond to ID requests
def i2c_request_handler():
    """Handles requests for the device's unique ID."""
    try:
        if i2c.any():  # Check if data is available
            data = i2c.readfrom(0x00, 6)  # Read incoming data
            if data == b"REQ_ID":
                i2c.writeto(0x00, str(DEVICE_ID).encode())  # Send ID
    except Exception as e:
        print(f"Error responding to ID request: {e}")


# Start the I2C scanning process using a hardware timer
scan_timer.init(period=5000, mode=Timer.PERIODIC, callback=scan_i2c_bus)

print("I2C Device Discovery Running...")

while True:
    sleep(1)

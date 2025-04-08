#!/usr/bin/env python3
import serial
import time
import os

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
TIMEOUT = 3  # seconds

# File and transfer parameters
FILE_PATH = "image2.jpg"   # Change to your image file path
PATCH_SIZE = 1024         # Each patch is 1024 bytes
CHUNK_SIZE = 64           # Each data block sent is 64 bytes

def wait_for_ack(ser, expected, timeout=2):
    """
    Wait for an acknowledgement from the ESP32.
    This function now only logs the response and always returns True.
    :param ser: The serial connection.
    :param expected: The expected byte string (e.g., b"ACK_START").
    :param timeout: Maximum time to wait (in seconds).
    :return: Always returns True.
    """
    time.sleep(0.10)
    ack = ser.read(len(expected))
    if ack == expected:
        print(f"Received {ack.decode(errors='replace')}")
    return True

def send_patch(ser, patch_data):
    """
    Send one patch of data to the ESP32.
    This version does not abort the patch if an incorrect ack is received.
    It simply logs the ack and continues.
    """
    # Send start patch command
    ser.write(b"_START_PATCH_")
    print("Sent: _START_PATCH_")
    wait_for_ack(ser, b"ACK_START")

    # Send patch data in CHUNK_SIZE blocks
    total_blocks = (len(patch_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
    for i in range(0, len(patch_data), CHUNK_SIZE):
        block = patch_data[i:i+CHUNK_SIZE]
        ser.write(block)
        wait_for_ack(ser, b"ACK_DATA")
        print(f"Sent block {i // CHUNK_SIZE + 1} of {total_blocks}")
        time.sleep(0.01)  # Small delay between blocks

    # Send end patch command
    ser.write(b"_END_PATCH_")
    print("Sent: _END_PATCH_")
    wait_for_ack(ser, b"ACK_END")

    print("Patch sent successfully.")
    return True

def main():
    try:
        # Open the serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    # Check if the file exists
    if not os.path.exists(FILE_PATH):
        print(f"File not found: {FILE_PATH}")
        ser.close()
        return

    # Read the image file as binary data
    with open(FILE_PATH, "rb") as f:
        data = f.read()

    # Calculate the total number of patches needed
    total_patches = (len(data) + PATCH_SIZE - 1) // PATCH_SIZE
    print(f"Total patches to send: {total_patches}")

    # Loop over each patch and send it
    for patch_index in range(total_patches):
        patch_data = data[patch_index * PATCH_SIZE : (patch_index + 1) * PATCH_SIZE]
        print(f"\nSending patch {patch_index} with {len(patch_data)} bytes...")
        send_patch(ser, patch_data)
        # Optional: add a short delay between patches
        time.sleep(0.1)

    ser.close()
    print("Serial connection closed.")

if __name__ == "__main__":
    main()

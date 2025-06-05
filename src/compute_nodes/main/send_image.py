#!/usr/bin/env python3
"""
send_image.py

Send a PNG image over UART to ESP32 using the same packet protocol:
- PACKET_TYPE_START_DATA_TO_BLADE
- PACKET_TYPE_DATA_TO_BLADE
- PACKET_TYPE_END_DATA_TO_BLADE
"""

import serial
import time
import sys
import os 
from PIL import Image 
import io 

# Adjust these to match your ESP32 firmware definitions:
TRANSFER_SIZE = 1024  # must match ESP32 C code
PACKET_TYPE_START = 1
PACKET_TYPE_DATA  = 2
PACKET_TYPE_END   = 3

def send_image(port, baud, image_path):
    """Opens UART, sends image in framed packets, and prints the JSON response."""
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    ext = os.path.splitext(image_path)[1].lower()
    img = Image.open(image_path).convert('RGB').resize((224,224))
    buf = io.BytesIO()
    img.save(buf, format='PNG', optimize=True, compress_level=9)
    img_bytes = buf.getvalue()

    # 1) Send START packet (no payload)
    header = bytes([PACKET_TYPE_START]) + (0).to_bytes(2, 'big')
    packet = header + b'\x00' * (TRANSFER_SIZE - len(header))
    ser.write(packet)

    # 2) Send DATA and END packets in chunks
    max_payload = TRANSFER_SIZE - 3
    offset = 0
    while offset < len(img_bytes):
        chunk = img_bytes[offset:offset + max_payload]
        offset += len(chunk)
        if offset < len(img_bytes):
            ptype = PACKET_TYPE_DATA
        else:
            ptype = PACKET_TYPE_END
        length = len(chunk)
        header = bytes([ptype]) + length.to_bytes(2, 'big')
        packet = header + chunk
        # pad to fixed packet size
        packet += b'\x00' * (TRANSFER_SIZE - len(packet))
        ser.write(packet)
        time.sleep(0.01)

    # 3) Read JSON response from ESP32
    print("Awaiting response...")
    while True:
        line = ser.readline()
        if not line:
            continue
        try:
            text = line.decode('utf-8').strip()
        except UnicodeDecodeError:
            continue
        if text.startswith('{') and text.endswith('}'):
            print("Received:", text)
            break

    ser.close()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: python send_image.py <port> <baud> <image_path>")
        sys.exit(1)
    send_image(sys.argv[1], int(sys.argv[2]), sys.argv[3])

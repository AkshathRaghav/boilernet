#!/usr/bin/env python3
import socket
import struct
import sys
import time
from enum import Enum

from PIL import Image
import lz4.frame

PACKET_TYPE_START = 0xA0
PACKET_TYPE_DATA  = 0xA1
PACKET_TYPE_MID   = 0xA2
PACKET_TYPE_END   = 0xA3

DATA_PACKET_SIZE = 1024  

class State(Enum):
    IDLE = 0
    SEND_START = 1
    WAIT_START_ACK = 2
    SEND_DATA = 3
    SEND_MID = 4
    SEND_REMAINING = 5
    SEND_END = 6
    WAIT_MID_ACK = 10
    WAIT_END_ACK = 7
    COMPLETE = 8
    ERROR = 9

# --------------------------
# Helper functions
# --------------------------
def calculate_checksum(data_bytes):
    # A simple checksum: sum of all bytes mod 2^32.
    return sum(data_bytes) & 0xFFFFFFFF

def create_start_packet(width, height, total_data_len):
    """
    START packet structure:
    [Packet Type (1 byte)]
    [Width (2 bytes, unsigned short)]
    [Height (2 bytes, unsigned short)]
    [Total data length (4 bytes, unsigned int)]
    """
    return struct.pack("!BHHI", PACKET_TYPE_START, width, height, total_data_len)

def create_data_packet(data_chunk):
    """
    DATA packet structure:
    [Packet Type (1 byte)]
    [Data length (2 bytes, unsigned short)]
    [Payload (data_chunk)]
    """
    length = len(data_chunk)
    header = struct.pack("!BH", PACKET_TYPE_DATA, length)
    return header + data_chunk

def create_mid_packet():
    """
    MID packet structure:
    [Packet Type (1 byte)]
    [Reserved/Padding (if needed, here 2 bytes length field set to zero)]
    """
    # For simplicity, no additional payload
    return struct.pack("!BH", PACKET_TYPE_MID, 0)

def create_end_packet(checksum):
    """
    END packet structure:
    [Packet Type (1 byte)]
    [Checksum (4 bytes, unsigned int)]
    """
    return struct.pack("!BI", PACKET_TYPE_END, checksum)

def wait_for_ack(sock, expected_ack, timeout=5):
    """
    Wait for a specific ACK byte or message from the ESP32.
    For simplicity, assume the ESP32 sends a single byte ACK corresponding to the packet type.
    """
    sock.settimeout(timeout)
    try:
        ack = sock.recv(1)
        if not ack:
            return False
        print(ack)
        return ack[0] == expected_ack
    except socket.timeout:
        return False

# --------------------------
# FSM for Image Transfer
# --------------------------
def fsm_image_transfer(sock, image_path):
    # Load image and convert to RGB bytes
    try:
        image = Image.open(image_path).convert("RGB")
    except Exception as e:
        print("Error loading image:", e)
        return

    width, height = image.size
    raw_data = image.tobytes()  # raw RGB bytes

    # Break the raw image into chunks, compress each using LZ4,
    # and then adjust so that every compressed chunk has an even number of bytes.
    compressed_chunks = []
    for i in range(0, len(raw_data), DATA_PACKET_SIZE):
        data_chunk = raw_data[i:i+DATA_PACKET_SIZE]
        comp_chunk = lz4.frame.compress(data_chunk)
        # If the compressed chunk length is odd, append one padding byte.
        if len(comp_chunk) % 2 != 0:
            comp_chunk += b'\x00'
        compressed_chunks.append(comp_chunk)

    total_packets = len(compressed_chunks)
    total_data_len = sum(len(chunk) for chunk in compressed_chunks)
    mid_packet_index = total_packets // 2  # when to send MID packet

    # Calculate checksum over the concatenated compressed chunks.
    comp_checksum = calculate_checksum(b"".join(compressed_chunks))

    print(f"Image loaded: {width}x{height}, raw {len(raw_data)} bytes")
    print(f"Compressed into {total_data_len} bytes over {total_packets} packets.")
    print(f"Checksum over compressed data: {comp_checksum:#010x}")

    state = State.SEND_START
    current_packet = 0  # counter for data packets sent

    while state != State.COMPLETE and state != State.ERROR:
        if state == State.SEND_START:
            # Build and send START packet with total compressed data length.
            start_packet = create_start_packet(width, height, total_data_len)
            sock.sendall(start_packet)
            print("Sent START packet.")
            state = State.WAIT_START_ACK

        elif state == State.WAIT_START_ACK:
            if wait_for_ack(sock, PACKET_TYPE_START):
                print("Received ACK for START.")
                state = State.SEND_DATA
            else:
                print("No ACK for START, error.")
                state = State.ERROR

        elif state == State.SEND_DATA:
            if current_packet >= total_packets:
                state = State.SEND_END
                continue

            comp_chunk = compressed_chunks[current_packet]
            data_packet = create_data_packet(comp_chunk)
            sock.sendall(data_packet)
            # print(f"Sent DATA packet {current_packet+1}/{total_packets}")
            current_packet += 1

            if current_packet == mid_packet_index:
                state = State.SEND_MID
            elif current_packet == total_packets:
                state = State.SEND_END

        elif state == State.SEND_MID:
            mid_packet = create_mid_packet()
            sock.sendall(mid_packet)
            print("Sent MID packet.")
            state = State.WAIT_MID_ACK  # Continue sending the rest

        elif state == State.WAIT_MID_ACK:
            if wait_for_ack(sock, PACKET_TYPE_MID):
                print("Received ACK for MID.")
                state = State.SEND_DATA
            else:
                print("No ACK for MID, error.")
                state = State.ERROR

        elif state == State.SEND_END:
            end_packet = create_end_packet(comp_checksum)
            sock.sendall(end_packet)
            print(f"Sent END packet with checksum {comp_checksum:#010x}")
            state = State.WAIT_END_ACK

        elif state == State.WAIT_END_ACK:
            if wait_for_ack(sock, PACKET_TYPE_END):
                print("Received final ACK for END. Transfer complete.")
                state = State.COMPLETE
            else:
                print("No final ACK for END, error.")
                state = State.ERROR

        else:
            print("Undefined state, error.")
            state = State.ERROR

    if state == State.COMPLETE:
        print("FSM: Transfer completed successfully.")
    else:
        print("FSM: Transfer encountered an error.")

def main():
    if len(sys.argv) != 4:
        print("Usage: python transfer_image.py <image_file> <ESP32_IP> <port>")
        sys.exit(1)

    image_file = sys.argv[1]
    esp32_ip = sys.argv[2]
    port = int(sys.argv[3])

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print(f"Connecting to {esp32_ip}:{port}...")
        sock.connect((esp32_ip, port))
        print("Connected to ESP32.")

        fsm_image_transfer(sock, image_file)

if __name__ == "__main__":
    main()


# python send_eth_image.py akshath_face_tdm.png 192.168.0.50 8080

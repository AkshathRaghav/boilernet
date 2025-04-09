#!/usr/bin/env python3
import socket
import struct
import sys
import time
from enum import Enum

from PIL import Image  # Still used by the write FSM

# --------------------------
# Packet Type Macros
# --------------------------
# Write packet types (existing)
PACKET_TYPE_START_WRITE = 0xA0
PACKET_TYPE_DATA_WRITE  = 0xA1
PACKET_TYPE_MID_WRITE   = 0xA2
PACKET_TYPE_END_WRITE   = 0xA3
PACKET_TYPE_FUC = 0xA4

# Read packet types (new)
PACKET_TYPE_START_READ = 0xB0
PACKET_TYPE_DATA_READ  = 0xB1
PACKET_TYPE_END_READ   = 0xB3

# Predefined error code for "file not found" (you can adjust the value as needed)
FILE_NOT_FOUND_ERROR_CODE = 0xFF

DATA_PACKET_SIZE = 2030
FILENAME = "example.txt"  # used for writes; you might choose to override via CLI

# --------------------------
# FSM State enum (for write only; new read FSM uses a loop)
# --------------------------
class State(Enum):
    IDLE = 0
    SEND_START_WRITE = 1
    WAIT_START_ACK_WRITE = 2
    SEND_DATA_WRITE = 3
    SEND_MID_WRITE = 4
    SEND_END_WRITE = 6
    WAIT_MID_ACK_WRITE = 10
    WAIT_END_ACK_WRITE = 7
    COMPLETE = 8
    ERROR = 9

# --------------------------
# Helper functions for common tasks
# --------------------------
def calculate_checksum(data_bytes):
    # A simple checksum: sum of all bytes mod 2^32.
    return sum(data_bytes) & 0xFFFFFFFF

def recvall(sock, n):
    """
    Helper to receive exactly n bytes from the socket.
    """
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            break
        data.extend(packet)
    return bytes(data)

# --------------------------
# Write Mode Helper Functions and FSM (existing)
# --------------------------
def create_start_packet(filename):
    """
    START packet structure for writing:
    [Packet Type (1 byte)]
    [Filename (48 bytes)]
    """
    filename_bytes = filename.encode('utf-8')
    if len(filename_bytes) > 48:
        filename_bytes = filename_bytes[:48]
    else:
        filename_bytes = filename_bytes.ljust(48, b'\0')
    return struct.pack("!B48s", PACKET_TYPE_START_WRITE, filename_bytes)

def create_data_packet(data_chunk):
    """
    DATA packet structure (write):
    [Packet Type (1 byte)]
    [Data length (2 bytes, unsigned short)]
    [Payload (data_chunk)]
    """
    length = len(data_chunk)
    header = struct.pack("!BH", PACKET_TYPE_DATA_WRITE, length)
    return header + data_chunk

def create_mid_packet():
    """
    MID packet structure (write):
    [Packet Type (1 byte)]
    [Reserved/Padding (2 bytes length set to zero)]
    """
    return struct.pack("!BH", PACKET_TYPE_MID_WRITE, 0)

def create_end_packet(checksum):
    """
    END packet structure (write):
    [Packet Type (1 byte)]
    [Checksum (4 bytes, unsigned int)]
    """
    return struct.pack("!BI", PACKET_TYPE_END_WRITE, checksum)

def wait_for_ack(sock, expected_ack, timeout=5):
    """
    Wait for a specific ACK packet from the ESP32.
    For simplicity, assume the ESP32 sends a single byte ACK corresponding to the packet type.
    """
    sock.settimeout(timeout)
    try:
        ack = sock.recv(1)
        if not ack:
            return False
        print("ACK received:", ack[0])
        return ack[0] == expected_ack
    except socket.timeout:
        return False

def fsm_image_transfer(sock, image_path):
    try:
        with open(image_path, 'rb') as f:
            raw_data = f.read()
    except Exception as e:
        print("Error loading image:", e)
        return

    width, height = 25, 25
    total_data_len = len(raw_data)
    print(f"Image loaded: {width}x{height}, total {total_data_len} bytes")

    num_full_packets = total_data_len // DATA_PACKET_SIZE
    remainder = total_data_len % DATA_PACKET_SIZE
    total_packets = num_full_packets + (1 if remainder > 0 else 0)
    mid_packet_index = total_packets // 2  # when to send MID packet
    print("num_full_packets ", num_full_packets);
    print("remainder ", remainder);
    print("total_packets ", total_packets)
    print("mid_packet_index ", mid_packet_index)

    state = State.SEND_START_WRITE
    current_packet = 0  # counter for data packets sent

    while state != State.COMPLETE and state != State.ERROR:
        if state == State.SEND_START_WRITE:
            start_packet = create_start_packet(image_path)
            sock.sendall(start_packet)
            print("Sent START packet for write.")
            state = State.WAIT_START_ACK_WRITE

        elif state == State.WAIT_START_ACK_WRITE:
            if wait_for_ack(sock, PACKET_TYPE_START_WRITE):
                print("Received ACK for START (write).")
                state = State.SEND_DATA_WRITE
            else:
                print("No ACK for START (write), error.")
                state = State.ERROR

        elif state == State.SEND_DATA_WRITE:
            start_idx = current_packet * DATA_PACKET_SIZE
            end_idx = start_idx + DATA_PACKET_SIZE
            if start_idx >= total_data_len:
                state = State.SEND_END_WRITE
                continue

            data_chunk = raw_data[start_idx:end_idx]
            data_packet = create_data_packet(data_chunk)
            sock.sendall(data_packet)
            current_packet += 1

            if current_packet == mid_packet_index:
                state = State.SEND_MID_WRITE
            elif current_packet == total_packets:
                state = State.SEND_END_WRITE

        elif state == State.SEND_MID_WRITE:
            mid_packet = create_mid_packet()
            sock.sendall(mid_packet)
            print("Sent MID packet (write).")
            state = State.WAIT_MID_ACK_WRITE

        elif state == State.WAIT_MID_ACK_WRITE:
            if wait_for_ack(sock, PACKET_TYPE_MID_WRITE):
                print("Received ACK for MID (write).")
                state = State.SEND_DATA_WRITE
            else:
                print("No ACK for MID (write), error.")
                state = State.ERROR

        elif state == State.SEND_END_WRITE:
            checksum = calculate_checksum(raw_data)
            end_packet = create_end_packet(checksum)
            sock.sendall(end_packet)
            print(f"Sent END packet (write) with checksum {checksum:#010x}")
            state = State.WAIT_END_ACK_WRITE

        elif state == State.WAIT_END_ACK_WRITE:
            sock.settimeout(5)
            try:
                ack = sock.recv(1)
                if not ack:
                    return False
                print("ACK received:", ack[0])
                if (ack[0] == PACKET_TYPE_END_WRITE): 
                    print("Received final ACK for END (write). Transfer complete.")
                    state = State.COMPLETE
                elif (ack[0] == PACKET_TYPE_FUC): 
                    print("Got PACKET_TYPE_FUC")
                    state = State.ERROR
                else: 
                    print("WRONG ACK GOT: ", ack);
                    state = State.ERROR
            except socket.timeout:
                print("No final ACK for END (write), error.")
                state = State.ERROR

        else:
            print("Undefined state (write), error.")
            state = State.ERROR

    if state == State.COMPLETE:
        print("FSM (write): Transfer completed successfully.")
    else:
        print("FSM (write): Transfer encountered an error.")

# --------------------------
# New Read Mode Helper Functions and FSM (without MID packets)
# --------------------------
def create_start_read_packet(filename):
    """
    START_READ packet structure:
    [Packet Type (1 byte)]
    [Filename (48 bytes)]
    """
    filename_bytes = filename.encode('utf-8')
    if len(filename_bytes) > 48:
        filename_bytes = filename_bytes[:48]
    else:
        filename_bytes = filename_bytes.ljust(48, b'\0')
    return struct.pack("!B48s", PACKET_TYPE_START_READ, filename_bytes)

def fsm_file_read(sock, output_filename):
    """
    FSM for reading data from the slave.
    The master (router) sends a START_READ packet.
    Then the slave sends DATA_READ packets, and finally sends an END_READ packet with a checksum.
    In case the file is not found, the slave will send an END_READ packet with a special error code.
    """
    # Send the START_READ request
    start_read_packet = create_start_read_packet(output_filename)
    sock.sendall(start_read_packet)
    print("Sent START_READ packet.")

    # Optionally wait for an ACK from the slave for start read.
    if not wait_for_ack(sock, PACKET_TYPE_START_READ):
        print("No ACK for START_READ, error.")
        return

    print("Received ACK for START_READ. Beginning to receive data...")
    with open("returned_"+output_filename, 'wb') as f:
        while True:
            header = recvall(sock, 1)
            if not header:
                print("Socket closed unexpectedly during read.")
                return
            packet_type = header[0]
            
            if packet_type == PACKET_TYPE_DATA_READ:
                # Expect the next 2 bytes to specify the data length.
                length_bytes = recvall(sock, 2)
                if len(length_bytes) != 2:
                    print("Error reading DATA_READ length.")
                    return
                data_length = struct.unpack("!H", length_bytes)[0]
                data_chunk = recvall(sock, data_length)
                if len(data_chunk) != data_length:
                    print("Incomplete DATA_READ chunk received.")
                    return
                f.write(data_chunk)
                print(f"Received DATA_READ packet with {data_length} bytes.")

            elif packet_type == PACKET_TYPE_END_READ:
                # END_READ: read the 4-byte checksum.
                checksum_bytes = recvall(sock, 4)
                if len(checksum_bytes) != 4:
                    print("Error reading END_READ checksum.")
                    return
                remote_checksum = struct.unpack("!I", checksum_bytes)[0]
                if remote_checksum == FILE_NOT_FOUND_ERROR_CODE:
                    print("Error: The requested file was not found on the slave.")
                    return
                print(f"Received END_READ packet with checksum {remote_checksum:#010x}")
                f.flush()
                break

            else:
                print("Unexpected packet type received during read:", packet_type)
                return

    print("FSM (read): File read transfer complete.")

# --------------------------
# Main function: select mode based on arguments
# --------------------------
def main():
    if len(sys.argv) != 5:
        print("Usage: python transfer_image.py <port> <ESP32_IP> <file> <mode>")
        print("   mode: 'write' (to send data) or 'read' (to request and save data)")
        sys.exit(1)

    port = int(sys.argv[1])
    esp32_ip = sys.argv[2]
    file_arg = sys.argv[3]
    mode = sys.argv[4].lower()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print(f"Connecting to {esp32_ip}:{port}...")
        sock.connect((esp32_ip, port))
        print("Connected to ESP32.")

        start_time = time.time()
        if mode == "write":
            fsm_image_transfer(sock, file_arg)
        elif mode == "read":
            fsm_file_read(sock, file_arg)
        else:
            print("Invalid mode. Use 'write' or 'read'.")
            sys.exit(1)
        end_time = time.time()
        print("Total Time: ", end_time - start_time)

if __name__ == "__main__":
    main()

# python send_eth.py 8080 192.168.0.50 test2.txt read
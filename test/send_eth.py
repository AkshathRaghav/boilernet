#!/usr/bin/env python3
import socket
import struct
import sys
import time
import io
from enum import Enum
from PIL import Image 


INPUT_DIR = "./input"
RESULTS_DIR = "./results" 

# --------------------------
# Packet Type Macros
# --------------------------
PACKET_TYPE_START_WRITE = 0xA0
PACKET_TYPE_DATA_WRITE = 0xA1
PACKET_TYPE_MID_WRITE = 0xA2
PACKET_TYPE_END_WRITE = 0xA3
PACKET_TYPE_FUC = 0xA4

PACKET_TYPE_START_READ = 0xB0
PACKET_TYPE_DATA_READ = 0xB1
PACKET_TYPE_END_READ = 0xB3

PACKET_TYPE_START_COMPUTE = 0xC0
PACKET_TYPE_POLL_COMPUTE = 0xC1
PACKET_TYPE_WAIT_COMPUTE = 0xC2
PACKET_TYPE_END_COMPUTE = 0xC3

PACKET_TYPE_DELETE = 0xE0
PACKET_TYPE_DELETE_RET = 0xE1

FILE_NOT_FOUND_ERROR_CODE = 0xFF

DATA_PACKET_SIZE = 2030


# --------------------------
# Timing instrumentation
# --------------------------
response_times = []
packet_response_times = {}

def record_packet_time(packet_name, start, end):
    dt_ms = (end - start) * 1000
    response_times.append(dt_ms)
    packet_response_times.setdefault(packet_name, []).append(dt_ms)

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
# Write Mode Helper Functions and FSM
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

def create_start_packet(filename):
    """
    START packet structure for writing:
    [Packet Type (1 byte)]
    [Filename (80 bytes)]
    """
    if filename.lower().endswith((".jpg", ".jpeg", ".jpeg")):
        filename = filename.split(".")[0] + ".png"
    filename_bytes = filename.encode("utf-8")
    if len(filename_bytes) > 80:
        filename_bytes = filename_bytes[:80]
    else:
        filename_bytes = filename_bytes.ljust(80, b"\0")
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
    Measures the time from calling this function to receiving the ACK.
    """
    sock.settimeout(timeout)
    start = time.perf_counter()
    try:
        ack = sock.recv(1)
        end = time.perf_counter()
        record_packet_time(f"ACK_{expected_ack:02X}", start, end)
        if not ack:
            return False
        print("ACK received:", ack[0])
        return ack[0] == expected_ack
    except socket.timeout:
        end = time.perf_counter()
        record_packet_time(f"ACK_TIMEOUT_{expected_ack:02X}", start, end)
        return False


def fsm_image_transfer(sock, image_path):
    try:
        if image_path.lower().endswith((".jpg", ".jpeg", ".jpeg")):
            img = Image.open(image_path).convert("RGB")
            img = img.resize((224, 224))
            buf = io.BytesIO()
            img.save(buf, format="PNG", optimize=True, compress_level=9)
            raw_data = buf.getvalue()
        else:
            with open(image_path, "rb") as f:
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
    print("num_full_packets ", num_full_packets)
    print("remainder ", remainder)
    print("total_packets ", total_packets)
    print("mid_packet_index ", mid_packet_index)

    state = State.SEND_START_WRITE
    current_packet = 0  # counter for data packets sent

    while state not in (State.COMPLETE, State.ERROR):
        if state == State.SEND_START_WRITE:
            pkt = create_start_packet(image_path)
            sock.sendall(pkt)
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
            pkt = create_data_packet(data_chunk)
            sock.sendall(pkt)
            current_packet += 1

            if current_packet == mid_packet_index:
                state = State.SEND_MID_WRITE
            elif current_packet == total_packets:
                state = State.SEND_END_WRITE

        elif state == State.SEND_MID_WRITE:
            pkt = create_mid_packet()
            sock.sendall(pkt)
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
            pkt = create_end_packet(checksum)
            sock.sendall(pkt)
            print(f"Sent END packet (write) with checksum {checksum:#010x}")
            state = State.WAIT_END_ACK_WRITE

        elif state == State.WAIT_END_ACK_WRITE:
            sock.settimeout(5)
            start = time.perf_counter()
            try:
                ack = sock.recv(1)
                end = time.perf_counter()
                record_packet_time(f"ACK_{PACKET_TYPE_END_WRITE:02X}", start, end)
                if not ack:
                    return False
                print("ACK received:", ack[0])
                if ack[0] == PACKET_TYPE_END_WRITE:
                    print("Received final ACK for END (write). Transfer complete.")
                    state = State.COMPLETE
                elif ack[0] == PACKET_TYPE_FUC:
                    print("Got PACKET_TYPE_FUC")
                    state = State.ERROR
                else:
                    print("WRONG ACK GOT: ", ack)
                    state = State.ERROR
            except socket.timeout:
                end = time.perf_counter()
                record_packet_time(
                    f"ACK_TIMEOUT_{PACKET_TYPE_END_WRITE:02X}", start, end
                )
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
# Read Mode Helper Functions and FSM
# --------------------------
def create_start_read_packet(filename):
    """
    START_READ packet structure:
    [Packet Type (1 byte)]
    [Filename (48 bytes)]
    """
    filename_bytes = filename.encode("utf-8")
    if len(filename_bytes) > 80:
        filename_bytes = filename_bytes[:80]
    else:
        filename_bytes = filename_bytes.ljust(80, b"\0")
    return struct.pack("!B48s", PACKET_TYPE_START_READ, filename_bytes)


def fsm_file_read(sock, output_filename):
    if output_filename.lower().endswith((".jpg", ".jpeg", ".jpeg")):
        output_filename = output_filename.split(".")[0] + ".png"
        print(output_filename)

    start_read_pkt = create_start_read_packet(output_filename)
    sock.sendall(start_read_pkt)
    print("Sent START_READ packet.")

    if not wait_for_ack(sock, PACKET_TYPE_START_READ):
        print("No ACK for START_READ, error.")
        return

    print("Received ACK for START_READ. Beginning to receive data...")
    with open(os.path.join(RESULTS_DIR, output_filename), "wb") as f:
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
                start = time.perf_counter()
                data_chunk = recvall(sock, data_length)
                end = time.perf_counter()
                record_packet_time(f"DATA_READ", start, end)
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
                msb = (remote_checksum >> 24) & 0xFF

                if msb == FILE_NOT_FOUND_ERROR_CODE:
                    print("Error: The requested file was not found on the slave.")
                    return
                print(f"Received END_READ packet with checksum {remote_checksum:#010x}")
                break

            else:
                print("Unexpected packet type received during read:", packet_type)
                return

    print("FSM (read): File read transfer complete.")


# --------------------------
# Compute Mode Helper Functions and FSM
# --------------------------
def create_start_compute_packet(filename):
    filename_bytes = filename.encode("utf-8")
    if len(filename_bytes) > 80:
        filename_bytes = filename_bytes[:80]
    else:
        filename_bytes = filename_bytes.ljust(80, b"\0")
    return struct.pack("!B48s", PACKET_TYPE_START_COMPUTE, filename_bytes)

def create_poll_compute_packet():
    return struct.pack("!B", PACKET_TYPE_POLL_COMPUTE)

def fsm_compute(sock, filename):
    if filename.lower().endswith((".jpg", ".jpeg", ".jpeg")):
        filename = filename.split(".")[0] + ".png"

    print(filename)
    start_pkt = create_start_compute_packet(filename)
    sock.sendall(start_pkt)
    print("Sent START_COMPUTE packet to Master.")

    sock.settimeout(2)
    start = time.perf_counter()
    try:
        response = sock.recv(2)
        end = time.perf_counter()
        record_packet_time("START_COMPUTE_RESPONSE", start, end)
    except socket.timeout:
        print("Timeout waiting for compute response.")
        return

    if not response:
        print("No response received for compute flow.")
        return

    resp_type = response[0]
    if resp_type == PACKET_TYPE_FUC:
        print("Compute error: Received FUC packet.")
        return
    elif resp_type == PACKET_TYPE_END_COMPUTE:
        if len(response) >= 2 and response[1] == 0xFF:
            print("Compute error: No such file on the SD Card (flag 0xFF).")
        else:
            print(
                "Compute ended unexpectedly. Flag:",
                hex(response[1]) if len(response) > 1 else "N/A",
            )
        return
    elif resp_type == PACKET_TYPE_START_COMPUTE:
        print("Compute process has been confirmed and started by Master/Slave.")
    else:
        print("Unexpected response type: 0x{:02X}".format(resp_type))
        return

    while True:
        time.sleep(2.5)
        poll_pkt = create_poll_compute_packet()
        sock.sendall(poll_pkt)
        print("Sent POLL_COMPUTE packet, waiting for status...")

        sock.settimeout(10)
        start = time.perf_counter()
        try:
            poll_response = sock.recv(2)
            end = time.perf_counter()
            record_packet_time("POLL_COMPUTE_RESPONSE", start, end)
        except socket.timeout:
            print("Timeout waiting for compute poll response.")
            return

        if not poll_response:
            print("No response received during polling.")
            return

        poll_type = poll_response[0]
        if poll_type == PACKET_TYPE_FUC:
            print("Compute error: Received FUC packet during polling.")
            return
        elif poll_type == PACKET_TYPE_END_COMPUTE:
            if len(poll_response) >= 2 and poll_response[1] == 0xFF:
                print(
                    "Compute error during polling: No such file on SD Card (flag 0xFF)."
                )
                return
            else:
                print(
                    "Compute process completed successfully. Flag:",
                    hex(poll_response[1]),
                )
                return
        elif poll_type == PACKET_TYPE_WAIT_COMPUTE:
            print("Compute is still in progress... waiting...")
        else:
            print("Unexpected poll response: 0x{:02X}".format(poll_type))
            return


# --------------------------
# Main function: select mode based on arguments
# --------------------------
def main():
    if len(sys.argv) != 5:
        print("Usage: python transfer_image.py <port> <ESP32_IP> <file> <mode>")
        print(
            "   mode: 'write' (to send data) or 'read' (to request and save data) or 'compute'"
        )
        sys.exit(1)

    port = int(sys.argv[1])
    esp32_ip = sys.argv[2]
    file_arg = os.path.join(INPUT_DIR, sys.argv[3])
    mode = sys.argv[4].lower()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print(f"Connecting to {esp32_ip}:{port}...")
        sock.connect((esp32_ip, port))
        print("Connected to ESP32.")

        overall_start = time.perf_counter()
        if mode == "write":
            fsm_image_transfer(sock, file_arg)
        elif mode == "read":
            fsm_file_read(sock, file_arg)
        elif mode == "compute":
            fsm_compute(sock, file_arg)
        else:
            print("Invalid mode. Use 'write', 'read', or 'compute'.")
            sys.exit(1)
        overall_end = time.perf_counter()

        total_ms = (overall_end - overall_start) * 1000
        print(f"\n=== Timing Summary ===")
        print(f"Total round‑trip time: {total_ms:.2f} ms")
        if response_times:
            avg = sum(response_times) / len(response_times)
            print(
                f"Average response time: {avg:.2f} ms over {len(response_times)} measurements"
            )
        else:
            print("No response times recorded.")
        print("Per‑packet response times (ms):")
        for pkt, times in packet_response_times.items():
            avg = sum(times) / len(times)
            print(f"  {pkt}: {avg:.2f} ms")


if __name__ == "__main__":
    main()

# python send.py 8080 192.168.0.50 test2.txt read

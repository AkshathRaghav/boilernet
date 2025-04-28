#!/usr/bin/env python3
import socket
import struct
import sys
import time
from enum import Enum
from PIL import Image
import struct
import lz4.frame

DATA_PACKET_SIZE = 1024  

PACKET_TYPE_START = 0xA0
PACKET_TYPE_DATA  = 0xA1
PACKET_TYPE_MID   = 0xA2
PACKET_TYPE_END   = 0xA3

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

def create_start_packet(width, height, total_data_len):
    # Pack header info: image width, image height, total serialized data length.
    return struct.pack(">III", width, height, total_data_len)

def create_data_packet(data_chunk):
    # This is a placeholder: you might add headers or packet numbers as needed.
    return data_chunk

def create_mid_packet():
    # Placeholder for MID packet (could be used for synchronization)
    return b"MID"

def create_end_packet(checksum):
    # Pack the checksum into the end packet.
    return struct.pack(">I", checksum)

def wait_for_ack(sock, packet_type):
    # Dummy ACK wait function; replace with your actual ACK handling logic.
    return True

def calculate_checksum(data):
    # Simple checksum: sum of all bytes modulo 2**32.
    return sum(data) % (2**32)

# --------------------------
# Helper Functions: Tiling & Compression
# --------------------------
def encode_image_to_tiles(image_path, tile_width, tile_height, convert_bw=False):
    """
    Opens an image, optionally converts it to black and white,
    splits it into tiles of size (tile_width x tile_height),
    and compresses each tile using LZ4.
    
    Returns a dictionary containing:
      - image_width: width of the original image.
      - image_height: height of the original image.
      - tiles: a list of tile dictionaries, each with:
           x, y: position of the tile in the original image.
           width, height: dimensions of the tile.
           mode: image mode (e.g., 'RGB' or 'L').
           data: LZ4 compressed raw bytes of the tile.
    """
    im = Image.open(image_path)
    im = im.convert("L") if convert_bw else im.convert("RGB")
    width, height = im.size
    tiles = []
    
    for y in range(0, height, tile_height):
        for x in range(0, width, tile_width):
            box = (x, y, min(x + tile_width, width), min(y + tile_height, height))
            tile = im.crop(box)
            raw_data = tile.tobytes()
            compressed_data = lz4.frame.compress(raw_data)
            tile_info = {
                'x': x,
                'y': y,
                'width': tile.width,
                'height': tile.height,
                'mode': tile.mode,
                'data': compressed_data
            }
            tiles.append(tile_info)
    
    return {'image_width': width, 'image_height': height, 'tiles': tiles}

def serialize_encoded_tiles(encoded_data):
    """
    Serializes the encoded tile data into a binary format.
    
    Format:
      Header: image_width (4 bytes), image_height (4 bytes), number_of_tiles (4 bytes)
      Then for each tile:
         x (4 bytes), y (4 bytes), tile_width (4 bytes), tile_height (4 bytes),
         mode_length (1 byte), mode (mode_length bytes),
         compressed_data_length (4 bytes), compressed_data (variable length)
    """
    header = struct.pack(">III", encoded_data['image_width'], encoded_data['image_height'], len(encoded_data['tiles']))
    body = b""
    for tile in encoded_data['tiles']:
        mode_bytes = tile['mode'].encode('utf-8')
        mode_length = len(mode_bytes)
        # Pack tile metadata: x, y, tile width, tile height, and length of mode string.
        tile_header = struct.pack(">IIII B", tile['x'], tile['y'], tile['width'], tile['height'], mode_length)
        body += tile_header + mode_bytes
        # Pack the length of the compressed data and the data itself.
        comp_data = tile['data']
        body += struct.pack(">I", len(comp_data))
        body += comp_data
    return header + body

# --------------------------
# FSM for Image Transfer (Revised)
# --------------------------
def fsm_image_transfer(sock, image_path):
    """
    Revised FSM that encodes the image into tiles (with LZ4 compression),
    serializes the data into a single binary blob, splits it into DATA_PACKET_SIZE chunks,
    and sends it over the socket. The receiver can atomically reconstruct the image.
    """
    try:
        # Adjust tile dimensions as needed.
        tile_w, tile_h = 64, 64  # You can experiment with tile size.
        # Set convert_bw=True if you want to convert to black and white.
        encoded_data = encode_image_to_tiles(image_path, tile_w, tile_h, convert_bw=False)
        serialized_data = serialize_encoded_tiles(encoded_data)
    except Exception as e:
        print("Error encoding image:", e)
        return

    width = encoded_data['image_width']
    height = encoded_data['image_height']
    total_data_len = len(serialized_data)
    print(f"Image encoded into {len(encoded_data['tiles'])} tiles, total {total_data_len} bytes")

    # Determine total number of data packets based on DATA_PACKET_SIZE.
    num_full_packets = total_data_len // DATA_PACKET_SIZE
    remainder = total_data_len % DATA_PACKET_SIZE
    total_packets = num_full_packets + (1 if remainder > 0 else 0)
    mid_packet_index = total_packets // 2

    state = State.SEND_START
    current_packet = 0

    while state not in [State.COMPLETE, State.ERROR]:
        if state == State.SEND_START:
            # Send START packet containing basic image info and total payload length.
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
            start_idx = current_packet * DATA_PACKET_SIZE
            end_idx = start_idx + DATA_PACKET_SIZE
            if start_idx >= total_data_len:
                state = State.SEND_END
                continue

            data_chunk = serialized_data[start_idx:end_idx]
            data_packet = create_data_packet(data_chunk)
            sock.sendall(data_packet)
            current_packet += 1

            if current_packet == mid_packet_index:
                state = State.SEND_MID
            elif current_packet == total_packets:
                state = State.SEND_END

        elif state == State.SEND_MID:
            mid_packet = create_mid_packet()
            sock.sendall(mid_packet)
            print("Sent MID packet.")
            state = State.WAIT_MID_ACK

        elif state == State.WAIT_MID_ACK:
            if wait_for_ack(sock, PACKET_TYPE_MID):
                print("Received ACK for MID.")
                state = State.SEND_DATA
            else:
                print("No ACK for MID, error.")
                state = State.ERROR

        elif state == State.SEND_END:
            checksum = calculate_checksum(serialized_data)
            end_packet = create_end_packet(checksum)
            sock.sendall(end_packet)
            print(f"Sent END packet with checksum {checksum:#010x}")
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

# --------------------------
# Main function
# --------------------------
def main():
    if len(sys.argv) != 4:
        print("Usage: python transfer_image.py <image_file> <ESP32_IP> <port>")
        sys.exit(1)

    image_file = sys.argv[1]
    esp32_ip = sys.argv[2]
    port = int(sys.argv[3])

    # Create TCP socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        print(f"Connecting to {esp32_ip}:{port}...")
        sock.connect((esp32_ip, port))
        print("Connected to ESP32.")

        # Run the finite state machine for image transfer
        fsm_image_transfer(sock, image_file)

if __name__ == "__main__":
    main()


# python send_eth_image.py akshath_face_tdm.png 192.168.0.50 8080
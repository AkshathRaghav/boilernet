import socket

def send_packet(dest_ip, port, message):
    """Send a UDP packet over Ethernet."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create UDP socket
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    try:
        sock.sendto(message.encode(), (dest_ip, port))
        print(f"Sent packet to {dest_ip}:{port}")
    except Exception as e:
        print(f"Error sending packet: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    DEST_IP = "192.168.1.5"
    PORT = 5005  
    MESSAGE = "Hello from Python!"

    send_packet(DEST_IP, PORT, MESSAGE)

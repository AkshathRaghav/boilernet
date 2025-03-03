import socket

TCP_IP = "192.168.0.50"  # ESP32 static IP
TCP_PORT = 8080
MESSAGE = b"Hello Gautam, Gokul and Aneesh"

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the ESP32 TCP server
sock.connect((TCP_IP, TCP_PORT))

# Send the message
sock.sendall(MESSAGE)

# Wait for the response (adjust buffer size if needed)
response = sock.recv(1024)
print("Received from ESP32:", response.decode())

# Close the connection
sock.close()

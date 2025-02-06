import serial
import time

# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

# Open the file to send
file_path = "test.txt"
file_path = "/home/araviki/workbench/boilernet/internal_switches/spi_test/README.md"
file_path = "/home/araviki/workbench/boilernet/internal_switches/spi_test/QualificationsScreenshot.webp"
chunk_size = 1024  # Must match ESP32 buffer size

try:
    print(file_path)
    # Step 1: Send _START_ command
    ser.write(b"_START_")
    print("Sent: _START_")

    # Wait for ACK from ESP32
    ack = ser.read(3)
    if ack.decode().strip() != "ACK":
        print("Received: ", ack.decode().strip())
        print("ESP32 did not acknowledge start. Exiting.")
        exit()

    with open(file_path, "rb") as f:
        while True:
            # Read 1024 bytes from the file
            data = f.read(chunk_size)
            if not data:
                break  # Exit when EOF is reached

            # Send data in USB packet-sized chunks (64 bytes)
            for i in range(0, len(data), 64):
                ser.write(data[i:i+64])
                time.sleep(0.01)  # Ensure buffer processing

            # Wait for ACK after each chunk
            ack = ser.read(3)
            if ack.decode().strip() != "CCK":
                print("Received: ", ack.decode().strip())
                print("ESP32 did not acknowledge data. Exiting.")
                exit()

    # Step 3: Send _END_ command
    ser.write(b"_END_")
    print("Sent: _END_")

    # Wait for final ACK
    ack = ser.read(3)
    if ack.decode().strip() == "ECK":
        print("File transfer completed successfully!")

except Exception as e:
    print(f"Error: {e}")

finally:
    ser.close()
    print("Serial connection closed.")

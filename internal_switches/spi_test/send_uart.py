import serial
import time
import os 

# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)


file_path = "test.txt"
# file_path = "/home/araviki/workbench/boilernet/internal_switches/spi_test/README.md"
# file_path = "/home/araviki/workbench/boilernet/internal_switches/spi_test/QualificationsScreenshot.webp"
chunk_size = 1024  # Must match ESP32 buffer size


mode = 0
chunk_size = 64

if mode: 
    try:
        print(file_path)
        # Step 1: Send _START_ command
        ser.write(b"_START_WRITE_")
        print("Sent: _START_WRITE_")

        # Wait for ACK from ESP32
        ack = ser.read(3)
        if ack.decode().strip() != "ACK":
            print("Want: ACK -> Received: |", ack.decode().strip(), "|")
            print("ESP32 did not acknowledge start. Exiting.")
            exit()

        with open(file_path, "rb") as f:
            while True:
                data = f.read(chunk_size)
                if not data:
                    break  

                for i in range(0, len(data), 64):
                    ser.write(data[i:i+64])
                    time.sleep(0.01)  

                ack = ser.read(3)
                if ack.decode().strip() != "CCK":
                    print("Want: CCK -> Received: |", ack.decode().strip(), "|")
                    print("ESP32 did not acknowledge data. Exiting.")
                    exit()

        # Step 3: Send _END_ command
        ser.write(b"_END_WRITE_")
        print("Sent: _END_WRITE_")

        # Wait for final ACK
        ack = ser.read(3)
        if ack.decode().strip() == "ECK":
            print("File transfer completed successfully!")
        else: 
            print("Want: ECK -> Received: |", ack.decode().strip(), "|")
            print("ESP32 did not acknowledge. Exiting.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        ser.close()
        print("Serial connection closed.")
else: 
    try:
        ser.write(b"_START_GET_")
        print("Sent: _START_GET_")

        # Wait for ACK from ESP32
        ack = ser.read(3)
        if ack.decode().strip() != "ACK":
            print(f"Want: ACK -> Received: |{ack.decode().strip()}|")
            print("ESP32 did not acknowledge start. Exiting.")
            exit()

        total_received = 0

        with open(file_path, "wb") as f:
            while True:
                data = ser.read(chunk_size)
                if not data:
                    break  

                f.write(data)
                f.flush()
                total_received += len(data)


                if b"_END_GET_" in data:
                    print("ESP32 has completed file transfer.")
                    break
                else: 
                    ser.write(b"CCK")

                # else: 
                    # print(data)

        ser.write(b"_CONFIRM_")
        esp32_response = ser.read(10)
        esp32_response = esp32_response.decode().strip()
        expected_size = int(esp32_response) if esp32_response.isdigit() else -1

        # if total_received == expected_size:
        #     print(f"File transfer completed successfully! Received {total_received} bytes.")
        # else:
        #     print(f"File corruption detected! Expected {expected_size}, received {total_received}. Deleting file.")
        #     os.remove(file_path)

    except Exception as e:
        print(f"Error: {e}")
        if os.path.exists(file_path):
            os.remove(file_path)

    finally:
        ser.close()
        print("Serial connection closed.")
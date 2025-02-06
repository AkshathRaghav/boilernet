import serial
import time

# Open serial connection (Update '/dev/ttyUSB0' for Linux/macOS, or 'COM3' for Windows)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)

try:
    while True:
        # Send data to ESP32 (ensure it matches what ESP32 expects)
        data_out = b"ESP32\n"  # No '!' to match ESP32 checking for "ESP32"
        ser.write(data_out)
        print(f"Sent: {data_out.decode().strip()}")

        response = b""  
        while len(response) < 4:
            chunk = ser.read(1)  # Read one byte at a time
            if chunk:
                response += chunk
                print(f"Received chunk: {chunk.decode()}")  # Log the chunk received
            else:
                break  

        if len(response) == 4:
            decoded_response = response.decode().strip()
            print(f"Full response received: {decoded_response}")

            if decoded_response == "ACK":
                print("Completed!")
                break
            elif decoded_response == "NACK":
                print("Error!")
                break
        else:
            print("Invalid response, retrying...")

        time.sleep(1)  # Adjust delay between transmissions

except KeyboardInterrupt:
    print("\nUser interrupted communication.")

except Exception as e:
    print(f"Error: {e}")

finally:
    ser.close()
    print("Serial connection closed.")

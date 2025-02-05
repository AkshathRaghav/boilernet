import serial
import time

# Open serial connection
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Send data to ESP32
data = b"Hello ESP32!\n"
ser.write(data)
print("Data sent!")

# Close the connection
ser.close()

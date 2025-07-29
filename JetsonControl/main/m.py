import serial
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print("Port opened successfully")
    ser.close()
except Exception as e:
    print(f"Error opening serial port: {e}")
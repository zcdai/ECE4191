import serial
import time

try:
    ser = serial.Serial(
        port='/dev/serial0',
        baudrate=9600,
        timeout=1
    )
    print("Serial port opened successfully")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

def send_commands(cmd1, cmd2):
    ser.write(f'{cmd1}{cmd2}\n'.encode('utf-8'))
    ser.flush()  # Ensure the buffer is clear
    print(f"Sent: {cmd1}, {cmd2}")
    time.sleep(0.5)

import serial
import time
import threading

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

def stop_motors():
    ser.write("R0L0\n".encode('utf-8'))
    ser.flush()  # Ensure the buffer is clear
    print("Sent stop command: R0, L0")

def send_commands(cmd1, cmd2):
    ser.write(f'{cmd1}{cmd2}\n'.encode('utf-8'))
    ser.flush()  # Ensure the buffer is clear
    print(f"Sent: {cmd1}, {cmd2}")

def input_listener():
    while True:
        user_input = input("Enter 'F', 'B', 'L', or 'R' (or 'Q' to quit): ").upper()
        if user_input == 'F':
            send_commands('R0.8', 'L0.8')
        elif user_input == 'B':
            send_commands('R-0.8', 'L-0.8')
        elif user_input == 'L':
            send_commands('R0.8', 'L-0.8')
        elif user_input == 'R':
            send_commands('R-0.8', 'L0.8')
        elif user_input == 'Q':
            stop_motors()
            print("Exiting program.")
            break
        else:
            print("Invalid input. Please enter 'F', 'B', 'L', or 'R'.")
            continue
        # Start a timer to stop the motors after 3 seconds
        timer = threading.Timer(3.0, stop_motors)
        timer.start()

if __name__ == "__main__":
    input_listener()
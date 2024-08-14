import serial
import time
import threading

try:
    ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate=9600,
        timeout=1
    )
    print("Serial port opened successfully")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

def stop_motors():
    ser.write("R0".encode('utf-8'))
    time.sleep(0.2)  # Short pause before sending the second stop command
    ser.flush()  # Ensure the buffer is clear
    ser.write("L0".encode('utf-8'))
    print("Sent stop command: R0, L0")

def send_commands(cmd1, cmd2):
    ser.write(cmd1.encode('utf-8'))
    time.sleep(0.2)  # Short pause before sending the second command
    ser.flush()  # Ensure the buffer is clear
    ser.write(cmd2.encode('utf-8'))
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
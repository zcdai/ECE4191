import serial
import time

# Initialize serial communication (make sure the port matches your setup)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Replace ttyUSB0 with the actual port
time.sleep(2)  # Give some time to establish the connection

pins = 4, 26

def send_command(dir='F', ticks='500'):
    command = f"{dir}{ticks}"
    print(f"Sending: {command}")
    ser.write((command + '\n').encode())  # Send the command over serial
    _wait_for_confirmation(command)


def _wait_for_confirmation(expected_command):
    confirmations_needed = 4  # Arduino sends each confirmation 4 times
    confirmation_count = 0
    while confirmation_count < confirmations_needed:
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            if received_data == expected_command:
                confirmation_count += 1   
                
    print(f"Confirmation {confirmation_count}/{confirmations_needed} received for {expected_command}")

def 
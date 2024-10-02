import serial
import time
import RPi.GPIO as GPIO

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
TRIG_PIN = 4
ECHO_PIN = 26

# Set up GPIO pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    # Ensure the trigger pin is set low
    GPIO.output(TRIG_PIN, False)
    time.sleep(2)  # Give the sensor time to settle
    # Send a 10 microsecond pulse to the trigger pin
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    # Measure the time for the echo pin to go high and then low
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    # Calculate the duration of the pulse
    pulse_duration = pulse_end - pulse_start
    # Distance is speed of sound (34300 cm/s) divided by 2 (for round-trip) and multiplied by pulse_duration
    distance = pulse_duration * 17150
    return round(distance, 2)

# Initialize serial communication (make sure the port matches your setup)
# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Replace ttyUSB0 with the actual port
# time.sleep(2)  # Give some time to establish the connection



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

if __name__ == '__main__':
    try:
        while True:
            dist = get_distance()
            print(f"Distance: {dist} cm")
            time.sleep(1)  # Delay between measurements
    except KeyboardInterrupt:
        print("Measurement stopped by user")
        GPIO.cleanup()
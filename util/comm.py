import serial
import time
import RPi.GPIO as GPIO

GPIO.cleanup()
# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Initialize serial communication (make sure the port matches your setup)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Replace ttyUSB0 with the actual port
time.sleep(2)  # Give some time to establish the connection

# Define GPIO pins
TRIG_PIN = 4
ECHO_PIN = 26
BOOM_PIN = 18
SCOOP_PIN = 19 

# Set up GPIO pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(BOOM_PIN, GPIO.OUT)
GPIO.setup(SCOOP_PIN, GPIO.OUT)

pwm_boom = GPIO.PWM(BOOM_PIN, 50)
pwm_scoop = GPIO.PWM(SCOOP_PIN, 50)
pwm_boom.start(0)
pwm_scoop.start(0)


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
    out = round(distance, 2)/100
    print(f'Distance to box is:{out}')
    return out



def set_servo_angle(angle, boom):
    """
    Set the servo to the specified angle.
    
    :param angle: Angle to set the servo to (0 to 180 degrees)
    """
    # Duty cycle calculation for the servo
    duty_cycle = 2 + (angle / 18)  # 2% to 12% duty cycle for 0 to 180 degrees
    if boom:
        pwm_boom.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)  # Allow time for the servo to move to the position
        pwm_boom.ChangeDutyCycle(0)  # Stop sending the PWM signal to avoid jitter
    else:
        pwm_scoop.ChangeDutyCycle(duty_cycle)
        duty_cycle -= 6
        duty_cycle = max(duty_cycle, 2)
        time.sleep(1.75)  # Allow time for the servo to move to the position
        pwm_scoop.ChangeDutyCycle(0)  # Stop sending the PWM signal to avoid jitter


set_servo_angle(0, False)
set_servo_angle(0, True)

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
    while True:
        get_distance()
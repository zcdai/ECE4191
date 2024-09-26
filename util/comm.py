import time
import RPi.GPIO as GPIO

PERIOD = 0.05
GPIO.setmode(GPIO.BCM)

class Motor:
    def __init__(self, signal, dir):
        self.signal = signal
        self.dir = dir
        self._enable()

    def _enable(self):
        GPIO.setup(self.signal, GPIO.OUT)
        GPIO.setup(self.dir, GPIO.OUT)

    def _pwm(self, pin):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(PERIOD)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(PERIOD)

    def forward(self):
        GPIO.output(self.dir, GPIO.HIGH)
        for _ in range(10000):
            self._pwm(self.signal)

    def backward(self):
        GPIO.output(self.dir, GPIO.LOW)
        for _ in range(1000):
            self._pwm(self.signal)

    def stop(self):
        GPIO.output(self.signal, GPIO.LOW)

L_MOTOR = Motor(13, 17)
R_MOTOR = Motor(12, 27)

def straight_drive(dir='F', dist=1):
    if dir == 'F':
        L_MOTOR.forward()
        R_MOTOR.forward()
    else:
        L_MOTOR.backward()
        R_MOTOR.backward()

    start_time = time.time()
    while time.time() - start_time < dist:
        L_MOTOR._pwm(L_MOTOR.signal)
        R_MOTOR._pwm(R_MOTOR.signal)

    L_MOTOR.stop()
    R_MOTOR.stop()

def rotate(dir='L', angle=90):
    if dir == 'L':
        L_MOTOR.backward()
        R_MOTOR.forward()
    else:
        L_MOTOR.forward()
        R_MOTOR.backward()

    start_time = time.time()
    while time.time() - start_time < angle:
        L_MOTOR._pwm(L_MOTOR.signal)
        R_MOTOR._pwm(R_MOTOR.signal)

    L_MOTOR.stop()
    R_MOTOR.stop()

if __name__ == '__main__':
    GPIO.setwarnings(False)
    straight_drive('F', 1)
    GPIO.cleanup()
 

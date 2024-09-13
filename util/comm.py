import Rpi.GPIO as GPIO
import time

class Motor:
    def __init__(self, signal, dir) -> None:
        self.signal = signal
        self.dir = dir
        self.enable()

    def enable(self):
        GPIO.setup(self.signal, GPIO.OUT)
        GPIO.setup(self.dir, GPIO.OUT)

    def forward(self):
        GPIO.output(self.dir, GPIO.HIGH)
        GPIO.output(self.signal, GPIO.HIGH)

    def backward(self):
        GPIO.output(self.dir, GPIO.LOW)
        GPIO.output(self.signal, GPIO.HIGH)
    
    def stop(self):
        GPIO.output(self.signal, GPIO.LOW)

L_MOTOR = Motor(12, 17)
R_MOTOR = Motor(13, 27)
PERIOD = 0.01


def straight_drive(dir='F', dist=1):
    if dir == 'F':
        L_MOTOR.forward()
        R_MOTOR.forward()
    else:
        L_MOTOR.backward()
        R_MOTOR.backward()
    time.sleep(dist)
    L_MOTOR.stop()
    R_MOTOR.stop()
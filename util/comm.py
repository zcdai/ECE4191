import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class Motor:
    def __init__(self, signal, dir) -> None:
        self.signal = signal
        self.dir = dir
        self._enable()

    def _pwm(self, pin):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(PERIOD)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(PERIOD)

    def _enable(self):
        GPIO.setup(self.signal, GPIO.OUT)
        GPIO.setup(self.dir, GPIO.OUT)

    def forward(self):
        GPIO.output(self.dir, GPIO.HIGH)
        for _ in range(100):
            self._pwm(self.signal)

    def backward(self):
        GPIO.output(self.dir, GPIO.LOW)
        self._pwm(self.signal)    

    def stop(self):
        GPIO.output(self.signal, GPIO.LOW)

L_MOTOR = Motor(12, 17)
R_MOTOR = Motor(13, 27)
PERIOD = 0.001


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


if __name__ == '__main__':
    L_MOTOR.forward()
    R_MOTOR.forward()
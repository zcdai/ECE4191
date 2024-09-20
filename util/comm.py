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
PERIOD = 0.05

class MotorPair:
    def __init__(self, l, r) -> None:
        pass

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
    # GPIO.setup(12, GPIO.OUT)
    # GPIO.setup(13, GPIO.OUT)
    # def _pwm():
    #     GPIO.output(12, GPIO.HIGH)
    #     GPIO.output(13, GPIO.HIGH)
    #     time.sleep(PERIOD)
    #     GPIO.output(12, GPIO.LOW)
    #     GPIO.output(13, GPIO.HIGH)
    #     time.sleep(PERIOD)
    # for _ in range(10000):
    #     _pwm()

    # R_MOTOR.forward()
    L_MOTOR.backward()
 

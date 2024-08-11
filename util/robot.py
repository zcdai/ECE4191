import numpy as np
import requests
import cv2

class DriveCommand:
    # forward_speed: [0, 1], float
    # turn_speed: [-1, 1], float
    def __init__(self, forward_speed, turn_speed=0):
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed


class PenguinPi:
    def __init__(self):
        self.wheel_vel = [0, 0]



    def set_velocity(self, drive_cmd: DriveCommand, speed=100, turning_speed=20): 
        l_vel = drive_cmd.forward_speed*speed - drive_cmd.turn_speed*turning_speed
        r_vel = drive_cmd.forward_speed*speed + drive_cmd.turn_speed*turning_speed
        self.wheel_vel = [l_vel, r_vel]

        return l_vel, r_vel

    def get_image(self):
        pass
        # get image from camera and turn it into a processable image
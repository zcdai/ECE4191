import numpy as np


class DriveCommand:
    # forward_speed: [0, 1], float
    # turn_speed: [-1, 1], float
    def __init__(self, forward_speed, turn_speed=0):
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed

class Position:
    def __init__(self, x, y):
        # x is length deep, y is length lateral
        self.x = x
        self.y = y


class PenguinPi:
    def __init__(self):
        self.wheel_vel = [0, 0]
        self.pos = Position(0, 0)
        self.angle = 0
        self.ball_pos = []

    def get_image(self):
        # get image from camera and save positions of any balls
        # check if balls are a reasonable distance away (< hypotenuse of tennis quadrant)
        # maybe sort by distance? longest to shortest
        pass

    def get_closest_ball(self):
        try:
            return self.ball_pos.pop()
        except IndexError:
            return None
        


    def set_angle(self, angle):
        if angle == self.angle:
            return
        angle_delta = angle - self.angle
        self.rotate(angle_delta)

        self.angle = angle

    def primitive_path(self, new_pos):
        # calculate the primitive path
        if new_pos.x > self.pos.x:
            self.set_angle(0)
            x_delta = new_pos.x - self.pos.x
            self.drive(x_delta)

        pass

    def rotate(self, angle):
        # rotate the robot by angle degrees
        pass

    def drive(self, distance):
        # drive forward set distance
        pass

    def probe(self):
        # search for balls in vicinity
        self.get_image()

        while len(self.ball_pos) == 0:
            self.rotate(45) # TODO: find camera FOV and good rotation value
            self.get_image()

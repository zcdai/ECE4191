import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Detector import Detector
from TennisBallPose import estimate_pose
import os
import cv2
from comm import send_commands


class DriveCommand:
    # forward_speed: [0, 1], float
    # turn_speed: [-1, 1], float
    def __init__(self, forward_speed, turn_speed=0):
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed

class BallerRover():
    def __init__(self, pos=[0, 0], angle=0, diameter=0.2286):
        self.origin = pos
        self.pos = pos
        self.angle = angle
        self.ball_pos = []
        self.diameter = diameter


    def get_image(self):
        # get image from camera and save positions of any balls
        # check if balls are a reasonable distance away (< hypotenuse of tennis quadrant)
        # maybe sort by distance? longest to shortest

        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Read in camera matrix
        fileK = f'{script_dir}/../Params/intrinsic.txt'
        camera_matrix = np.loadtxt(fileK, delimiter=',')

        # Initialize YOLO model
        model_path = f'{script_dir}/../YOLO/Model/best (1).pt'
        yolo = Detector(model_path)

        # Open video capture (0 for default camera)
        cap = cv2.VideoCapture(0)

        # Capture a single image
        ret, frame = cap.read()

        cap.release() 

        bounding_boxes, bbox_img = yolo.detect_single_image(frame)
        robot_pose = [self.x, self.y, self.angle]

        target_poses = []

        for detection in bounding_boxes:
            target_pose = estimate_pose(camera_matrix, detection, robot_pose)
            print(f"Detected {detection[0]} at {target_pose}")
            target_poses.append(target_pose)

        return target_poses

    def get_closest_ball(self):
        try:
            return self.ball_pos.pop()
        except IndexError:
            return None

    def primitive_path(self, new_pos):
        if new_pos[0] > self.pos[0]:
            self.set_angle(0)
            x_delta = new_pos[0] - self.pos[0]
            self.drive(x_delta)
        else:
            self.set_angle(180)
            x_delta = self.pos[0] - new_pos[0]
            self.drive(x_delta)

        if new_pos[1] > self.pos[1]:
            self.set_angle(270)
            y_delta = new_pos[1] - self.pos[1]
            self.drive(y_delta)
        else:
            self.set_angle(90)
            y_delta = self.pos[1] - new_pos[1]
            self.drive(y_delta)

        self.pos = new_pos

    # TODO: There is a problem with calculating the path, since the robot moves during rotation, it is not a simple trigonometric calculation
    def direct_path(self, new_pos):
        x_delta = new_pos[0] - self.pos[0]
        y_delta = new_pos[1] - self.pos[1]
        angle = np.arctan2(y_delta, x_delta) * 180 / np.pi
        self.set_angle(angle)
        distance = np.sqrt(x_delta ** 2 + y_delta ** 2)
        self.drive('F', distance)
        self.pos = new_pos

    def return_to_origin(self):
        self.direct_path(self.origin)

    def set_angle(self, angle):
        angle = angle % 360  # ensure angle is within 0 to 360 range
        angle_delta = angle - self.angle
        self.rotate(angle_delta)

    def rotate(self, angle_delta):
        # rotate the robot by angle degrees
        # positive angle is CCW
        if angle_delta == 0:
            return
        if angle_delta > 180:
            angle_delta -= 360
        elif angle_delta < -180:
            angle_delta += 360

        constant = 0.528

        if angle_delta < 0:
            self.drive('R', -angle_delta * constant)
            pivot_point = self.x + np.cos(self.angle), self.y + np.sin(self.angle)


        else:   
            self.drive('L', angle_delta * constant)
            pivot_point = self.x - np.cos(self.angle), self.y - np.sin(self.angle)

        self.angle += angle_delta
        self.pos = self._rotate_arnd_point(angle_delta, pivot_point)

    """Calculates the new position of the robot after rotating around a pivot point
    This is only during rotation, where the bot rotates around one of the wheels"""
    def _rotate_arnd_point(self, angle_delta, pivot_point): 
        x_offset, y_offset = self.x - pivot_point[0], self.y - pivot_point[1]
        x_new, y_new = x_offset * np.cos(angle_delta) - y_offset * np.sin(angle_delta), x_offset * np.sin(angle_delta) + y_offset * np.cos(angle_delta)
        return x_new + pivot_point[0], y_new + pivot_point[1]

    def drive(self, direction='F', distance=1):
        send_commands(f"{direction}", f"{distance}")


    def probe(self):
        # skip if there are balls to path for
        if len(self.ball_pos) > 0:
            return
        # search for balls in vicinity
        self.get_image()

        while len(self.ball_pos) == 0:
            self._rotate(45)  # TODO: find camera FOV and good rotation value
            self.get_image()

    def check_contact(image):
        pass
        # if ball is in pickup range

    def check_valid_ball(image):
        pass
        # if ball is on screen and close enough to go path to

    def nav2ball(image):
        forward_speed = 0
        turn_speed = 0
        pass
        return DriveCommand(forward_speed, turn_speed)
        # _rotate bot to center the ball in the image, and approach using pid?

    def pickup():
        pass
        # pickup the ball
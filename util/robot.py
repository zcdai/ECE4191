import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Detector import Detector
from TennisBallPose import estimate_pose
import os
import cv2
import threading
from comm import send_commands, stop_motors


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


class BallerRover():
    def __init__(self):
        self.wheel_vel = [0, 0]
        self.pos = Position(0, 0)
        self.angle = 0
        self.ball_pos = []


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
        robot_pose = np.append(self.pos, self.angle)

        target_poses = []
        distances = []

        for detection in bounding_boxes:
            target_pose = estimate_pose(camera_matrix, detection, robot_pose)
            print(f"Detected {detection[0]} at {target_pose}")
            target_poses.append(target_pose)

            # Calculate distance from the robot to the target
            dx = target_pose[0] - self.x
            dy = target_pose[1] - self.y
            distance = np.sqrt(dx**2 + dy**2)
            distances.append(distance)
            # Combine target poses with their distances
            targets_with_distances = list(zip(target_poses, distances))

            # Sort targets by distance (closest first)
            sorted_targets_with_distances = sorted(targets_with_distances, key=lambda x: x[1])

            # Extract sorted target poses
            sorted_target_poses = [target[0] for target in sorted_targets_with_distances]

        return sorted_target_poses

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
        else:
            self.set_angle(180)
            x_delta = self.pos.x - new_pos.x
            self.drive(x_delta)

        if new_pos.y > self.pos.y:
            self.set_angle(270)
            y_delta = new_pos.y - self.pos.y
            self.drive(y_delta)
        else:
            self.set_angle(90)
            y_delta = self.pos.y - new_pos.y
            self.drive(y_delta)

        self.pos = new_pos

    def return_to_origin(self):
        self.primitive_path(Position(0, 0))

    def rotate(self, angle):
        # rotate the robot by angle degrees
        # positive angle is CCW
        constant = 0.528
        if angle > 180:
            angle = 360 - angle
            self.drive('R', angle * constant)
        else:
            self.drive('L', angle * constant)

    def drive(self, direction, distance=1):
        send_commands(f"{direction}", f"{distance}")


    def probe(self):
        # skip if there are balls to path for
        if len(self.ball_pos) > 0:
            return
        # search for balls in vicinity
        self.get_image()

        while len(self.ball_pos) == 0:
            self.rotate(45)  # TODO: find camera FOV and good rotation value
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
        # rotate bot to center the ball in the image, and approach using pid?

    def pickup():
        pass
        # pickup the ball
        
if __name__ == "__main__":
    r = BallerRover()
    img = r.get_image()
    print(img)
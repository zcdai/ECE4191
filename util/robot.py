import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Detector import Detector
from TennisBallPose import estimate_pose
import os
import cv2
import time
from comm import send_command


class BallerRover():
    def __init__(self, pos=[0, 0], angle=90, diameter=0.2286):
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
        sorted_target_poses = []

        # Read in camera matrix
        fileK = f'{script_dir}/../Params/intrinsic.txt'
        camera_matrix = np.loadtxt(fileK, delimiter=',')

        # Initialize YOLO model
        model_path = f'{script_dir}/../YOLO/Model/best.pt'
        yolo = Detector(model_path)

        # Open video capture (0 for default camera)
        cap = cv2.VideoCapture(0)

        # Capture a single image
        ret, frame = cap.read()

        cap.release() 

        bounding_boxes, bbox_img = yolo.detect_single_image(frame, conf_threshold=0.7)
        robot_pose = [self.pos[0], self.pos[1], self.angle]

        target_poses = []
        distances = []

        for detection in bounding_boxes:
            target_pose, _ = estimate_pose(camera_matrix, detection, robot_pose)
            print(f"Detected {detection[0]} at {target_pose}")
            target_poses.append(target_pose)

            # Calculate distance from the robot to the target
            dx = target_pose[0] - self.pos[0]
            dy = target_pose[1] - self.pos[1]
            distance = np.sqrt(dx**2 + dy**2)
            distances.append(distance)
            # Combine target poses with their distances
            targets_with_distances = list(zip(target_poses, distances))

            # Sort targets by distance (closest first)
            sorted_targets_with_distances = sorted(targets_with_distances, key=lambda x: x[1])

            # Extract sorted target poses
            sorted_target_poses = [target[0] for target in sorted_targets_with_distances]
            if len(sorted_target_poses) > 2:
                # Delete all elements except the first one
                del sorted_target_poses[1:]
            
            dist_y = sorted_target_poses[0][1] - self.pos[1]
            if dist_y > 2:
                sorted_target_poses[0][1] = 2

        return sorted_target_poses

    def get_closest_ball(self):
        try:
            return self.ball_pos.pop()
        except IndexError:
            return None
    
    def direct_path(self, new_pos):
        x_delta = new_pos[0] - self.pos[0]
        y_delta = new_pos[1] - self.pos[1]
        angle = -np.arctan2(x_delta, y_delta) * 180 / np.pi
        self.set_angle(angle)
        distance = np.sqrt(x_delta ** 2 + y_delta ** 2) * 0.9   # scale down to allow for centering
        self.drive('F', distance)

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


        if angle_delta < 0:
            self.drive('R', -angle_delta)

        else:   
            self.drive('L', angle_delta)


        self.angle += angle_delta
        self.angle = self.angle % 360

    def drive(self, direction='F', distance=1):
        #TODO: find constant for forward and rotation
        forward_const = 1200
        rotation_const = 647/180
        if direction == 'R' or direction == 'L':
            ticks = distance*rotation_const
        else:
            ticks = distance*forward_const


        send_command(direction, ticks)
        if direction == 'F':
            dx = distance * np.cos(np.radians(self.angle))
            dy = distance * np.sin(np.radians(self.angle))
            self.pos = [self.pos[0] + dx, self.pos[1] + dy]

        if direction == 'B':
            dx = distance * np.cos(np.radians(self.angle))
            dy = distance * np.sin(np.radians(self.angle))
            self.pos = [self.pos[0] - dx, self.pos[1] - dy]


    def probe(self):
        for angle in range(0, 360, 45):  # Rotate in 45 degree increments
            self.set_angle(angle)
            balls = self.get_image()
            if balls:
                self.ball_pos.extend(balls)



    def center_ball(self):
        """Get a image of the ball once close to it, center the ball in the image"""
        balls = self.get_image()
        pass

    def nav2ball(self, ball_pos):
        ball_x, ball_y = ball_pos
        bot_x, bot_y = self.pos
        x_delta, y_delta = ball_x - bot_x, ball_y - bot_y
        if abs(ball_x) > abs(ball_y):
            self.direct_path([ball_x, bot_y])
            if y_delta > 0:
                self.set_angle(90)
            else:
                self.set_angle(-90)
        else:
            self.direct_path([bot_x, ball_y])
            if x_delta > 0:
                self.set_angle(0)
            else:
                self.set_angle(180)

        new_pos = self.get_image()
        self.primitive_path(new_pos[0])


    def pickup():
        pass
        # pickup the ball

if __name__ == '__main__':
    bot = BallerRover()
    while True:
        print(bot.get_image())

import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Detector import Detector
from TennisBallPose import estimate_pose
import os
import cv2
import time
from comm import get_distance, send_command, set_servo_angle
from picamera2 import Picamera2


class BallerRover():
    def __init__(self, pos=[0, 0], angle=90, bottom_left=True):
        self.origin = pos
        self.pos = pos
        self.angle = angle
        self.ball_pos = []
        self.court_boundary = [[0, 4], [0, 5.4]]
        self.deposit_pos = [3.8, 5.4]
        self.deposit_angle = 180

        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(main={"size": (1280, 720),'format': 'RGB888'})
        self.camera.configure(config)


        # Start the camera
        self.camera.start()

        print("Initialized BallerRover")

    def get_image(self, force_local=False):
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
        frame = self.camera.capture_array()

        bounding_boxes, bbox_img = yolo.detect_single_image(frame, conf_threshold=0.7)
        robot_pose = [self.pos[0], self.pos[1], self.angle]
        if force_local:
            robot_pose = [0, 0, 90]

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

        return sorted_target_poses

    def get_closest_ball(self):
        try:
            return self.ball_pos.pop()
        except IndexError:
            return None
    
    def direct_path(self, new_pos, shortstop=False):
        x_delta = new_pos[0] - self.pos[0]
        y_delta = new_pos[1] - self.pos[1]
        angle = np.arctan2(y_delta, x_delta)
        self.set_angle(np.degrees(angle))
        distance = np.hypot(x_delta, y_delta)
        if shortstop:
            distance *= 0.9   # scale down to allow for centering
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
        balls = self.get_image(True)
        if balls:  
            target_ball = balls[0]
            x_obj, y_obj = target_ball
            angle_to_object = np.arctan2(y_obj, x_obj)
            self.rotate(np.degrees(angle_to_object))
        else:
            self.probe()

    def pickup_ball(self):
        set_servo_angle(180, False)
        time.sleep(2)
        set_servo_angle(0, False)        

    def lift_boom(self):
        set_servo_angle(180, True)
        time.sleep(4)
        set_servo_angle(90, True)

    def deposit_ball(self):
        self.direct_path(self.deposit_pos)
        self.set_angle(self.deposit_angle)
        self.box_reverse()
        self.lift_boom()
        self.direct_path(2, 3)

    def box_reverse(self):
        # find distance to box
        distance = get_distance() * 0.9
        while distance > 0.03:
            self.drive('B', distance)
            distance = get_distance() * 0.9

if __name__ == '__main__':
    bot = BallerRover()
    bot.direct_path([1,0])
    bot.direct_path([1,1])
    bot.direct_path([0,1])
    bot.return_to_origin()




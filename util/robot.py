import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Detector import Detector
from TennisBallPose import estimate_pose
import os
import cv2
import time
from comm import send_commands, read


class BallerRover():
    def __init__(self, pos=[0, 0], angle=90, diameter=0.2286):
        self.origin = pos
        self.pos = pos
        self.angle = angle
        self.ball_pos = []
        self.diameter = diameter

    def check_stopped(self):
        if read() == 'S':
            return True
        return False


    def get_image(self):
        while not self.check_stopped():
            time.sleep(1)

        # get image from camera and save positions of any balls
        # check if balls are a reasonable distance away (< hypotenuse of tennis quadrant)
        # maybe sort by distance? longest to shortest
        script_dir = os.path.dirname(os.path.abspath(__file__))
        sorted_target_poses = []

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

    def primitive_path(self, new_pos):
        x_delta = new_pos[0] - self.pos[0]
        y_delta = new_pos[1] - self.pos[1]

        running_error = np.array([0.0, 0.0])

        if y_delta > 0:
            running_error += self.test_set_error(90, self.angle)
            resulting_angle = 90

        elif y_delta < 0:
            running_error += self.test_set_error(-90, self.angle)
            resulting_angle = -90

        if x_delta > 0:
            running_error += self.test_set_error(0, resulting_angle)

        elif x_delta < 0:
            running_error += self.test_set_error(180, resulting_angle)

        dy_to_travel = y_delta - running_error[1]
        if dy_to_travel > 0:
            self.set_angle(90)

        elif dy_to_travel < 0:
            self.set_angle(-90)

        self.drive(distance=abs(dy_to_travel))

        dx_to_travel = x_delta - running_error[0]
        if x_delta > 0:
            self.set_angle(0)

        elif x_delta < 0:
            self.set_angle(180)

        self.drive(distance=abs(dx_to_travel))




    # TODO: There is a problem with calculating the path, since the robot moves during rotation, it is not a simple trigonometric calculation
    def direct_path(self, new_pos):
        x_delta = new_pos[0] - self.pos[0]
        y_delta = new_pos[1] - self.pos[1]
        angle = -np.arctan2(x_delta, y_delta) * 180 / np.pi
        self.set_angle(angle)
        distance = np.sqrt(x_delta ** 2 + y_delta ** 2)
        self.drive('F', distance)

    def return_to_origin(self):
        self.primitive_path(self.origin)

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

        c_l = 0.525/180
        c_r = 0.696/180

        pivot_point = self.pos[0] + np.sin(np.radians(self.angle))*self.diameter/2, self.pos[1] - np.cos(np.radians(self.angle))*self.diameter/2

        if angle_delta < 0:
            self.drive('R', -angle_delta * c_r)

        else:   
            self.drive('L', angle_delta * c_l)


        self.angle += angle_delta
        self.angle = self.angle % 360
        self.pos = self.rotate_point(angle_delta, pivot_point)

    """Calculates the new position of the robot after rotating around a pivot point
    This is only during rotation, where the bot rotates around the right wheel"""
    
    def rotate_point(self, angle_delta, pivot_point=[0.1143, 0]):
        x, y = self.pos[0], self.pos[1]
        cx, cy = pivot_point[0], pivot_point[1]
        angle_delta = angle_delta % 360  # ensure angle is within 0 to 360 range
        angle_delta = np.radians(angle_delta)

        # Translate point to origin
        translated_point = np.array([x - cx, y - cy])

        # Rotation matrix
        rotation_matrix = np.array([
            [np.cos(angle_delta), -np.sin(angle_delta)],
            [np.sin(angle_delta),  np.cos(angle_delta)]
        ])

        # Rotate the point
        rotated_point = np.dot(rotation_matrix, translated_point)

        # Translate point back
        x_rotated, y_rotated = rotated_point + np.array([cx, cy])

        return x_rotated, y_rotated
    
    def test_set_error(self, angle, cur_angle):
        angle = angle % 360  # ensure angle is within 0 to 360 range
        cur_angle = cur_angle % 360  # ensure angle is within 0 to 360 range
        angle_delta = angle - cur_angle
        pivot_point = self.pos[0] + np.sin(np.radians(cur_angle))*self.diameter/2, self.pos[1] - np.cos(np.radians(cur_angle))*self.diameter/2
        
        x_err, y_err = self.rotate_point(angle_delta, pivot_point)
        x_err -= self.pos[0]
        y_err -= self.pos[1]

        return x_err, y_err

    def drive(self, direction='F', distance=1):
        send_commands(direction, distance)
        if direction == 'F':
            dx = distance * np.cos(np.radians(self.angle))
            dy = distance * np.sin(np.radians(self.angle))
            self.pos = [self.pos[0] + dx, self.pos[1] + dy]


    def probe(self):
        # skip if there are balls to path for
        if len(self.ball_pos) > 0:
            return
        # search for balls in vicinity
        self.ball_pos.extend(self.get_image())
        while len(self.ball_pos) == 0:
            self.rotate(45)
            self.ball_pos.extend(self.get_image())
            while not self.check_stopped():
                time.sleep(2)

    def check_contact(image):
        pass
        # if ball is in pickup range

    def check_valid_ball(image):
        pass
        # if ball is on screen and close enough to go path to

    def nav2ball(self, ball_pos):
        ball_x, ball_y = ball_pos
        bot_x, bot_y = self.pos
        x_delta, y_delta = ball_x - bot_x, ball_y - bot_y
        if abs(ball_x) > abs(ball_y):
            self.primitive_path([ball_x, bot_y])
            if y_delta > 0:
                self.set_angle(90)
            else:
                self.set_angle(-90)
        else:
            self.primitive_path([bot_x, ball_y])
            if x_delta > 0:
                self.set_angle(0)
            else:
                self.set_angle(180)

        new_pos = self.get_image()
        self.primitive_path(new_pos[0])


    def pickup():
        pass
        # pickup the ball
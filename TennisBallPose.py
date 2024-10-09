# estimate the pose of target objects detected
import numpy as np
import json
import os
import ast
import cv2
from Detector import Detector
import math
from picamera2 import Picamera2


# list of target fruits and vegs types
# Make sure the names are the same as the ones used in your YOLO model
TARGET_TYPES = ['Ball']


def estimate_pose(camera_matrix, obj_info, robot_pose):
    """
    function:
        estimate the pose of a target based on size and location of its bounding box and the corresponding robot pose
    input:
        camera_matrix: list, the intrinsic matrix computed from camera calibration (read from 'param/intrinsic.txt')
            |f_x, s,   c_x|
            |0,   f_y, c_y|
            |0,   0,   1  |
            (f_x, f_y): focal length in pixels
            (c_x, c_y): optical centre in pixels
            s: skew coefficient (should be 0 for PenguinPi)
        obj_info: list, an individual bounding box in an image (generated by get_bounding_box, [label,[x,y,width,height]])
        robot_pose: list, pose of robot corresponding to the image (read from 'lab_output/images.txt', [x,y,theta])
    output:
        target_pose: dict, prediction of target pose
    """
    # read in camera matrix (from camera calibration results)
    focal_length = camera_matrix[0][0]
    # there are 8 possible types of fruits and vegs
    ######### Replace with your codes #########
    # TODO: measure actual sizes of targets [width, depth, height] and update the dictionary of true target dimensions
    target_dimensions_dict = {'Ball': [0.067,0.067,0.067]}
    #########
# estimate target pose using bounding box and robot pose
    target_class = obj_info[0]     # get predicted target label of the box
    target_box = obj_info[1]       # get bounding box measures: [x,y,width,height]
    true_height = target_dimensions_dict[target_class][2]   # look up true height of by class label

    # compute pose of the target based on bounding box info, true object height, and robot's pose
    pixel_height = target_box[3]
    pixel_center = target_box[0]
    distance = true_height/pixel_height * focal_length  # estimated distance between the robot and the centre of the image plane based on height

    x_shift = 640 - pixel_center              # x distance between bounding box centre and centreline in camera view
    theta = np.arctan(x_shift/focal_length)     # angle of object relative to the robot
    
   # relative object location
    distance_obj = distance/np.cos(theta) # relative distance between robot and object
    x_relative = distance_obj * np.cos(theta) # relative x pose
    y_relative = distance_obj * np.sin(theta) # relative y pose
    relative_pose = {'x': x_relative, 'y': y_relative}
    target_pose = []

    # location of object in the world frame using rotation matrix
    delta_x_world = x_relative * np.cos(np.radians(robot_pose[2])) - y_relative * np.sin(np.radians(robot_pose[2]))
    delta_y_world = x_relative * np.sin(np.radians(robot_pose[2])) + y_relative * np.cos(np.radians(robot_pose[2]))
    # add robot pose with delta target pose
    # add robot pose with delta target pose
    target_pose = [robot_pose[0] + delta_x_world, robot_pose[1] + delta_y_world]
    #print(f'delta_x_world: {delta_x_world}, delta_y_world: {delta_y_world}')
    #print(f'target_pose: {target_pose}')
    
    return target_pose, relative_pose

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Read in camera matrix
    fileK = f'{script_dir}/Params/intrinsic.txt'
    camera_matrix = np.loadtxt(fileK, delimiter=',')

    # Initialize YOLO model
    model_path = f'{script_dir}/YOLO/Model/best.pt'
    yolo = Detector(model_path)

    # Initialize the PiCamera
    camera = Picamera2()
    # Configure the camera
    config = camera.create_preview_configuration(main={"size": (1280, 720),'format': 'RGB888'})
    camera.configure(config)

    # Start the camera
    camera.start()

    try:
        while True:
            # Capture a frame
            frame = camera.capture_array()

            bounding_boxes, bbox_img = yolo.detect_single_image(frame)
            robot_pose = [0, 0, 0]  # Assuming robot_pose is [x, y, theta], replace with actual robot pose if available

            for detection in bounding_boxes:
                target_pose = estimate_pose(camera_matrix, detection, robot_pose)
                print(f"Detected {detection[0]} at {target_pose}")

            cv2.imshow('Detected Objects', bbox_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
import cv2
import os
import numpy as np
from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops
from picamera2 import Picamera2, Preview
import time
import time

class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

        self.class_colour = {
            'Ball': (0, 255, 0),
        }

    def detect_single_image(self, img, conf_threshold=0.8):
        """
        function:
            detect target(s) in an image
        input:
            img: image, e.g., image read by the cv2.imread() function
        output:
            bboxes: list of lists, box info [label,[x,y,width,height]] for all detected targets in image
            img_out: image with bounding boxes and class labels drawn on
        """
        bboxes = self._get_bounding_boxes(img, conf_threshold)

        img_out = deepcopy(img)

        # draw bounding boxes on the image
        for bbox in bboxes:
            #  translate bounding box info back to the format of [x1,y1,x2,y2]
            xyxy = ops.xywh2xyxy(bbox[1])
            x1 = int(xyxy[0])
            y1 = int(xyxy[1])
            x2 = int(xyxy[2])
            y2 = int(xyxy[3])
            # draw bounding box
            img_out = cv2.rectangle(img_out, (x1, y1), (x2, y2), self.class_colour[bbox[0]], thickness=2)

            # draw class label
            img_out = cv2.putText(img_out, bbox[0], (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                  self.class_colour[bbox[0]], 2)

        return bboxes, img_out

    def _get_bounding_boxes(self, cv_img, conf_threshold):
        """
        function:
            get bounding box and class label of target(s) in an image as detected by YOLOv8
        input:
            cv_img    : image, e.g., image read by the cv2.imread() function
            model_path: str, e.g., 'yolov8n.pt', trained YOLOv8 model
        output:
            bounding_boxes: list of lists, box info [label,[x,y,width,height]] for all detected targets in image
        """

        # predict target type and bounding box with your trained YOLO

        predictions = self.model.predict(cv_img, imgsz=640, verbose=False)
        # get bounding box and class label for target(s) detected
        bounding_boxes = []
        for prediction in predictions:
            boxes = prediction.boxes
            for box in boxes:
                confidence = box.conf[0]  # Confidence score of the detection
                print(confidence)
                if confidence >= conf_threshold:
                    # bounding format in [x, y, width, height]
                    box_cord = box.xywh[0]
                    box_label = int(box.cls)  # class label of the box

                    bounding_boxes.append([prediction.names[(box_label)], np.asarray(box_cord), confidence])

        return bounding_boxes


if __name__ == '__main__':
    # Initialize the YOLO model
    yolo = Detector('/home/ECE4191/ECE4191/YOLO/Model/best.pt')

    # Set up the folder to save the calibration images
    save_folder ='Model_Images'
    os.makedirs(save_folder, exist_ok=True)

    # Initialize the PiCamera
    camera = Picamera2()
    # Configure the camera
    config = camera.create_preview_configuration(main={"size": (1280, 720),'format': 'RGB888'})
    camera.configure(config)

    # Start the camera
    camera.start()

    image_count = 0
    while True:
        # Capture a frame
        frame = camera.capture_array()

        # Perform object detection
        bboxes, img_out = yolo.detect_single_image(frame, conf_threshold=0.2)

        # Display the output with detected bounding boxes
        cv2.imshow('YOLO Detection', img_out)

        # Press 's' to save the image and object points
        if key == ord('s') and ret:
        image_filename = os.path.join(save_folder, f'image_{image_count:03d}.jpg')
        cv2.imwrite(image_filename, img_out)
        print(f'Saved {image_filename}')
        image_count += 1

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

    # Close any OpenCV windows
    cv2.destroyAllWindows()

    #stop the camera
    camera.stop()
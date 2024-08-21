import cv2
import os
import numpy as np
from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops


class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

        self.class_colour = {
            'Balls': (0, 255, 0),
        }

    def detect_single_image(self, img):
        """
        function:
            detect target(s) in an image
        input:
            img: image, e.g., image read by the cv2.imread() function
        output:
            bboxes: list of lists, box info [label,[x,y,width,height]] for all detected targets in image
            img_out: image with bounding boxes and class labels drawn on
        """
        bboxes = self._get_bounding_boxes(img)

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

    def _get_bounding_boxes(self, cv_img):
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

        predictions = self.model.predict(cv_img, imgsz=320, verbose=False)
        # get bounding box and class label for target(s) detected
        bounding_boxes = []
        for prediction in predictions:
            boxes = prediction.boxes
            for box in boxes:
                # bounding format in [x, y, width, height]
                box_cord = box.xywh[0]

                box_label = int(box.cls)  # class label of the box

                bounding_boxes.append([prediction.names[(box_label)], np.asarray(box_cord)])

        return bounding_boxes


if __name__ == '__main__':
    # Initialize the YOLO model
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yolo = Detector(f'{script_dir}/YOLO/Model/best (1).pt')

    # Open the video capture (0 is usually the default camera)
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Resize the frame to the desired size (e.g., 640x480)
        resized_frame = cv2.resize(frame, (640, 480))

        # Perform object detection
        bboxes, img_out = yolo.detect_single_image(resized_frame)

        # Display the output with detected bounding boxes
        cv2.imshow('YOLO Detection', img_out)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close any OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
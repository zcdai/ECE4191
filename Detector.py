import cv2
import torch
from ultralytics import YOLO

# Load the model with custom weights
model = YOLO('YOLO/Model/yolov8s (1).pt') 
model.fuse()
# Load an image using OpenCV
image = cv2.imread('YOLO/Test_Image/high_light4.jpg')

# Perform object detection
results = model.predict(image)

# Draw bounding boxes on the image
for result in results:
    for box in result.boxes:
        # Get coordinates and class of detected object
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf = box.conf[0].item()
        label = int(box.cls[0])
        
        # Draw the bounding box and label on the image
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, f'Class: {label} Conf: {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Display the image with bounding boxes
cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
import cv2
import os
from picamera2 import Picamera2, Preview

# Set up the folder to save the calibration images
save_folder = 'Model_Images'
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

    # Display the resulting frame
    cv2.imshow('Camera Frame', frame)

    # Check for key press
    key = cv2.waitKey(1) & 0xFF

    # Press 's' to save the image and object points
    if key == ord('s') and ret:
        image_filename = os.path.join(save_folder, f'image_{image_count:03d}.jpg')
        cv2.imwrite(image_filename, frame)
        print(f'Saved {image_filename}')
        image_count += 1

    # Break the loop if 'q' is pressed
    if key == ord('q'):
        break

# When everything done, release the capture and close windows
#stop the camera
camera.stop()
cv2.destroyAllWindows()
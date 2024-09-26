import cv2
import numpy as np
import os
from picamera2 import Picamera2

# Set up the folder to save the calibration images
save_folder = 'calibration_images'
os.makedirs(save_folder, exist_ok=True)

# Define the dimensions of the chessboard
chessboard_size = (10, 7)  # number of inner corners per chessboard row and column
square_size = 15  # Define the size of a square in your defined unit (point, millimeter, etc.)

# Prepare object points based on the real-world dimensions of the chessboard
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3d point in real-world space
imgpoints = []  # 2d points in image plane

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

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
   
    # If found, draw and display the corners
    if ret:
        print(ret)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(frame, chessboard_size, corners2, ret)

    # Display the resulting frame
    cv2.imshow('Camera Calibration', frame)

    # Check for key press
    key = cv2.waitKey(1) & 0xFF

    # Press 's' to save the image and object points
    if key == ord('s') and ret:
        objpoints.append(objp)
        imgpoints.append(corners2)
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

# Perform camera calibration
if image_count > 0:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Save the calibration data
    np.savez('camera_calibration.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)

    print("Calibration completed and saved.")
else:
    print("No images were captured for calibration.")

# Example: Undistort a captured image
if image_count > 0:
    sample_image_path = os.path.join(save_folder, 'image_000.jpg')
    img = cv2.imread(sample_image_path)
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

    # Undistort
    dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

    # Crop the image based on the region of interest (roi)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imshow('Undistorted Image', dst)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Load the calibration data
calibration_data = np.load('camera_calibration.npz')

# Extract the camera matrix and distortion coefficients
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

# Extract rotation and translation vectors
rvecs = calibration_data['rvecs']
tvecs = calibration_data['tvecs']

print(camera_matrix)

# save the intrinsic parameters 
dataDir = "{}/Params/".format(os.getcwd())
print("\nIntrinsic parameters:\n", camera_matrix)
fileNameI = "{}intrinsic.txt".format(dataDir)
np.savetxt(fileNameI, camera_matrix, delimiter=',')

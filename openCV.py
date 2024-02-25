import cv2
import time

# Initialize the camera
camera = cv2.VideoCapture(0)  # Use 0 for the default camera

# Give the camera some time to warm up
time.sleep(2)

while True:
    # Capture a frame from the camera
    ret, frame = camera.read()

    # Display the captured frame
    cv2.imshow("Camera Feed", frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the OpenCV window
camera.release()
cv2.destroyAllWindows()

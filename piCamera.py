import picamera
import cv2
import numpy as np
import time

def convert_pixels_to_inches(pixels, dpi=96):
    # Assuming a standard screen DPI of 96
    inches = pixels / dpi
    return inches

def locate_boxes_with_sizes(frame, target_size_inches, color_lower, color_upper):
    # Convert the frame to the HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask to extract the colored regions
    color_mask = cv2.inRange(hsv_frame, color_lower, color_upper)

    # Find contours in the color mask
    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []

    # Iterate through contours
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the polygon has four vertices (a rectangle) and is of the desired size
        if len(approx) == 4:
            # Get the bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)

            print(w,h) #print width and height

            # Convert pixel dimensions to inches
            width_inches = convert_pixels_to_inches(w)
            height_inches = convert_pixels_to_inches(h)

            # Check if the box is of the desired size
            if (
                target_size_inches - 0.1 < width_inches < target_size_inches + 0.1
                and target_size_inches - 0.1 < height_inches < target_size_inches + 0.1
            ):
                # Draw a bounding box around the rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                boxes.append((x, y, w, h))

    return frame, boxes


def locate_cylinders(frame):
    # Convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise
    blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

    # Use Canny edge detection
    edges = cv2.Canny(blurred_frame, 50, 150)

    # Find contours in the edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cylindrical_objects = []

    # Iterate through contours
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the polygon is approximately circular
        if len(approx) > 7:
            # Draw the contour on the frame
            cv2.drawContours(frame, [contour], 0, (0, 255, 0), 2)

            # Calculate the bounding rectangle and extract height
            x, y, w, h = cv2.boundingRect(contour)
            cylindrical_objects.append((x, y, w, h))

    return frame, cylindrical_objects


# Create a PiCamera object
camera = picamera.PiCamera()

# Set camera resolution (optional)
camera.resolution = (640, 480)

# Set camera rotation (if needed)
# camera.rotation = 180  # Adjust the value based on your camera orientation

# Start the preview (optional)
camera.start_preview()

# Add a delay to allow the camera to adjust to lighting conditions
time.sleep(2)

try:
    while True:
        # Capture a frame from the camera
        image_stream = np.empty((480 * 640 * 3,), dtype=np.uint8)
        camera.capture(image_stream, 'bgr')
        frame = image_stream.reshape((480, 640, 3))

        # Locate purple boxes of 1 inch size
        result_frame, purple_boxes = locate_boxes_with_sizes(
            frame,
            target_size_inches=1.0,
            color_lower=np.array([130, 50, 50]),
            color_upper=np.array([170, 255, 255])
        )
        # Display the result frame
        cv2.imshow("Object Detection", result_frame)

        # Locate brown boxes of 1.5 inches size
        result_frame, brown_boxes = locate_boxes_with_sizes(
            result_frame,
            target_size_inches=1.5,
            color_lower=np.array([10, 50, 50]),
            color_upper=np.array([30, 255, 255])
        )
        # Display the result frame
        cv2.imshow("Object Detection", result_frame)

        # Locate cylindrical objects in the frame
        result_frame, cylinders = locate_cylinders(frame)
        # Display the result frame
        cv2.imshow("Object Detection", result_frame)

        # Print the coordinates of detected boxes
        print("Purple Boxes:", purple_boxes)
        print("Brown Boxes:", brown_boxes)

        # Print the coordinates of detected cylindrical objects
        print("Cylinders:", cylinders)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the preview and close the camera
    camera.stop_preview()
    camera.close()
    cv2.destroyAllWindows()

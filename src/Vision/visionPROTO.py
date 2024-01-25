import cv2
import numpy as np
import os

def load_torus_images(folder_path):
    torus_images = [cv2.imread(os.path.join(folder_path, img)) for img in os.listdir(folder_path)]
    return torus_images

# Function to convert RGB to HSV
def rgb_to_hsv(rgb):
    return cv2.cvtColor(np.uint8([[rgb]]), cv2.COLOR_RGB2HSV)[0][0]

# Function to detect orange torus
def detect_orange_torus(frame, lower_orange_rgb, upper_orange_rgb):
    # Convert RGB values to HSV
    lower_orange_hsv = cv2.cvtColor(np.uint8([[lower_orange_rgb]]), cv2.COLOR_RGB2HSV)[0][0]
    upper_orange_hsv = cv2.cvtColor(np.uint8([[upper_orange_rgb]]), cv2.COLOR_RGB2HSV)[0][0]

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a binary mask for the orange color
    orange_mask = cv2.inRange(hsv_frame, lower_orange_hsv, upper_orange_hsv)

    # Apply morphological operations to improve object shape
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    smoothed_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
    smoothed_mask = cv2.morphologyEx(smoothed_mask, cv2.MORPH_CLOSE, kernel)

    # Find contours of the detected orange torus
    contours, _ = cv2.findContours(smoothed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on size
    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > 100]

    result = cv2.bitwise_and(frame, frame, mask=smoothed_mask)
    return result, smoothed_mask, filtered_contours

# Function to draw a bounding box around the detected target
def draw_bounding_box(frame, contours, color):
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
    return frame

# Function to detect torus shape
def detect_torus(frame, filtered_contours):
    for contour in filtered_contours:
        # Approximate the contour to a polygon
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(approx)

        # Calculate aspect ratio
        aspect_ratio = float(w) / h

        # Set a threshold for aspect ratio to filter out false positives
        aspect_ratio_threshold = 1.5  # Adjust this threshold based on your torus characteristics

        if 0.5 <= aspect_ratio <= aspect_ratio_threshold:
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
            cv2.putText(frame, 'Torus', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame

# Function to estimate distance from camera using triangulation
def estimate_distance(apparent_width, known_width, focal_length):
    distance = (known_width * focal_length) / apparent_width
    return distance

def main():
    # Set the path to the "Assets" folder (Made Universal)
    folder_path = "./src/Assets"

    # Example range for orange color in RGB
    lower_orange_rgb = (200, 50, 0)  # Adjusted lower threshold for orange color
    upper_orange_rgb = (255, 150, 50)  # Adjusted upper threshold for orange color

    # Load torus images
    torus_images = load_torus_images(folder_path)

    # Create windows for displaying the live camera feed, color-only view, bounding box view, and distance view
    cv2.namedWindow("Live Feed", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Color Only View", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Box View", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Distance View", cv2.WINDOW_NORMAL)

    # Open the camera (change 0 to the appropriate camera index if needed [1 or -1 for external cameras])
    cap = cv2.VideoCapture(0)

    # Known physical width of the torus in inches (example)
    known_width_inches = 10  # 10 for Note

    # Known focal length of the camera (example, you need to calibrate this based on your camera)
    focal_length = 320.8

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Detect orange torus in the frame
        color_only_view, mask, filtered_contours = detect_orange_torus(frame.copy(), lower_orange_rgb, upper_orange_rgb)

        # Find contours of the detected orange torus
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on size or other criteria
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > 100]

        # Draw a bounding box around the detected target in the "Box View" window
        box_view = draw_bounding_box(frame.copy(), filtered_contours, (0, 255, 0))
        cv2.imshow("Box View", box_view)

        # Calculate apparent width of the torus in pixels
        if len(filtered_contours) > 0:
            x, y, w, h = cv2.boundingRect(filtered_contours[0])
            apparent_width = w

            # Estimate distance using triangulation
            distance = estimate_distance(apparent_width, known_width_inches, focal_length)
            distance_text = f"Distance: {distance * 2.54:.2f} inches"

            # Display the distance and bounding box in the "Distance View" window
            distance_view = frame.copy()
            cv2.putText(distance_view, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            distance_view_with_box = draw_bounding_box(distance_view, filtered_contours, (0, 0, 255))
            cv2.imshow("Distance View", distance_view_with_box)

        # Display the color-only view
        cv2.imshow("Color Only View", color_only_view)

        # Display the live camera feed
        cv2.imshow("Live Feed", frame)

        # Exit the program when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

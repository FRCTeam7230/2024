import cv2 as cv
import os
import numpy as np
import configparser
import yaml

path = os.path.dirname(os.path.realpath(__file__))
with open(path + '//' + 'config.yaml', 'r') as stream:
    config = yaml.safe_load(stream)

color_preset = config['color_presets'][0]

lOrange = np.array(config['Color']['lowerOrange'], dtype=np.uint8)
uOrange = np.array(config['Color']['upperOrange'], dtype=np.uint8)
def load_torus_images(folder_path):
    torus_images = [cv.imread(os.path.join(folder_path, img)) for img in os.listdir(folder_path)]
    return torus_images

# Function for orange color detection with added smoothing
def detect_orange_torus(frame, lower_orange, upper_orange):
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_frame, lower_orange, upper_orange)

    # Apply Gaussian blur to reduce noise
    blurred_mask = cv.GaussianBlur(mask, (5, 5), 0)

    # Apply morphological operations to improve object shape
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    smoothed_mask = cv.morphologyEx(blurred_mask, cv.MORPH_OPEN, kernel)
    smoothed_mask = cv.morphologyEx(smoothed_mask, cv.MORPH_CLOSE, kernel)

    result = cv.bitwise_and(frame, frame, mask=smoothed_mask)
    return result, smoothed_mask

# Function to draw a bounding box around the detected target
def draw_bounding_box(frame, contours, color):
    for contour in contours:
        x, y, w, h = cv.boundingRect(contour)
        cv.rectangle(frame, (x, y), (x+w, y+h), color, 2)
    return frame

# Function to estimate distance from camera using triangulation
def estimate_distance(apparent_width, known_width, focal_length):
    distance = (known_width * focal_length) / apparent_width
    return distance

# Function to measure the longest side of the bounding box
def measure_longest_side(contour):
    _, _, w, h = cv.boundingRect(contour)
    return max(w, h)

# Main function for live camera feed
def main():
    # Set the path to the "Assets" folder (Made Universal)
    folder_path = "./src/Assets"

    # Example range for orange color in HSV
    lower_orange = lOrange
    upper_orange = uOrange

    # Load torus images
    torus_images = load_torus_images(folder_path)

    # Create windows for displaying the live camera feed, color-only view, bounding box view, and combined view
    cv.namedWindow("Color Only View", cv.WINDOW_NORMAL)
    cv.namedWindow("Box View", cv.WINDOW_NORMAL)
    cv.namedWindow("Distance View", cv.WINDOW_NORMAL)

    # Open the camera (change 0 to the appropriate camera index if needed [1 or -1 for external cameras])
    cap = cv.VideoCapture(0)

    # Known physical width of the torus in inches (example)
    known_width_inches = 10 # change to 10.0 for torus

    # Known focal length of the camera (example, you need to calibrate this based on your camera)
    focal_length = 320.8

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Detect orange torus in the frame
        color_only_view, mask = detect_orange_torus(frame.copy(), lower_orange, upper_orange)

        # Find contours of the detected orange torus
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Filter contours based on size or other criteria
        filtered_contours = [contour for contour in contours if cv.contourArea(contour) > 100]

        # Draw a bounding box around the detected target in the "Box View" window
        box_view = draw_bounding_box(frame.copy(), filtered_contours, (0, 255, 0))
        cv.imshow("Box View", box_view)

        # Calculate apparent width of the torus in pixels
        if len(filtered_contours) > 0:
            x, y, w, h = cv.boundingRect(filtered_contours[0])
            apparent_width = w

            # Estimate distance using triangulation
            distance = estimate_distance(apparent_width, known_width_inches, focal_length)
            distance_text = f"Distance: {distance:.2f} inches"

            # Display the combined view with red-colored bounding box and red text
            combined_view = frame.copy()
            cv.putText(combined_view, distance_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            draw_bounding_box(combined_view, filtered_contours, (0, 0, 255))
            cv.imshow("Distance View", combined_view)

        # Display the color-only view
        cv.imshow("Color Only View", color_only_view)

        # Exit the program when the 'q' key is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
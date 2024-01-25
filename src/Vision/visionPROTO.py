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

def main():
    # Set the path to the "Assets" folder (Made Universal)
    folder_path = "./src/Assets"

    # Example range for orange color in RGB
    lower_orange_rgb = (200, 50, 0)  # Adjusted lower threshold for orange color
    upper_orange_rgb = (255, 150, 50)  # Adjusted upper threshold for orange color

    # Load torus images
    torus_images = load_torus_images(folder_path)

    # Create windows for displaying the live camera feed and torus view
    cv2.namedWindow("Torus View", cv2.WINDOW_NORMAL)

    # Open the camera (change 0 to the appropriate camera index if needed [1 or -1 for external cameras])
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Detect orange torus in the frame
        torus_view, _, filtered_contours = detect_orange_torus(frame.copy(), lower_orange_rgb, upper_orange_rgb)

        # Detect torus shape in the frame
        frame_with_torus = detect_torus(frame.copy(), filtered_contours)

        # Display the results
        cv2.imshow("Torus View", frame_with_torus)

        # Exit the program when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

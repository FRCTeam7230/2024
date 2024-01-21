import cv2
import os

def load_torus_images(folder_path):
    torus_images = [cv2.imread(os.path.join(folder_path, img)) for img in os.listdir(folder_path)]
    return torus_images

# Function for orange color detection with added smoothing
def detect_orange_torus(frame, lower_orange, upper_orange):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

    # Apply Gaussian blur to reduce noise
    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Apply morphological operations to improve object shape
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    smoothed_mask = cv2.morphologyEx(blurred_mask, cv2.MORPH_OPEN, kernel)
    smoothed_mask = cv2.morphologyEx(smoothed_mask, cv2.MORPH_CLOSE, kernel)

    result = cv2.bitwise_and(frame, frame, mask=smoothed_mask)
    return result, smoothed_mask

# Function to draw a bounding box around the detected target
def draw_bounding_box(frame, contours):
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    return frame

# Function to estimate distance from camera using triangulation
def estimate_distance(apparent_width, known_width, focal_length):
    distance = (known_width * focal_length) / apparent_width
    return distance

# Main function for live camera feed
def main():
    # Set the path to the "Assets" folder (Made Universal)
    folder_path = "./src/Assets"

    # Example range for orange color in HSV
    lower_orange = (0, 150, 100)
    upper_orange = (10, 255, 255)

    # Load torus images
    torus_images = load_torus_images(folder_path)

    # Create windows for displaying the live camera feed, color-only view, bounding box view, and distance view
    cv2.namedWindow("Color Only View", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Box View", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Distance View", cv2.WINDOW_NORMAL)

    # Open the camera (change 0 to the appropriate camera index if needed [1 or -1 for external cameras])
    cap = cv2.VideoCapture(0)

    # Known physical width of the torus in centimeters (example)
    known_width = 25.4

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
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on size or other criteria
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > 100]

        # Draw a bounding box around the detected target in the "Box View" window
        box_view = draw_bounding_box(frame.copy(), filtered_contours)
        cv2.imshow("Box View", box_view)

        # Calculate apparent width of the torus in pixels
        if len(filtered_contours) > 0:
            x, y, w, h = cv2.boundingRect(filtered_contours[0])
            apparent_width = w

            # Estimate distance using triangulation
            distance = estimate_distance(apparent_width, known_width, focal_length)
            distance_text = f"Distance: {distance:.2f} cm"

            # Display the distance and bounding box in the "Distance View" window
            distance_view = frame.copy()
            cv2.putText(distance_view, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            distance_view_with_box = draw_bounding_box(distance_view, filtered_contours)
            cv2.imshow("Distance View", distance_view_with_box)

        # Display the color-only view
        cv2.imshow("Color Only View", color_only_view)

        # Exit the program when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

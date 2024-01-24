import cv2 as cv
import os

def load_torus_images(folder_path):
    torus_images = [cv.imread(os.path.join(folder_path, img)) for img in os.listdir(folder_path)]
    return torus_images

# Function for orange color detection with added smoothing
def detect_orange_torus(frame, lower_orange, upper_orange):
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_frame, lower_orange, upper_orange)

    # Apply adaptive thresholding to the grayscale image
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _, thresholded = cv.threshold(gray_frame, 200, 255, cv.THRESH_BINARY_INV)

    # Combine color-based mask and adaptive thresholding mask
    combined_mask = cv.bitwise_and(mask, thresholded)

    # Apply Gaussian blur to reduce noise
    blurred_mask = cv.GaussianBlur(combined_mask, (5, 5), 0)

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

# Main function for live camera feed
def main():
    # Set the path to the "Assets" folder (Made Universal)
    folder_path = "./src/Assets"

    # Example range for orange color in HSV
    lower_orange = (5, 170, 200)
    upper_orange = (15, 255, 255)

    # Load torus images
    torus_images = load_torus_images(folder_path)

    # Create windows for displaying the live camera feed, color-only view, bounding box view, and distance view
    cv.namedWindow("Color Only View", cv.WINDOW_NORMAL)
    cv.namedWindow("Box View", cv.WINDOW_NORMAL)
    cv.namedWindow("Distance View", cv.WINDOW_NORMAL)

    # Open the camera (change 0 to the appropriate camera index if needed [1 or -1 for external cameras])
    cap = cv.VideoCapture(0)

    # Known physical width of the torus in inches (example)
    known_width_inches = 10 # 10 for Note

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
            distance_text = f"Distance: {distance *2.54:.2f} inches"

            # Display the distance and bounding box in the "Distance View" window
            distance_view = frame.copy()
            cv.putText(distance_view, distance_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            distance_view_with_box = draw_bounding_box(distance_view, filtered_contours, (0, 0, 255))
            cv.imshow("Distance View", distance_view_with_box)

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

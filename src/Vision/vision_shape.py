import cv2
import os
import numpy as np

def load_torus_images(folder_path):
    torus_images = [cv2.imread(os.path.join(folder_path, img), cv2.IMREAD_GRAYSCALE) for img in os.listdir(folder_path)]
    return torus_images

# Function to draw a bounding box around the detected target
def draw_bounding_box(frame, contours, color):
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
    return frame

# Function to match the detected contour with torus templates based on shape and color
def match_torus_shape_and_color(contour, torus_images, frame, lower_orange, upper_orange):
    # Create a binary mask from the contour
    mask = np.zeros_like(frame)
    cv2.drawContours(mask, [contour], 0, (255, 255, 255), thickness=cv2.FILLED)
    
    # Extract the region of interest (ROI) from the frame using the binary mask
    roi = cv2.bitwise_and(frame, mask)

    # Convert the ROI to grayscale
    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    for template in torus_images:
        _, threshold_template = cv2.threshold(template, 128, 255, cv2.THRESH_BINARY)

        # Find contours in the binary images
        contours_template, _ = cv2.findContours(threshold_template, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Compare the shapes of the first contours
        if len(contours_template) > 0:
            result = cv2.matchShapes(contour, contours_template[0], cv2.CONTOURS_MATCH_I1, 0.0)

            # You can set a threshold for matching (lower values indicate a better match)
            if result < 0.1:
                # Check color similarity
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_roi, lower_orange, upper_orange)
                color_similarity = cv2.countNonZero(mask) / cv2.countNonZero(threshold_template)

                # You can set a threshold for color matching
                if color_similarity > 0.7:
                    return True  # Match found (both shape and color)

    return False  # No match found



# Main function for live camera feed
def main():
    # Set the path to the "Assets" folder (Made Universal)
    folder_path = "./src/Assets"

    # Load torus images
    torus_images = load_torus_images(folder_path)

    # Example range for orange color in HSV
    lower_orange = (0, 150, 100)
    upper_orange = (10, 255, 255)

    # Create windows for displaying the live camera feed, bounding box view, and combined view
    cv2.namedWindow("Box View", cv2.WINDOW_NORMAL)

    # Open the camera (change 0 to the appropriate camera index if needed [1 or -1 for external cameras])
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Find contours of the detected torus
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, threshold_frame = cv2.threshold(gray_frame, 128, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(threshold_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Match the detected contour with torus templates based on shape and color
        for contour in contours:
            if match_torus_shape_and_color(contour, torus_images, frame, lower_orange, upper_orange):
                # Draw a bounding box around the matched torus in the "Box View" window
                box_view = draw_bounding_box(frame.copy(), [contour], (0, 255, 0))
                cv2.imshow("Box View", box_view)

        # Exit the program when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import cv2
import os

# Function to load torus images from the "Assets" folder
def load_torus_images(folder_path):
    torus_images = [cv2.imread(os.path.join(folder_path, img)) for img in os.listdir(folder_path)]
    return torus_images

# Function for orange color detection
def detect_orange_torus(frame, lower_orange, upper_orange):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    return result

# Main function for live camera feed
def main():
    # Set the path to the "Assets" folder (Made Universal)
    folder_path = "../Assets"

    # Example range for orange color in HSV
    lower_orange = (0, 100, 100)
    upper_orange = (20, 255, 255)

    # Load torus images
    torus_images = load_torus_images(folder_path)

    # Create a window for displaying the live camera feed
    cv2.namedWindow("Live Orange Torus Detection", cv2.WINDOW_NORMAL)

    # Open the camera (change 0 to the appropriate camera index if needed [1 or -1 for external cameras])
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Detect orange torus in the frame
        result = detect_orange_torus(frame, lower_orange, upper_orange)

        # Display the result
        cv2.imshow("Live Orange Torus Detection", result)

        # Exit the program when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import cv2 as cv
import os
# your welcome videsh

# Function to load torus images
def load_torus_images(folder_path):
    torus_images = [cv.imread(os.path.join(folder_path, img)) for img in os.listdir(folder_path)]
    return torus_images

# Function to detect orange torus
def detect_orange_torus(frame, lower_orange_rgb, upper_orange_rgb):
    lower_orange = lower_orange_rgb[::-1]
    upper_orange = upper_orange_rgb[::-1]

    blurred_frame = cv.GaussianBlur(frame, (5, 5), 0)
    mask = cv.inRange(blurred_frame, lower_orange, upper_orange)

    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    smoothed_mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    smoothed_mask = cv.morphologyEx(smoothed_mask, cv.MORPH_CLOSE, kernel)

    result = cv.bitwise_and(frame, frame, mask=smoothed_mask)
    return result, smoothed_mask

# Function to draw bounding box around contours
def draw_outermost_bounding_box(frame, contours, color):
    if len(contours) > 0:
        # Find the outermost bounding box that encloses all contours
        x, y, w, h = cv.boundingRect(cv.convexHull(contours[0]))
        for contour in contours[1:]:
            temp_x, temp_y, temp_w, temp_h = cv.boundingRect(cv.convexHull(contour))
            x = min(x, temp_x)
            y = min(y, temp_y)
            w = max(x + w, temp_x + temp_w) - x
            h = max(y + h, temp_y + temp_h) - y

        cv.rectangle(frame, (x, y), (x+w, y+h), color, 2)
    return frame

# Function to estimate distance
def estimate_distance(apparent_width, known_width, focal_length):
    distance = (known_width * focal_length) / apparent_width
    return distance

# Function to manually adjust exposure based on pixel intensity
def adjust_exposure(frame, target_intensity):
    current_intensity = cv.mean(frame)[0]  # Calculate the average pixel intensity

    # Adjust exposure based on the difference between current and target intensities
    exposure_adjustment = 1.0 + (target_intensity - current_intensity) / 255.0
    return cv.convertScaleAbs(frame, alpha=exposure_adjustment)

# Main function
def main():
    folder_path = "./src/Assets"

    upper_orange_rgb = (255, 150, 100)
    lower_orange_rgb = (200, 50, 0)

    torus_images = load_torus_images(folder_path)

    cv.namedWindow("Color Only View", cv.WINDOW_NORMAL)
    cv.namedWindow("Box View", cv.WINDOW_NORMAL)
    cv.namedWindow("Distance View", cv.WINDOW_NORMAL)

    cap = cv.VideoCapture(0)

    known_width_inches = 10.0
    focal_length = 320.8

    target_intensity = 120  # Adjust this value based on your preference

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Manually adjust exposure based on pixel intensity
        frame = adjust_exposure(frame, target_intensity)

        color_only_view, mask = detect_orange_torus(frame.copy(), lower_orange_rgb, upper_orange_rgb)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        filtered_contours = [contour for contour in contours if cv.contourArea(contour) > 100]

        box_view = draw_outermost_bounding_box(frame.copy(), filtered_contours, (0, 255, 0))
        cv.imshow("Box View", box_view)

        if len(filtered_contours) > 0:
            x, y, w, h = cv.boundingRect(filtered_contours[0])
            apparent_width = w

            distance = estimate_distance(apparent_width, known_width_inches, focal_length)
            distance_text = f"Distance: {distance:.2f} inches"

            combined_view = frame.copy()
            cv.putText(combined_view, distance_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            draw_outermost_bounding_box(combined_view, filtered_contours, (0, 0, 255))
            cv.imshow("Distance View", combined_view)

        cv.imshow("Color Only View", color_only_view)

        # Exit the program when the 'q' key is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()

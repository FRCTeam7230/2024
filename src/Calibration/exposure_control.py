import cv2 as cv

def set_exposure(cap, exposure_value):
    # Check if the camera supports setting exposure
    if cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.0):  # Disable auto exposure
        cap.set(cv.CAP_PROP_EXPOSURE, exposure_value)
        print(f"Exposure set to {exposure_value}")
    else:
        print("Exposure setting not supported by the camera")

def main():
    cap = cv.VideoCapture(0)

    # Set the desired exposure value (experiment with different values)
    exposure_value = -6  # Adjust this value based on your needs

    # Set exposure if supported
    set_exposure(cap, exposure_value)

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        cv.imshow("Camera Feed", frame)

        # Exit the program when the 'q' key is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()

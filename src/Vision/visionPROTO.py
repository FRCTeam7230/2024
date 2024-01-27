import cv2
import numpy as np
import threading
from tkinter import Tk, Label, Scale, HORIZONTAL, RIGHT, LEFT, BOTH
from PIL import Image, ImageTk

class ThresholdInRange:
    def __init__(self, camera_device=0):
        self.MAX_VALUE_H = 180
        self.MAX_VALUE = 255
        self.WINDOW_NAME = "Slider Color Detection"
        self.LOW_H_NAME = "Low H"
        self.LOW_S_NAME = "Low S"
        self.LOW_V_NAME = "Low V"
        self.HIGH_H_NAME = "High H"
        self.HIGH_S_NAME = "High S"
        self.HIGH_V_NAME = "High V"

        self.cap = cv2.VideoCapture(camera_device)
        if not self.cap.isOpened():
            print(f"Cannot open camera: {camera_device}")
            return

        _, self.mat_frame = self.cap.read()

        self.root = Tk()
        self.init_ui()

        self.capture_thread = threading.Thread(target=self.capture_task)
        self.capture_thread.start()

        self.root.mainloop()

    def init_ui(self):
        self.root.title(self.WINDOW_NAME)

        # Sliders
        self.slider_low_h = Scale(self.root, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.LOW_H_NAME)
        self.slider_high_h = Scale(self.root, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.HIGH_H_NAME)
        self.slider_low_s = Scale(self.root, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_S_NAME)
        self.slider_high_s = Scale(self.root, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_S_NAME)
        self.slider_low_v = Scale(self.root, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_V_NAME)
        self.slider_high_v = Scale(self.root, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_V_NAME)

        self.slider_low_h.grid(row=0, column=0, padx=5, pady=5)
        self.slider_high_h.grid(row=1, column=0, padx=5, pady=5)
        self.slider_low_s.grid(row=2, column=0, padx=5, pady=5)
        self.slider_high_s.grid(row=3, column=0, padx=5, pady=5)
        self.slider_low_v.grid(row=4, column=0, padx=5, pady=5)
        self.slider_high_v.grid(row=5, column=0, padx=5, pady=5)

        # Labels for all views
        self.img_capture_label = Label(self.root)
        self.img_detection_label = Label(self.root)
        self.img_combined_label = Label(self.root)
        self.img_raw_label = Label(self.root)
        self.img_box_label = Label(self.root)
        self.img_distance_label = Label(self.root)
        self.slider_values_label = Label(self.root)

        self.img_capture_label.grid(row=0, column=1, padx=5, pady=5)
        self.img_detection_label.grid(row=0, column=2, padx=5, pady=5)
        self.img_combined_label.grid(row=1, column=1, padx=5, pady=5)
        self.img_raw_label.grid(row=1, column=2, padx=5, pady=5)
        self.img_box_label.grid(row=0, column=3, padx=5, pady=5)
        self.img_distance_label.grid(row=1, column=3, padx=5, pady=5)
        self.slider_values_label.grid(row=0, column=4, rowspan=2, padx=5, pady=5)

    def capture_task(self):
        known_width_inches = 10  # known physical width of the torus in inches (example)
        focal_length = 320.8  # known focal length of the camera (example, you need to calibrate this based on your camera)

        while True:
            ret, self.mat_frame = self.cap.read()
            if not ret:
                break

            frame_hsv = cv2.cvtColor(self.mat_frame, cv2.COLOR_BGR2HSV)

            # Apply the sliders values to filter the color
            thresh = cv2.inRange(frame_hsv, (self.slider_low_h.get(), self.slider_low_s.get(), self.slider_low_v.get()),
                                 (self.slider_high_h.get(), self.slider_high_s.get(), self.slider_high_v.get()))

            # Combine original frame with the thresholded image
            img_combined = cv2.bitwise_and(self.mat_frame, self.mat_frame, mask=thresh)

            # Box detection
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > 100]

            box_view = self.draw_bounding_box(self.mat_frame.copy(), filtered_contours, (0, 255, 0))

            # Distance estimation
            distance_text = ""
            if len(filtered_contours) > 0:
                apparent_width = self.measure_longest_side(filtered_contours[0])
                distance = self.estimate_distance(apparent_width, known_width_inches, focal_length)
                distance_text = f"Distance: {distance:.2f} inches"

            combined_view = self.mat_frame.copy()
            cv2.putText(combined_view, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            box_view_resized = cv2.resize(box_view, (320, 240))

            frame_resized = cv2.resize(self.mat_frame, (320, 240))
            thresh_resized = cv2.resize(thresh, (320, 240))
            img_combined_resized = cv2.resize(img_combined, (320, 240))
            combined_view_resized = cv2.resize(combined_view, (320, 240))

            self.update_display(frame_resized, thresh_resized, img_combined_resized,
                                box_view_resized, combined_view_resized)

    def update_display(self, img_capture, img_thresh, img_combined, img_box, img_distance):
        img_capture = cv2.cvtColor(img_capture, cv2.COLOR_BGR2RGB)
        img_thresh = cv2.cvtColor(img_thresh, cv2.COLOR_GRAY2RGB)
        img_combined = cv2.cvtColor(img_combined, cv2.COLOR_BGR2RGB)
        img_box = cv2.cvtColor(img_box, cv2.COLOR_BGR2RGB)
        img_distance = cv2.cvtColor(img_distance, cv2.COLOR_BGR2RGB)

        img_capture = Image.fromarray(img_capture)
        img_thresh = Image.fromarray(img_thresh)
        img_combined = Image.fromarray(img_combined)
        img_box = Image.fromarray(img_box)
        img_distance = Image.fromarray(img_distance)

        img_capture = ImageTk.PhotoImage(img_capture)
        img_thresh = ImageTk.PhotoImage(img_thresh)
        img_combined = ImageTk.PhotoImage(img_combined)
        img_box = ImageTk.PhotoImage(img_box)
        img_distance = ImageTk.PhotoImage(img_distance)

        self.img_capture_label.configure(image=img_capture)
        self.img_capture_label.image = img_capture

        self.img_detection_label.configure(image=img_thresh)
        self.img_detection_label.image = img_thresh

        self.img_combined_label.configure(image=img_combined)
        self.img_combined_label.image = img_combined

        self.img_box_label.configure(image=img_box)
        self.img_box_label.image = img_box

        self.img_distance_label.configure(image=img_distance)
        self.img_distance_label.image = img_distance

        # Update the slider values label
        slider_values_text = f"Low H: {self.slider_low_h.get()}, High H: {self.slider_high_h.get()}\n" \
                             f"Low S: {self.slider_low_s.get()}, High S: {self.slider_high_s.get()}\n" \
                             f"Low V: {self.slider_low_v.get()}, High V: {self.slider_high_v.get()}"
        self.slider_values_label.configure(text=slider_values_text)

    @staticmethod
    def draw_bounding_box(frame, contours, color):
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        return frame

    @staticmethod
    def estimate_distance(apparent_width, known_width, focal_length):
        distance = (known_width * focal_length) / apparent_width
        return distance

    @staticmethod
    def measure_longest_side(contour):
        _, _, w, h = cv2.boundingRect(contour)
        return max(w, h)


if __name__ == "__main__":
    # Load the native OpenCV library
    cv2.ocl.setUseOpenCL(False)
    # Creating and showing the application's GUI
    threshold_in_range = ThresholdInRange()

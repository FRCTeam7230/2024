import cv2
import numpy as np
import threading
from tkinter import Tk, Label, Scale, HORIZONTAL, RIGHT, LEFT, BOTH, Frame, Button
from PIL import Image, ImageTk
import configparser as config


def draw_bounding_box(frame, contours, color):
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
    return frame

def estimate_distance(apparent_width, known_width, focal_length):
    distance = (known_width * focal_length) / apparent_width
    return distance

class ThresholdInRange:
    def __init__(self, camera_device=1):
        self.MAX_VALUE_H = 180
        self.MAX_VALUE = 255
        self.WINDOW_NAME = "Slider Color Detection"
        self.LOW_H_NAME = "Low Hue"
        self.LOW_S_NAME = "Low Saturation"
        self.LOW_V_NAME = "Low Value"
        self.HIGH_H_NAME = "High Hue"
        self.HIGH_S_NAME = "High Saturation"
        self.HIGH_V_NAME = "High Value"

        self.cap = cv2.VideoCapture(camera_device)
        if not self.cap.isOpened():
            print(f"Cannot open camera: {camera_device}")
            return

        _, self.mat_frame = self.cap.read()

        self.root = Tk()
        self.init_ui()

        self.capture_thread = threading.Thread(target=self.capture_task)
        self.capture_thread.start()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()


    def on_close(self):
        # Release the camera
        if self.cap.isOpened():
            self.cap.release()

        # Close the Tkinter window
        self.root.destroy()

    def init_ui(self):
        self.root.title(self.WINDOW_NAME)

        # Sliders on the left
        slider_frame = Frame(self.root)
        slider_frame.pack(side=LEFT, padx=10, pady=10)

        # Low Hue slider
        self.slider_low_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.LOW_H_NAME, command=self.update_settings)
        self.slider_low_h.pack()

        # High Hue slider
        self.slider_high_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.HIGH_H_NAME, command=self.update_settings)
        self.slider_high_h.pack()

        # Other sliders
        self.slider_low_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_S_NAME, command=self.update_settings)
        self.slider_high_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_S_NAME, command=self.update_settings)
        self.slider_low_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_V_NAME, command=self.update_settings)
        self.slider_high_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_V_NAME, command=self.update_settings)

        self.slider_low_s.pack()
        self.slider_high_s.pack()
        self.slider_low_v.pack()
        self.slider_high_v.pack()

        # Views on the right in a 2x2 grid
        views_frame = Frame(self.root)
        views_frame.pack(side=RIGHT, expand=True, fill=BOTH, padx=10, pady=10)

        self.img_raw_label = Label(views_frame)
        self.img_raw_label.grid(row=0, column=0, padx=5, pady=5)

        self.img_color_label = Label(views_frame)
        self.img_color_label.grid(row=0, column=1, padx=5, pady=5)

        self.img_bw_label = Label(views_frame)
        self.img_bw_label.grid(row=1, column=0, padx=5, pady=5)

        self.img_box_label = Label(views_frame)
        self.img_box_label.grid(row=1, column=1, padx=5, pady=5)

    def update_settings(self, event=None):
        # Called when sliders are adjusted
        self.root.update()

    def capture_task(self):
        # Calibration parameters (change these based on your setup)
        known_width_inches = 10.0  # Example: the actual width of the torus in inches
        focal_length = 320.8  # Example: you need to calibrate this based on your camera

        while True:
            ret, self.mat_frame = self.cap.read()
            if not ret:
                break

            frame_hsv = cv2.cvtColor(self.mat_frame, cv2.COLOR_BGR2HSV)

            # Get slider values
            low_h = self.slider_low_h.get()
            high_h = self.slider_high_h.get()
            low_s = self.slider_low_s.get()
            high_s = self.slider_high_s.get()
            low_v = self.slider_low_v.get()
            high_v = self.slider_high_v.get()

            # Color Only View
            color_only_view = cv2.cvtColor(frame_hsv, cv2.COLOR_HSV2RGB)
            color_only_view = cv2.resize(color_only_view, (400, 300))
            color_only_view = Image.fromarray(color_only_view)
            color_only_view = ImageTk.PhotoImage(color_only_view)

            self.img_color_label.configure(image=color_only_view)
            self.img_color_label.image = color_only_view

            # Raw Footage View
            raw_view = cv2.cvtColor(self.mat_frame, cv2.COLOR_BGR2RGB)
            raw_view = cv2.resize(raw_view, (400, 300))
            raw_view = Image.fromarray(raw_view)
            raw_view = ImageTk.PhotoImage(raw_view)

            self.img_raw_label.configure(image=raw_view)
            self.img_raw_label.image = raw_view

            # Black and White View
            bw_frame = cv2.inRange(frame_hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))
            bw_frame = cv2.cvtColor(bw_frame, cv2.COLOR_GRAY2RGB)
            bw_frame = cv2.resize(bw_frame, (400, 300))
            bw_frame = Image.fromarray(bw_frame)
            bw_frame = ImageTk.PhotoImage(bw_frame)

            self.img_bw_label.configure(image=bw_frame)
            self.img_bw_label.image = bw_frame

            # Box View
            thresh = cv2.inRange(frame_hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            frame_with_box = np.copy(self.mat_frame)
            frame_with_box = draw_bounding_box(frame_with_box, contours, (0, 255, 0))

            box_view = cv2.cvtColor(frame_with_box, cv2.COLOR_BGR2RGB)
            box_view = cv2.resize(box_view, (400, 300))
            box_view = Image.fromarray(box_view)
            box_view = ImageTk.PhotoImage(box_view)

            self.img_box_label.configure(image=box_view)
            self.img_box_label.image = box_view

            self.root.update()

        self.cap.release()

if __name__ == "__main__":
    threshold_in_range = ThresholdInRange()

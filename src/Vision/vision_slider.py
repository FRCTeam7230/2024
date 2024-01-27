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
        self.LOW_H_NAME = "Low Hue (H)"
        self.LOW_S_NAME = "Low Saturation (S)"
        self.LOW_V_NAME = "Low Value (V)"
        self.HIGH_H_NAME = "High Hue (H)"
        self.HIGH_S_NAME = "High Saturation (S)"
        self.HIGH_V_NAME = "High Value (V)"

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

        slider_frame = Label(self.root)
        slider_frame.pack(side=LEFT, fill=BOTH, expand=True)

        self.slider_low_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.LOW_H_NAME)
        self.slider_low_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_S_NAME)
        self.slider_low_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_V_NAME)
        self.slider_high_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.HIGH_H_NAME)
        self.slider_high_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_S_NAME)
        self.slider_high_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_V_NAME)

        self.slider_low_h.pack()
        self.slider_high_h.pack()
        self.slider_low_s.pack()
        self.slider_high_s.pack()
        self.slider_low_v.pack()
        self.slider_high_v.pack()

        self.img_capture_label = Label(self.root)
        self.img_detection_label = Label(self.root)

        self.img_capture_label.pack(side=RIGHT)
        self.img_detection_label.pack(side=RIGHT)

    def capture_task(self):
        while True:
            ret, self.mat_frame = self.cap.read()
            if not ret:
                break

            frame_hsv = cv2.cvtColor(self.mat_frame, cv2.COLOR_BGR2HSV)

            thresh = cv2.inRange(frame_hsv, (self.slider_low_h.get(), self.slider_low_s.get(), self.slider_low_v.get()),
                                 (self.slider_high_h.get(), self.slider_high_s.get(), self.slider_high_v.get()))

            frame_resized = cv2.resize(self.mat_frame, (640, 480))
            thresh_resized = cv2.resize(thresh, (640, 480))

            self.update_display(frame_resized, thresh_resized)

    def update_display(self, img_capture, img_thresh):
        img_capture = cv2.cvtColor(img_capture, cv2.COLOR_BGR2RGB)

        img_capture = Image.fromarray(img_capture)
        img_thresh = Image.fromarray(img_thresh)

        img_capture = ImageTk.PhotoImage(img_capture)
        img_thresh = ImageTk.PhotoImage(img_thresh)

        self.img_capture_label.configure(image=img_capture)
        self.img_capture_label.image = img_capture

        self.img_detection_label.configure(image=img_thresh)
        self.img_detection_label.image = img_thresh

if __name__ == "__main__":
    cv2.ocl.setUseOpenCL(False)
    threshold_in_range = ThresholdInRange()

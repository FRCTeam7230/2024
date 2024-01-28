import cv2
import numpy as np
import threading
from tkinter import *
from PIL import Image, ImageTk
import json
import os
import platform
import subprocess
import sys

path = os.path.dirname(os.path.abspath(__file__))
os_sys = platform.system()

if os_sys == "Windows":
    config_f = open(path + "//" + 'config.json')
else:
    config_f = open('src/Vision/config.json')
config = json.load(config_f)

class ThresholdInRange:
    def __init__(self, camera_device=0):
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
        if self.cap.isOpened():
            self.cap.release()
        self.root.destroy()

    def init_ui(self):
        self.root.title(self.WINDOW_NAME)

        slider_frame = Frame(self.root)
        slider_frame.pack(side=LEFT, padx=10, pady=10)

        self.slider_low_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.LOW_H_NAME, command=self.update_settings)
        self.slider_low_h.pack()

        self.slider_high_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.HIGH_H_NAME, command=self.update_settings)
        self.slider_high_h.pack()

        self.slider_low_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_S_NAME, command=self.update_settings)
        self.slider_high_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_S_NAME, command=self.update_settings)
        self.slider_low_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_V_NAME, command=self.update_settings)
        self.slider_high_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_V_NAME, command=self.update_settings)

        self.slider_low_s.pack()
        self.slider_high_s.pack()
        self.slider_low_v.pack()
        self.slider_high_v.pack()

        self.low_high_labels = Label(slider_frame, text="Low/High Values: RGB, BGR, HSV", font=("Helvetica", 10, "bold"))
        self.low_high_labels.pack(pady=(10, 0))

        self.rgb_low_label = Label(slider_frame, text="RGB Low: (0, 0, 0)")
        self.rgb_high_label = Label(slider_frame, text="RGB High: (255, 255, 255)")
        self.bgr_low_label = Label(slider_frame, text="BGR Low: (0, 0, 0)")
        self.bgr_high_label = Label(slider_frame, text="BGR High: (255, 255, 255)")
        self.hsv_low_label = Label(slider_frame, text="HSV Low: (0, 0, 0)")
        self.hsv_high_label = Label(slider_frame, text="HSV High: (180, 255, 255)")

        self.rgb_low_label.pack()
        self.rgb_high_label.pack()
        self.bgr_low_label.pack()
        self.bgr_high_label.pack()
        self.hsv_low_label.pack()
        self.hsv_high_label.pack()

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

        self.current_preset_var = StringVar()
        self.current_preset_var.set("debug_block")

        self.apply_preset_button = Button(self.root, text="Apply Preset", command=self.apply_preset)
        self.apply_preset_button.pack(side=LEFT, padx=10, pady=10)

        for preset in config["color_presets"]:
            preset_name = preset["name"]
            button = Button(self.root, text=preset_name, command=lambda name=preset_name: self.select_preset(name))
            button.pack(side=LEFT, padx=10, pady=10)

    def select_preset(self, preset_name):
        self.current_preset_var.set(preset_name)
        self.apply_preset()

    def update_settings(self, event=None):
        low_h = self.slider_low_h.get()
        high_h = self.slider_high_h.get()
        low_s = self.slider_low_s.get()
        high_s = self.slider_high_s.get()
        low_v = self.slider_low_v.get()
        high_v = self.slider_high_v.get()

        self.rgb_low_label.config(text=f"RGB Low: ({low_h}, {low_s}, {low_v})")
        self.rgb_high_label.config(text=f"RGB High: ({high_h}, {high_s}, {high_v})")

        self.bgr_low_label.config(text=f"BGR Low: ({low_v}, {low_s}, {low_h})")
        self.bgr_high_label.config(text=f"BGR High: ({high_v}, {high_s}, {high_h})")

        self.hsv_low_label.config(text=f"HSV Low: ({low_h}, {low_s}, {low_v})")
        self.hsv_high_label.config(text=f"HSV High: ({high_h}, {high_s}, {high_v})")

        self.root.update()

    def apply_preset(self):
        current_preset = self.current_preset_var.get()
        preset_values = next((preset for preset in config["color_presets"] if preset["name"] == current_preset), {})

        self.slider_low_h.set(int(preset_values.get("low_hue", 0) or 0))
        self.slider_high_h.set(int(preset_values.get("high_hue", 0) or 0))
        self.slider_low_s.set(int(preset_values.get("low_saturation", 0) or 0))
        self.slider_high_s.set(int(preset_values.get("high_saturation", 0) or 0))
        self.slider_low_v.set(int(preset_values.get("low_value", 0) or 0))
        self.slider_high_v.set(int(preset_values.get("high_value", 0) or 0))

    def draw_bounding_box(self, frame, contours, color):
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        if contours:
            largest_contour = contours[0]
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        return frame

    def capture_task(self):
        while True:
            ret, self.mat_frame = self.cap.read()
            if not ret:
                break
            frame_hsv = cv2.cvtColor(self.mat_frame, cv2.COLOR_BGR2HSV)

            low_h = self.slider_low_h.get()
            high_h = self.slider_high_h.get()
            low_s = self.slider_low_s.get()
            high_s = self.slider_high_s.get()
            low_v = self.slider_low_v.get()
            high_v = self.slider_high_v.get()

            mask = cv2.inRange(frame_hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))
            color_only_view = cv2.bitwise_and(self.mat_frame, self.mat_frame, mask=mask)
            color_only_view = cv2.cvtColor(color_only_view, cv2.COLOR_BGR2RGB)
            color_only_view = cv2.resize(color_only_view, (400, 300))
            color_only_view = Image.fromarray(color_only_view)
            color_only_view = ImageTk.PhotoImage(color_only_view)

            self.img_color_label.configure(image=color_only_view)
            self.img_color_label.image = color_only_view

            raw_view = cv2.cvtColor(self.mat_frame, cv2.COLOR_BGR2RGB)
            raw_view = cv2.resize(raw_view, (400, 300))
            raw_view = Image.fromarray(raw_view)
            raw_view = ImageTk.PhotoImage(raw_view)

            self.img_raw_label.configure(image=raw_view)
            self.img_raw_label.image = raw_view

            bw_frame = cv2.inRange(frame_hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))
            bw_frame = cv2.cvtColor(bw_frame, cv2.COLOR_GRAY2RGB)
            bw_frame = cv2.resize(bw_frame, (400, 300))
            bw_frame = Image.fromarray(bw_frame)
            bw_frame = ImageTk.PhotoImage(bw_frame)

            self.img_bw_label.configure(image=bw_frame)
            self.img_bw_label.image = bw_frame

            thresh = cv2.inRange(frame_hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            frame_with_box = np.copy(self.mat_frame)
            frame_with_box = self.draw_bounding_box(frame_with_box, contours, (0, 255, 0))

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

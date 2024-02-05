import cv2
import numpy as np
import threading
from tkinter import *
from PIL import Image, ImageTk
import json
import os
import platform
import math

# Load configuration from JSON file
path = os.path.dirname(os.path.abspath(__file__))
os_sys = platform.system()

if os_sys == "Windows":
    config_f = open(path + "//" + 'config.json')
else:
    config_f = open('src/Vision/config.json')
config = json.load(config_f)

# Class to handle thresholding and object detection
class ThresholdInRange:
    def __init__(self, camera_device=0):
        # Constants for color thresholding
        self.MAX_VALUE_H = 180
        self.MAX_VALUE = 255
        self.WINDOW_NAME = "Slider Color Detection"
        self.LOW_H_NAME = "Low Hue"
        self.LOW_S_NAME = "Low Saturation"
        self.LOW_V_NAME = "Low Value"
        self.HIGH_H_NAME = "High Hue"
        self.HIGH_S_NAME = "High Saturation"
        self.HIGH_V_NAME = "High Value"

        # Open the video capture device
        self.cap = cv2.VideoCapture(camera_device)
        if not self.cap.isOpened():
            print(f"Cannot open camera: {camera_device}")
            return

        # Read an initial frame
        _, self.mat_frame = self.cap.read()

        # Create a Tkinter window
        self.root = Tk()
        self.init_ui()

        # Start a thread to capture frames continuously
        self.capture_thread = threading.Thread(target=self.capture_task)
        self.capture_thread.start()
        
        # Set up closing behavior for the window
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    # Method to handle window closing
    def on_close(self):
        if self.cap.isOpened():
            self.cap.release()
        self.root.destroy()
        self.root.quit()

    # Method to initialize the Tkinter user interface
    def init_ui(self):
        self.root.title(self.WINDOW_NAME)

        # Create sliders for adjusting color threshold values
        slider_frame = Frame(self.root)
        slider_frame.pack(side=LEFT, padx=10, pady=10)

        self.slider_low_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.LOW_H_NAME, command=self.update_settings)
        self.slider_high_h = Scale(slider_frame, from_=0, to=self.MAX_VALUE_H, orient=HORIZONTAL, label=self.HIGH_H_NAME, command=self.update_settings)
        self.slider_low_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_S_NAME, command=self.update_settings)
        self.slider_high_s = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_S_NAME, command=self.update_settings)
        self.slider_low_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.LOW_V_NAME, command=self.update_settings)
        self.slider_high_v = Scale(slider_frame, from_=0, to=self.MAX_VALUE, orient=HORIZONTAL, label=self.HIGH_V_NAME, command=self.update_settings)

        self.slider_low_s.pack()
        self.slider_high_s.pack()
        self.slider_low_v.pack()
        self.slider_high_v.pack()
        self.slider_low_h.pack()
        self.slider_high_h.pack()

        # Labels to display current threshold values
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

        # Frame to display various views
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

        self.img_distance_label = Label(views_frame)
        self.img_distance_label.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

        # Create buttons to apply presets
        self.current_preset_var = StringVar()
        self.current_preset_var.set("debug_block")

        self.apply_preset_button = Button(self.root, text="Apply Preset", command=self.apply_preset)
        self.apply_preset_button.pack(side=LEFT, padx=10, pady=10)

        for preset in config["color_presets"]:
            preset_name = preset["name"]
            button = Button(self.root, text=preset_name, command=lambda name=preset_name: self.select_preset(name))
            button.pack(side=LEFT, padx=10, pady=10)

    # Method to select a color preset
    def select_preset(self, preset_name):
        self.current_preset_var.set(preset_name)
        self.apply_preset()

    # Method to update threshold values and labels
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
        self.bgr_high_label.config(text=f"BGR High: ({high_v}, {high_s}, {low_h})")

        self.hsv_low_label.config(text=f"HSV Low: ({low_h}, {low_s}, {low_v})")
        self.hsv_high_label.config(text=f"HSV High: ({high_h}, {high_s}, {high_v})")

        self.root.update()

    # Method to apply a color preset
    def apply_preset(self):
        current_preset = self.current_preset_var.get()
        preset_values = next((preset for preset in config["color_presets"] if preset["name"] == current_preset), {})

        self.slider_low_h.set(int(preset_values.get("low_hue", 0) or 0))
        self.slider_high_h.set(int(preset_values.get("high_hue", 0) or 0))
        self.slider_low_s.set(int(preset_values.get("low_saturation", 0) or 0))
        self.slider_high_s.set(int(preset_values.get("high_saturation", 0) or 0))
        self.slider_low_v.set(int(preset_values.get("low_value", 0) or 0))
        self.slider_high_v.set(int(preset_values.get("high_value", 0) or 0))

    # Method to draw a bounding box around the detected object
    def draw_bounding_box(self, frame, contours, color):
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        if contours:
            largest_contour = contours[0]
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

            center_x = self.mat_frame.shape[1] // 2
            center_y = self.mat_frame.shape[0] // 2

            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.line(frame, (center_x, center_y), (cx, cy), (0, 255, 0), 2)

                angle_rad = math.atan2(cy - center_y, cx - center_x)
                angle_deg = math.degrees(angle_rad)

                cv2.line(frame, (center_x, center_y), (center_x, cy), (0, 0, 255), 2)

                if center_y < cy:
                    angle_deg = -angle_deg
                    angle_text_color = (0, 0, 255)
                else:
                    angle_text_color = (0, 255, 0)

                angle_deg = abs(angle_deg)
                offset_text = f"Angular Offset: {angle_deg:.2f} degrees"
                cv2.putText(frame, offset_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, angle_text_color, 2, cv2.LINE_AA)

                cv2.line(frame, (center_x, center_y), (cx, center_y), (255, 0, 0), 2)

        return frame

    # Method to capture frames and update UI
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

            # Create a mask based on the color threshold values
            mask = cv2.inRange(frame_hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create a frame with bounding box around the detected object
            frame_with_box = np.copy(self.mat_frame)
            frame_with_box = self.draw_bounding_box(frame_with_box, contours, (0, 255, 0))

            # Calculate the apparent width of the detected object
            if contours:
                largest_contour = contours[0]
                apparent_width = cv2.arcLength(largest_contour, True)

                # Check if apparent width is not zero
                if apparent_width > 0:
                    # Known physical width of the object in inches (example)
                    known_width_inches = 10

                    # Known focal length of the camera (example, you need to calibrate this based on your camera)
                    focal_length = 320.8

                    # Estimate distance using triangulation
                    distance = (known_width_inches * focal_length) / apparent_width

                    # Display the combined view with red-colored bounding box and distance text
                    combined_view = np.copy(self.mat_frame)
                    distance_text = f"Distance: {distance:.2f} inches"
                    cv2.putText(combined_view, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                    # Draw bounding box around the detected object in the distance view
                    cv2.drawContours(combined_view, [largest_contour], -1, (0, 0, 255), 2)

                    # Resize the combined view to a common size
                    combined_view = cv2.resize(combined_view, (400, 300))

                    # Update the distance view in the Tkinter window
                    distance_view = Image.fromarray(combined_view)
                    distance_view = ImageTk.PhotoImage(distance_view)
                    self.img_distance_label.configure(image=distance_view)
                    self.img_distance_label.image = distance_view

            # Convert the original frame to RGB and resize for display
            raw_view = cv2.cvtColor(self.mat_frame, cv2.COLOR_BGR2RGB)
            raw_view = cv2.resize(raw_view, (400, 300))
            raw_view = Image.fromarray(raw_view)
            raw_view = ImageTk.PhotoImage(raw_view)

            # Extract and display the color-only view
            color_only_view = cv2.bitwise_and(self.mat_frame, self.mat_frame, mask=mask)
            color_only_view = cv2.cvtColor(color_only_view, cv2.COLOR_BGR2RGB)
            color_only_view = cv2.resize(color_only_view, (400, 300))
            color_only_view = Image.fromarray(color_only_view)
            color_only_view = ImageTk.PhotoImage(color_only_view)

            # Convert the thresholded frame to RGB and resize for display
            bw_frame = cv2.inRange(frame_hsv, (low_h, low_s, low_v), (high_h, high_s, high_v))
            bw_frame = cv2.cvtColor(bw_frame, cv2.COLOR_GRAY2RGB)
            bw_frame = cv2.resize(bw_frame, (400, 300))
            bw_frame = Image.fromarray(bw_frame)
            bw_frame = ImageTk.PhotoImage(bw_frame)

            # Convert the frame with bounding box to RGB and resize for display
            box_view = cv2.cvtColor(frame_with_box, cv2.COLOR_BGR2RGB)
            box_view = cv2.resize(box_view, (400, 300))
            box_view = Image.fromarray(box_view)
            box_view = ImageTk.PhotoImage(box_view)

            # Update the labels in the Tkinter window with the new images
            self.img_raw_label.configure(image=raw_view)
            self.img_raw_label.image = raw_view

            self.img_color_label.configure(image=color_only_view)
            self.img_color_label.image = color_only_view

            self.img_bw_label.configure(image=bw_frame)
            self.img_bw_label.image = bw_frame

            self.img_box_label.configure(image=box_view)
            self.img_box_label.image = box_view

            # Update the Tkinter window
            self.root.update()

        # Release the video capture device when the window is closed
        self.cap.release()

# Main execution block
if __name__ == "__main__":
    # Create an instance of the ThresholdInRange class
    threshold_in_range = ThresholdInRange()

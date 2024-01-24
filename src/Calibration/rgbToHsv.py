import cv2
import numpy as np

def rgb_to_hsv(rgb_color):
    # Normalize RGB values to the range [0, 1]
    r, g, b = [x / 255.0 for x in rgb_color]

    # Convert RGB to HSV
    hsv_color = cv2.cvtColor(np.uint8([[[
        int(r * 255),
        int(g * 255),
        int(b * 255)
    ]]]), cv2.COLOR_RGB2HSV)[0][0]

    return hsv_color

# Example usage
rgb_color = (255, 0, 0)  # Red color in RGB
hsv_color = rgb_to_hsv(rgb_color)
print(f"RGB: {rgb_color}, HSV: {hsv_color}")

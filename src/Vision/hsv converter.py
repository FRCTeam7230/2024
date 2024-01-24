import time
def convert_opencv_hsv_to_standard(opencv_hsv):
    # opencv_hsv is a tuple (H, S, V)
    h, s, v = opencv_hsv

    # Normalize Hue to [0, 360]
    h_standard = (h / 179) * 360

    # Normalize Saturation and Value to [0, 100]
    s_standard = (s / 255) * 100
    v_standard = (v / 255) * 100

    return h_standard, s_standard, v_standard

# Example usage:
opencv_hsv = (5, 170, 100)
standard_hsv = convert_opencv_hsv_to_standard(opencv_hsv)
print("Standard HSV:", standard_hsv)
opencv_hsv = (15, 255, 255)
standard_hsv = convert_opencv_hsv_to_standard(opencv_hsv)
print("Standard HSV:", standard_hsv)
time.sleep(10)


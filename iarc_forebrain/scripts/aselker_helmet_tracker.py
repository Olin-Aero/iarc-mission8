#!/usr/bin/env python3
import sys
import cv2
import numpy as np


# In normal usage, this function will be imported by the *_ros file, and called from there.
# For test purposes, this file is also executable and can directly process a saved image.
def detect_helmet_coordinates(image, visualize=True):
    """
    Detects the pixel coordinates of the helmet in the image feed

    Input: OpenCV image
    Output: tuple of x- and y- coordinates of the center of the green helmet in the display
    """

    h_range = 6
    s_range = 100
    v_range = 155

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_lower = np.array([180 - h_range/2, 255 - s_range, 100 - v_range])
    red_upper = np.array([h_range/2, 255, 255])

    red_lower_low_h = red_lower
    red_lower_low_h[0] = 0
    mask_low_h = cv2.inRange(hsv, red_lower_low_h, red_upper)

    red_upper_high_h = red_upper
    red_upper_high_h[0] = 179
    mask_high_h = cv2.inRange(hsv, red_lower, red_upper_high_h)

    mask = mask_low_h | mask_high_h
    masked = cv2.bitwise_and(image, image, mask=mask)

    if visualize:
        cv2.imshow('Raw Image', image)
        cv2.imshow("Masked image", masked)
    return (0, 0)  # (x, y) measured in pixels



### Test code for processing data directly from the computer webcam or a saved image file
if __name__ == '__main__':
    if len(sys.argv) == 2:
        filename = sys.argv[1]
        if filename == 'webcam':
            # Process the webcam in realtime
            camera = cv2.VideoCapture(0)  # "0" here means "the first webcam on your system"

            while True:
                # Get the latest image from the webcam
                ok, image = camera.read()
                if not ok:
                    print("Unable to open webcam...")
                    break

                # Process it
                coordinates = detect_helmet_coordinates(image)
                print(coordinates)

                # Exit if the "Esc" key is pressed
                key = cv2.waitKey(1)
                if key == 27:
                    break
        else:
            # Read data from an image file
            image = cv2.imread(filename)
            coordinates = detect_helmet_coordinates(image)
            print(coordinates)
            cv2.waitKey(0)

    else:
        print('Usage: When you run this script, provide either the filename of an image or the string "webcam"')

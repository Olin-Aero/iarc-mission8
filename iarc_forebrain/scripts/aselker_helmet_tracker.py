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

    # Define some threshholds
    h_range = 12
    s_range = 100
    v_range = 155

    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Define bounds for "red"
    red_lower = np.array([180 - h_range/2, 255 - s_range, 100 - v_range])
    red_upper = np.array([h_range/2, 255, 255])

    # Red loops over the 179-0 boundary in hue space, so we need two separate masks
    red_lower_low_h = red_lower.copy()
    red_lower_low_h[0] = 0
    mask_low_h = cv2.inRange(hsv, red_lower_low_h, red_upper)

    red_upper_high_h = red_upper.copy()
    red_upper_high_h[0] = 179
    mask_high_h = cv2.inRange(hsv, red_lower, red_upper_high_h)

    # Merge the masks
    mask = mask_low_h | mask_high_h

    # Erode and dilate to remove small blobs and islands
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)


    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[1] # This is [0] for opencv2, but we're on 3
    center = None

    if len(cnts) > 0: # If there are any contours
      c = max(cnts, key=cv2.contourArea) # Find the contour that surrounds the largest area
      m = cv2.moments(c)
      x = int(m["m10"] / m["m00"])
      y = int(m["m01"] / m["m00"])
      coords = (x, y)
    else:
      coords = None

    if visualize:
        crossed = image.copy()
        cv2.drawMarker(crossed, coords, (0, 255, 0), cv2.MARKER_CROSS)

        cv2.imshow('Full Image', crossed)
        masked = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("Masked image", masked)

    return coords



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

#!/usr/bin/env python2
import sys

import math
import cv2
import numpy as np

def nothing(x):
    pass

def color_picker(image, visualize=True):
    """
    Tool for choosing color thresholds
    """

    h1 = cv2.getTrackbarPos('H low','image')
    h2 = cv2.getTrackbarPos('H high','image')
    s1 = cv2.getTrackbarPos('S low','image')
    s2 = cv2.getTrackbarPos('S high','image')
    v1 = cv2.getTrackbarPos('V low','image')
    v2 = cv2.getTrackbarPos('V high','image')


    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply color filter
    if h2 >= h1:
        lower = np.array([h1, s1, v1])
        upper = np.array([h2, s2, v2])
        mask = cv2.inRange(hsv, lower, upper)
        frame = cv2.bitwise_and(image, image, mask=mask)
    else:
        lower1 = np.array([h1, s1, v1])
        upper1 = np.array([180, s2, v2])
        lower2 = np.array([0, s1, v1])
        upper2 = np.array([h2, s2, v2])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        frame1 = cv2.bitwise_and(image, image, mask=mask1)
        frame2 = cv2.bitwise_and(image, image, mask=mask2)
        frame = cv2.bitwise_or(frame1, frame2)
    
    # Display image
    if visualize:
        cv2.imshow('image', frame)
        # cv2.imshow('Processed image', display)

    return 0

def init_sliders(img):

    cv2.imshow('image', img)

    cv2.createTrackbar('H low','image',0,180,nothing)
    cv2.createTrackbar('H high','image',0,180,nothing)
    cv2.createTrackbar('S low','image',0,255,nothing)
    cv2.createTrackbar('S high','image',0,255,nothing)
    cv2.createTrackbar('V low','image',0,255,nothing)
    cv2.createTrackbar('V high','image',0,255,nothing)


### Test code for processing data directly from the computer webcam or a saved image file
if __name__ == '__main__':
    if len(sys.argv) == 2:
        filename = sys.argv[1]
        if filename == 'webcam':
            # Process the webcam in realtime
            camera = cv2.VideoCapture(0)  # "0" here means "the first webcam on your system"
            ok, image = camera.read()
            init_sliders(image)
            while True:
                # Get the latest image from the webcam
                ok, image = camera.read()
                if not ok:
                    print("Unable to open webcam...")
                    break

                # Process it
                coordinates = color_picker(image)
                print(coordinates)

                # Exit if the "Esc" key is pressed
                key = cv2.waitKey(1)
                if key == 27:
                    break
        else:
            # Read data from an image file
            image = cv2.imread(filename)
            init_sliders(image)
            count = color_picker(image)
            while True:
                # Process it
                count = color_picker(image)

                # Exit if the "Esc" key is pressed
                key = cv2.waitKey(1)
                if key == 27:
                    break
            print(count)
            cv2.waitKey(0)

    else:
        print('Usage: When you run this script, provide either the filename of an image or the string "webcam"')

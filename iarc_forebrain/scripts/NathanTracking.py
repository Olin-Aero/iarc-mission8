#!/usr/bin/env python2
import sys

import cv2
import numpy as np


# In normal usage, this function will be imported by the *_ros file, and called from there.
# For test purposes, this file is also executable and can directly process a saved image.
def detect_helmet_coordinates(image, visualize=True):
    """
    Detects the pixel coordinates of the blue marker cap

    Input: OpenCV image
    Output: tuple of x- and y- coordinates of the center of the green helmet in the display
    """
    hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Converts the image into hsv
    if visualize:
        cv2.imshow('HSV Image', hsvImage) # shows the HSV Image
    blueBound1 = (110,90,50) # the lower bound for blue
    blueBound2 = (120,255,255) # the upper bound for blue
    b1 = cv2.inRange(hsvImage, blueBound1, blueBound2) # makes a binary that is 1 if the pixel is blue and 0 else
    kernel = np.ones((5, 5), np.uint8) # takes out the noise
    b1 = cv2.morphologyEx(b1, cv2.MORPH_OPEN, kernel)
    if visualize:
        cv2.imshow('CapIllum',b1) # shows the binary image
    ret, thresh = cv2.threshold(b1, 127, 255, 0) # sets a threshold
    _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # gets the contours in the image
    # loop over the contours
    for c in contours:
        (x,y),radius = cv2.minEnclosingCircle(c) # get the minimum circle that can enclose the contour
        cX = int(x) # transforms to int
        cY = int(y)
        lineLength = int(radius)
        cv2.circle(image, (cX, cY), lineLength, (0, 0, 255), 1) # draws a red circle around the object
        cv2.line(image,(cX-lineLength,cY),(cX+lineLength,cY),(0,0,255)) # places 2 lines in it
        cv2.line(image,(cX,cY-lineLength),(cX,cY+lineLength),(0,0,255))
        if visualize:
            cv2.imshow("Image", image)
        return (cX, cY)  # (x, y) measured in pixels
    # show the image so it still displays if it doesn't see a thing.
    if visualize: 
        cv2.imshow("Image", image) # shows the original image



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

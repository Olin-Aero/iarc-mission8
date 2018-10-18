#!/usr/bin/env python2
import sys
import cv2

def callback(x):
    """
    Callback necessary for the createTrackbar call
    """
    pass

# Create the sliders and a window to hold them in
cv2.namedWindow('slider control window')
cv2.createTrackbar("h range", "slider control window", 0, 130, callback)
cv2.createTrackbar("s range", "slider control window", 30, 130, callback)
cv2.createTrackbar("v range", "slider control window", 0, 130, callback)


# In normal usage, this function will be imported by the *_ros file, and called from there.
# For test purposes, this file is also executable and can directly process a saved image.
def detect_helmet_coordinates(image, visualize=True):
    """
    Detects the pixel coordinates of the helmet in the image feed. Color selection is made
    by selecting HSV values through three gui sliders.

    Input: OpenCV image
    Output: tuple of x- and y- coordinates of the center of the helmet in the display.
            Returns None if no contours found.

    """
    # blur the image to make finding contours easier
    blurred = cv2.GaussianBlur(image, (11, 11), 0)

    # convert to hsv color values from BGR
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # receive values from track bars and apply them to the lower limit of the HSV range
    hmin = cv2.getTrackbarPos("h range", "slider control window")
    smin = cv2.getTrackbarPos("s range", "slider control window")
    vmin = cv2.getTrackbarPos("v range", "slider control window")
    lower = (hmin, smin, vmin)
    upper = (90, 255, 255)

    # create a mask filtered by the bounds
    mask = cv2.inRange(hsv, lower, upper)

    # filter to reduce noise
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    ret, thresh1 = cv2.threshold(mask, 127, 255, 0)

    # find contours and draw them
    im2, contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

    if visualize:
        cv2.imshow("Image", image)
        cv2.imshow('mask', mask)
        cv2.imshow('mask', thresh1)

    # loop through the contours and find the center of circles which enclose the contours
    for cont in contours:
        (x, y), radius = cv2.minEnclosingCircle(cont)
        contX = int(x)
        contY = int(y)

        return (contX, contY)  # (x, y) measured in pixels

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

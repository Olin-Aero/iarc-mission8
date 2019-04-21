#!/usr/bin/env python2
import sys

import math
import cv2
import numpy as np


def track_fingers(image, visualize=True):
    """
    Counts the number of raised fingers on a person's hand

    Input: OpenCV image
    Output: number of raised fingers
    """

    # Change image format from BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply color filter
    lower = np.array([0, 0, 0])
    upper = np.array([20, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    frame = cv2.bitwise_and(image, image, mask=mask)
    ret,frame_1 = cv2.threshold(frame,1,255,cv2.THRESH_BINARY)
    frame = cv2.cvtColor(frame_1, cv2.COLOR_BGR2GRAY)

    # Clean up small particles
    kernel = np.ones((3, 3), np.uint8)
    frame = cv2.erode(frame, kernel, iterations=1)
    frame, contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = filter(lambda x:len(x)>200, contours)

    # Compute convex hull
    if len(contours)==0: return
    contours = sorted(contours, key=cv2.contourArea)
    hand = contours[-1]
    convexIndices = cv2.convexHull(hand, returnPoints=False)
    fingertips = [contours[-1][i[0]] for i in convexIndices]
    fingertips, convexIndices = groupPoints(fingertips, convexIndices)

    # Compute defects
    defects = cv2.convexityDefects(contours[-1], convexIndices) # n x 1 x 4
    web = []
    # # TODO: filter defect points at the same time as non-defects
    for p1 in defects[:]:
        p = contours[-1][p1[0][2]][0]
        for p2 in fingertips:
            if dist(p, p2) < 10000:
                break
        web += [p]
    print(web)

    # Display image
    if visualize:
        display = cv2.drawContours(frame_1, contours[-1], -1, (0,255,0), 3)
        for point in fingertips:
            cv2.circle(display, (point[0], point[1]), 5, (0, 0, 255), thickness=-1)
        for point in web:
            print(point)
            point = contours[-1][point[2]]
            cv2.circle(display, (point[0][0], point[0][1]), 5, (255, 0, 0), thickness=-1)
        cv2.imshow('Raw Image', image)
        cv2.imshow('Processed image', display)

    return 0


def groupPoints(points, convexIndices):
    output = []
    weights = []
    indices = []
    for j, p1 in enumerate(points):
        for i, p2 in enumerate(output):
            if dist(p1[0], p2) < 20:
                p2[0] += p1[0][0]/weights[i]
                p2[1] += p1[0][1]/weights[i]
                weights[i] += 1
                p2[0] *= weights[i]/(weights[i]+1)
                break
        output += [p1[0]]
        weights += [1]
        indices += [convexIndices[j]]
    return output, np.array(indices)
            

def dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


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
                coordinates = track_fingers(image)
                print(coordinates)

                # Exit if the "Esc" key is pressed
                key = cv2.waitKey(1)
                if key == 27:
                    break
        else:
            # Read data from an image file
            image = cv2.imread(filename)
            count = track_fingers(image)
            print(count)
            cv2.waitKey(0)

    else:
        print('Usage: When you run this script, provide either the filename of an image or the string "webcam"')

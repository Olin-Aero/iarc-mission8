#!/usr/bin/env python2
import sys

import math
import cv2
from tf.transformations import *
import numpy as np

'''

Coordinate system: x is right, y is forward, z is up, distances are in meters,
angles are measured counterclockwise from positive x (right) axis
Direction goes from red (helmet) to blue (hand)

'''

PLAYER_HEIGHT = 1.5  # Height of human player color targets off the ground in meters
FOCAL_X = 520.6  # Camera focal length in pixels
FOCAL_Y = 514  # Camera focal length in pixels
IMAGE_WIDTH = 856
IMAGE_HEIGHT = 480
WEBCAM = False

if WEBCAM:
    FOCAL_X = 691
    FOCAL_Y = 689


def pointing_detection(image, pitch=math.pi/2, z=0, visualize=False):
    """
    Determines direction that human player is pointing and location of helmet
    pitch = 0 when camera pointed downwards, pitch = pi/2 when camera horizontal
    z = drone height off the ground in meters
    if visualize = true, then the annotated image will be displayed
    returns (angle in degrees CCW from +x axis, helmet position vector in drone frame)
    """
    pitch = math.pi/2  # camera automatically stabilizes, so pitch is constant
    pitch -= math.pi/4  # drone camera is angled downwards

    IMAGE_WIDTH = image.shape[1]
    IMAGE_HEIGHT = image.shape[0]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply red color filter (helmet)
    if WEBCAM:
        lower1 = np.array([171, 62, 116])
        upper1 = np.array([10, 255, 255])
    else:
        lower1 = np.array([171, 90, 190])
        upper1 = np.array([20, 255, 255])
    p1 = locate(hsv, lower1, upper1)

    # Apply blue color filter (hand)
    if WEBCAM:
        lower2 = np.array([98, 95, 76])
        upper2 = np.array([111, 255, 255])
    else:
        lower2 = np.array([98, 50, 76])
        upper2 = np.array([120, 255, 255])
    p2 = locate(hsv, lower2, upper2)

    if(p1 is None or p2 is None):
        return (None, None)

    # Simplified yaw estimate for comparison during testing
    # dx = p2[0]-p1[0]
    # dy = -p2[1]+p1[1] # image coordinate system is vertically flipped
    # dy = dy/math.cos(pitch)
    # yaw = math.atan2(dy, dx) - math.pi/2
    # print("Simple yaw")
    # print(yaw*180/math.pi)

    # Transformation based yaw estimate
    tvec_head = find_point(p1, z-PLAYER_HEIGHT, pitch)
    tvec_hand = find_point(p2, z-PLAYER_HEIGHT, pitch)

    dx = tvec_hand[0] - tvec_head[0]
    dy = tvec_hand[1] - tvec_head[1]
    yaw = math.atan2(dy, dx)
    if yaw < -math.pi:
        yaw += 2*math.pi

    # Display image
    if visualize:
        cv2.circle(image, p1, 10, (0, 0, 255), -1)
        cv2.circle(image, p2, 10, (255, 0, 0), -1)
        cv2.arrowedLine(image, p1, p2, (0, 255, 0), 5, -1)
        cv2.imshow('image', image)
        print("Gesture yaw: %s" % (yaw*180/math.pi))
        print("Head coordinates: %s" % (tvec_head))
        print("Hand coordinates: %s" % (tvec_hand))
    return (yaw, tvec_head)


def find_point(p, dz, pitch):
    ''' Locates point given coordinates in image, drone pitch, and height difference between drone and point.
        Returns a vector in drone right-front-up coordinates (meters)
    '''
    ang = math.atan2(p[1]-IMAGE_HEIGHT/2, FOCAL_Y)
    dist = (dz)/math.cos(pitch - ang)
    projectedDist = dist*math.cos(ang)
    tvec_cam = np.array([projectedDist*(p[0]-IMAGE_WIDTH/2)/FOCAL_X, -
                         projectedDist*(p[1]-IMAGE_HEIGHT/2)/FOCAL_Y, -projectedDist, 1])
    tvec_drone = np.dot(rotation_matrix(pitch, (1, 0, 0)), tvec_cam)
    return tvec_drone


def locate(image, lower, upper):
    if lower[0] > upper[0]:
        lower1 = np.array(lower)
        upper1 = np.array([180, upper[1], upper[2]])
        lower2 = np.array([0, lower[1], lower[2]])
        upper2 = np.array(upper)
        mask1 = cv2.inRange(image, lower1, upper1)
        mask2 = cv2.inRange(image, lower2, upper2)
        frame1 = cv2.bitwise_and(image, image, mask=mask1)
        frame2 = cv2.bitwise_and(image, image, mask=mask2)
        frame = cv2.bitwise_or(frame1, frame2)
    else:
        mask = cv2.inRange(image, lower, upper)
        frame = cv2.bitwise_and(image, image, mask=mask)
    ret, frame_1 = cv2.threshold(frame, 1, 255, cv2.THRESH_BINARY)
    frame = cv2.cvtColor(frame_1, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((3, 3), np.uint8)
    frame = cv2.erode(frame, kernel, iterations=1)
    frame, contours, hierarchy = cv2.findContours(
        frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = filter(lambda x: len(x) > 10, contours)
    contours = sorted(contours, key=cv2.contourArea)
    if not len(contours):
        return None
    target = contours[-1]

    M = cv2.moments(target)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return (cX, cY)


# Test code for processing data directly from the computer webcam or a saved image file
if __name__ == '__main__':
    # pitch = 60*math.pi/180
    pitch = 60*math.pi/180  # at pitch = 0 camera faces down
    # z = 2;
    z = .75  # height of camera above ground

    if len(sys.argv) == 2:
        filename = sys.argv[1]
        if filename == 'webcam':
            # Process the webcam in realtime
            # "0" here means "the first webcam on your system"
            camera = cv2.VideoCapture(0)

            while True:
                # Get the latest image from the webcam
                ok, image = camera.read()
                if not ok:
                    print("Unable to open webcam...")
                    break

                # Process it
                coordinates = pointing_detection(image, pitch, z, True)
                print(coordinates[0]*180/math.pi)

                # Exit if the "Esc" key is pressed
                key = cv2.waitKey(1)
                if key == 27:
                    break
        else:
            # Read data from an image file
            image = cv2.imread(filename)
            count = pointing_detection(image, pitch, z, True)
            print(count)
            cv2.waitKey(0)

    else:
        print('Usage: When you run this script, provide either the filename of an image or the string "webcam"')

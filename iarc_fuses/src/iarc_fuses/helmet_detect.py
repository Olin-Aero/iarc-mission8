#!/usr/bin/env python2

# import the necessary packages
from __future__ import print_function
from imutils.object_detection import non_max_suppression
import numpy as np
import imutils
from detect_config import PERSON_HSV_THRESHOLDS
from object_track import Object_Tracker
import cv2
import utils


def detect_helmet(image, show=True):
    frame_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    disp_contours = np.zeros(image.shape[:2], dtype="uint8")
    kernel = np.ones((5,5),np.uint8)
    # frame_HSV = cv2.dilate(frame_HSV,kernel,iterations = 1)
    # frame_HSV = cv2.dilate(frame_HSV)
    frame_threshold = cv2.inRange(frame_HSV, PERSON_HSV_THRESHOLDS['lowHSV'], PERSON_HSV_THRESHOLDS['highHSV'])
    if show:
      cv2.imshow("threshold", frame_threshold)

    im2, contours, hierarchy = cv2.findContours(frame_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None
    # print("_____________________")
    # print(contours)
    # contours = imutils.grab_contours(contours)
    contour = max(contours, key=cv2.contourArea)
    disp_contours = cv2.drawContours(disp_contours, [contour], 0, (255,255,255), 1)
    if show:
      cv2.imshow("contours", disp_contours)
    return contour


def main():
    vidPath = './schommer.mp4'
    cap = cv2.VideoCapture(vidPath)

    # Start tracker by initializing state
    ret, image = cap.read()
    # tracker.start_tracking(image, [334, 128, 438, 188])

    # startedTracking = False
    while cap.isOpened():
        ret, image = cap.read()
        if not ret: # If we have reached the end of the video
          break
        image = cv2.resize(image, None, fx=.5, fy=.5)
        detect_helmet(image)
        # res = utils.convert_to_pixels(np.shape(image), tracker.track(image))
        # print(tracker.state)
        # cv2.rectangle(image, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 3)
        # Show image
        cv2.imshow("object detection", image)

        # Press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()

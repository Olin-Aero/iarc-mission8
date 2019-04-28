#!/usr/bin/env python2

import cv2
import numpy as np
import utils
import rospy
import helmet_detect
import object_detection_tf


class Person_Helmet_Tracker:
    def __init__(self):
        self.person_detector = object_detection_tf.ObjectDetectorTF(
            use_gpu=False, cmap={1: "person"}
        )
        # boxes are upper-left y, x, lower-right y, x, as fractions of the image
        self.boxes = []  # pairs of (box, time last seen)

    def __call__(self, img, time, show):
        """
        Find people and their helmets.

        Takes an image and the ros-time that it was taken.  The time is used
        for filtering over time, to prevent jitter.  Detects humans using
        object_detection_tf, and finds helmets within them using helmet_detect.
        """
        if time is None:
            time = rospy.get_rostime()

        # Make a copy so we don't mess with the original
        if show:
            img_to_show = img.copy()

        # Find people
        objs = self.person_detector(img)  # Find all objs
        # Select objects with high scores and the right class, split out their scores and boxes
        mask = (objs["score"] > 0.5) & (objs["class"] == "person")
        boxes = objs["box"][mask]
        scores = objs["score"][mask]

        # TODO: Filter boxes using the past

        # Draw bounding boxes
        if show:
            for (box, score) in zip(boxes, scores):
                object_detection_tf.draw_bbox(img_to_show, box)

        # Search for helmets in the bounding boxes
        size_ratio_threshold = 0.01
        size_threshold = 100
        helmets = []
        for box in boxes:
            # Convert fractions to pixel counts
            bounds = np.round(
                box * np.array([img.shape[0], img.shape[1], img.shape[0], img.shape[1]])
            ).astype(int)
            subimg = img[bounds[0] : bounds[2], bounds[1] : bounds[3], :]
            helmet = helmet_detect.detect_helmet(subimg, show=False)
            if helmet is None:
                break
            # Is the helmet big enough to matter?
            box_area = abs((bounds[2] - bounds[0]) * (bounds[3] - bounds[1]))
            helmet_area = cv2.contourArea(helmet)
            if not (
                helmet_area / box_area > size_ratio_threshold
                and helmet_area > size_threshold
            ):
                break
            # Move contour from relative to subimg to relative to whole image
            helmet = utils.translate_contour(helmet, (bounds[1], bounds[0]))
            helmets += [helmet]

            # Outline helmets
            if show:
                cv2.drawContours(img_to_show, [helmet], 0, (255, 255, 255), 1)

        if show:
            cv2.imshow("boxes", img_to_show)


if __name__ == "__main__":

    rospy.init_node("person_helmet_detect")

    filename = "schommer.mp4"
    cam = cv2.VideoCapture(filename)

    tracker = Person_Helmet_Tracker()

    while True:
        ret, img = cam.read()
        if not ret:
            break

        tracker(img, None, True)

        k = cv2.waitKey(1)
        if k in [ord("q"), 27]:
            print("quitting...")
            break

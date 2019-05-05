#!/usr/bin/env python2

# TODO: Can we use a neural net to find helmets directly?  This model does
# not recognize helmets (see yolov3.txt, which afaict lists the things this
# model can see, though it actually comes from elsewhere.)

# TODO: Dynamically readjust helmet thresholds based on ambient lighting /
# whole-image properties

# TODO: ROS integration

import cv2
import numpy as np
import utils
import rospy

import helmet_detect
import object_detection_tf
import bbox_time_filter


class Person_Helmet_Tracker:
    def __init__(self):
        self.person_detector = object_detection_tf.ObjectDetectorTF(
            use_gpu=False, cmap={1: "person"}
        )
        # boxes are upper-left y, x, lower-right y, x, as fractions of the image
        self.boxes = []  # pairs of (box, time last seen)

        # TODO: Set sane values for filter
        self.time_filter = bbox_time_filter.Filter(
            min_age=1.0, max_age=0.2, max_side_dist=0.05, max_far_sides=2
        )

    def __call__(self, img, now=None, show=False):
        """
        Find people and their helmets.

        Takes an image and the ros-time that it was taken.  The time is used
        for filtering over time, to prevent jitter.  Detects humans using
        object_detection_tf, and finds helmets within them using helmet_detect.
        Returns filtered boxes which contain helmets, and the helmets inside
        them.

        Parameters
        ----------
        img : numpy.ndarray
          The image in which to look for people and helmets.  For best results,
          keep dimensions constant among calls.
        now : rospy.rostime.Time
          The time when the image was taken.  Should monotonically increase;
          for best results, should strictly increase.
        show : bool
          if True, this will draw an opencv window showing bounding boxes and
          helmet contours.

        """
        if now is None:
            now = rospy.get_rostime()

        # Make a copy so we don't mess with the original, which is passed by ref
        if show:
            img_to_show = img.copy()

        # Find people
        objs = self.person_detector(img)  # Find all objs
        # Select objects with high scores and the right class, split out their scores and boxes
        mask = (objs["score"] > 0.5) & (objs["class"] == "person")
        boxes = objs["box"][mask]

        filtered_boxes = self.time_filter(boxes, now)

        # Draw bounding boxes
        if show:
            for box in boxes:
                utils.draw_bbox(img_to_show, box)
            for box in filtered_boxes:
                # TODO: Display first/last-seen
                utils.draw_bbox(img_to_show, box, "filtered")

        # Search for helmets in the bounding boxes
        size_ratio_threshold = 0.01
        size_threshold = 100
        boxes_with_helmets = []
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
            # Move contour from relative to subimg to relative to whole image
            helmet = utils.translate_contour(helmet, (bounds[1], bounds[0]))
            # Is the helmet big enough to matter?
            box_area = abs((bounds[2] - bounds[0]) * (bounds[3] - bounds[1]))
            helmet_area = cv2.contourArea(helmet)
            if (
                helmet_area / box_area > size_ratio_threshold
                and helmet_area > size_threshold
            ):
                boxes_with_helmets += [box]
                helmets += [helmet]
                if show:
                    cv2.drawContours(img_to_show, [helmet], 0, (255, 255, 255), 1)
            else:
                if show:
                    cv2.drawContours(img_to_show, [helmet], 0, (255, 0, 0), 1)

        # Show the image with annotations
        if show:
            cv2.imshow("person_helmet_detect", img_to_show)

        return (boxes_with_helmets, helmets)

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

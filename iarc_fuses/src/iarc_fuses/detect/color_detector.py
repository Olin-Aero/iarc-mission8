#!/usr/bin/env python2

import numpy as np
import cv2
import warnings
import sys
from matplotlib import pyplot as plt
import utils

class ColorDetector:
    def __init__(self):
        self.color_space = None  # Set color_space using set_detection_color
        self.color_lower = None
        self.color_upper = None

    def set_detection_color(self, lower_threshold, upper_threshold, color_space="HSV"):
        self.color_space = color_space
        self.color_lower = np.array(lower_threshold, np.uint8)
        self.color_upper = np.array(upper_threshold, np.uint8)

    def detect_color(image, bounding_box=None):
        if self.color_space == "HSV":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            color = utils.inRangeWrap(image, self.color_lower, self.color_upper)
            if self.color_lower is None or self.color_upper is None:
                interactiveSession = Interactive_Threshold_Setter()
                interactiveSession.set_capture()

        elif self.color_space == "BGR":
            # TODO: Write BGR detector
            pass  # Image is expected to be in BGR space (default for opencv)
        elif self.color_space == "LAB":
            # TODO: Write LAB detector
            image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        else:
            raise Exception(
                'color_space "{}" is not a valid color space.'
                "Select from supported types".format(color_space)
            )

        # Morphological transformation, Dilation
        kernal = np.ones((5, 5), "uint8")
        blue = cv2.dilate(yellow, kernal)
        res = cv2.bitwise_and(img, img, mask=yellow)




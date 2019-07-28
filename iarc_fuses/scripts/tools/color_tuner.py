#!/usr/bin/env python2
import numpy as np
import cv2
import warnings
import sys
from matplotlib import pyplot as plt
from iarc_fuses import utils

# ColorTuner uses much code from
# https://github.com/opencv/opencv/blob/3.4/samples/python/tutorial_code/imgProc/threshold_inRange/threshold_inRange.py
class ColorTuner(object):
    def __init__(self):
        self.max_value = 255
        self.max_value_H = 360 // 2
        self.threshold_content_limit = (
            100
        )  # Percent of pixels contained within threshold
        self.low_confidence = 50  # Set the range of values acceptable
        self.high_confidence = 100
        self.low_H = 0
        self.low_S = 0
        self.low_V = 0
        self.high_H = self.max_value_H
        self.high_S = self.max_value
        self.high_V = self.max_value

        self.down_point = None
        self.up_point = None
        self.color_rect = None
        self.in_click = False

        self.window_capture_name = "Raw Source"
        self.window_detection_name = "Thresholded Source"
        self.low_H_name = "Low H"
        self.low_S_name = "Low S"
        self.low_V_name = "Low V"
        self.high_H_name = "High H"
        self.high_S_name = "High S"
        self.high_V_name = "High V"
        self.confidence_name = "Pixel Percentile"
        self.switch_name = "0 : OFF \n1 : Use Values"
        self.create_window()
        self.HSV_hists = [None, None, None]
        self.hists = None
        self.hsv_cropped = None

        self.create_trackbars()
        cv2.setMouseCallback(self.window_capture_name, self.on_mouse)
        self.source = None
        self.source_type = None

    def run(self):
        # Display Video if all else fails
        if self.source is None:
            self.set_capture(None)
            warnings.warn(
                "Capture is not set (using default). Set using the set_capture method."
            )

        while True:
            ## [while]
            if self.source_type == "VIDEO":
                ret, frame = self.source.read()
            elif self.source_type == "IMAGE":
                frame = self.source
            if frame is None:
                break

            frame_draw = np.copy(frame)
            if self.color_rect != None:
                cv2.rectangle(
                    frame_draw,
                    tuple(self.color_rect[0]),
                    tuple(self.color_rect[1]),
                    (255, 255, 255),
                )

            self.frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            if self.hsv_cropped is not None:
                self.plot_histogram(self.hsv_cropped)

                # Update the cropped region
                if self.down_point != self.up_point:
                    self.hsv_cropped = self.frame_HSV[
                        self.down_point[1] : self.up_point[1],
                        self.down_point[0] : self.up_point[0],
                    ]
            else:
                self.plot_histogram(self.frame_HSV)
            # print((self.(self.low_H, self.low_S, self.low_V))
            low_b = (self.low_H, self.low_S, self.low_V)
            high_b = (self.high_H, self.high_S, self.high_V)
            self.frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # frame_threshold = cv2.inRange(self.frame_HSV, low_b, high_b)
            ## [while]
            frame_threshold = utils.inRangeWrap(
                self.frame_HSV,
                (self.low_H, self.low_S, self.low_V),
                (self.high_H, self.high_S, self.high_V),
            )
            ## [while]

            cv2.imshow(self.window_capture_name, frame_draw)
            cv2.imshow(self.window_detection_name, frame_threshold)
            if self.hists is not None:
                cv2.imshow("plot", self.hists)

            switch = cv2.getTrackbarPos(self.switch_name, self.window_detection_name)

            key = cv2.waitKey(30)
            if key == ord("q") or key == 27 or switch != 0:
                if self.source_type == "VIDEO":
                    self.source.release()
                cv2.destroyAllWindows()
                return {
                    "lowHSV": (self.low_H, self.low_S, self.low_V),
                    "highHSV": (self.high_H, self.high_S, self.high_V),
                }

    def nothing(self, event):
        pass

    def plot_histogram(self, image):

        color = ("b", "g", "r")

        for i, col in enumerate(color):
            if not image.any():
                break
            histr = cv2.calcHist(image[:, :, i], [0], None, [256], [0, 256])
            if self.HSV_hists[i] == None:
                self.HSV_hists[i], = plt.plot(histr, color=col)
            else:
                self.HSV_hists[i].set_ydata(histr)
            plt.xlim([0, 256])

        self.fig.canvas.draw()
        # convert canvas to image
        self.hists = np.fromstring(
            self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep=""
        )
        self.hists = self.hists.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))

        # img is rgb, convert to opencv's default bgr
        self.hists = cv2.cvtColor(self.hists, cv2.COLOR_RGB2BGR)

    def set_source(self, source=None):
        if source is None:
            self.source = cv2.VideoCapture(0)
            self.source_type = "VIDEO"
        elif isinstance(source, cv2.VideoCapture):
            self.source = source
            self.source_type = "VIDEO"
        elif isinstance(source, np.ndarray):
            self.source_type = "IMAGE"
            self.source = source

    def create_window(self):
        cv2.namedWindow(self.window_capture_name)
        cv2.namedWindow(self.window_detection_name)
        self.fig = plt.figure()

    def create_trackbars(self):
        cv2.createTrackbar(
            self.low_H_name,
            self.window_detection_name,
            self.low_H,
            self.max_value_H,
            self.on_low_H_thresh_trackbar,
        )
        cv2.createTrackbar(
            self.high_H_name,
            self.window_detection_name,
            self.high_H,
            self.max_value_H,
            self.on_high_H_thresh_trackbar,
        )
        cv2.createTrackbar(
            self.low_S_name,
            self.window_detection_name,
            self.low_S,
            self.max_value,
            self.on_low_S_thresh_trackbar,
        )
        cv2.createTrackbar(
            self.high_S_name,
            self.window_detection_name,
            self.high_S,
            self.max_value,
            self.on_high_S_thresh_trackbar,
        )
        cv2.createTrackbar(
            self.low_V_name,
            self.window_detection_name,
            self.low_V,
            self.max_value,
            self.on_low_V_thresh_trackbar,
        )
        cv2.createTrackbar(
            self.high_V_name,
            self.window_detection_name,
            self.high_V,
            self.max_value,
            self.on_high_V_thresh_trackbar,
        )
        cv2.createTrackbar(
            self.confidence_name,
            self.window_detection_name,
            80,
            self.high_confidence,
            self.on_confidence_trackbar,
        )
        # create switch for ON/OFF functionality
        cv2.createTrackbar(
            self.switch_name, self.window_detection_name, 0, 1, self.nothing
        )

    def update_all_trackbars(self):
        cv2.setTrackbarPos(self.low_H_name, self.window_detection_name, self.low_H)
        cv2.setTrackbarPos(self.low_S_name, self.window_detection_name, self.low_S)
        cv2.setTrackbarPos(self.low_V_name, self.window_detection_name, self.low_V)
        cv2.setTrackbarPos(self.high_H_name, self.window_detection_name, self.high_H)
        cv2.setTrackbarPos(self.high_S_name, self.window_detection_name, self.high_S)
        cv2.setTrackbarPos(self.high_V_name, self.window_detection_name, self.high_V)

    def on_mouse(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.in_click = True
            self.down_point = [x, y]

        if event == cv2.EVENT_LBUTTONUP:
            self.in_click = False
            self.up_point = [x, y]
            if self.up_point == self.down_point:
                self.low_H = int(self.frame_HSV[y][x][0])
                self.low_S = int(self.frame_HSV[y][x][1])
                self.low_V = int(self.frame_HSV[y][x][2])
                self.high_H = int(self.low_H)
                self.high_S = int(self.low_S)
                self.high_V = int(self.low_V)
                self.color_rect = None
            else:
                for i in range(0, 2):
                    if self.down_point[i] > self.up_point[i]:
                        self.down_point[i], self.up_point[i] = (
                            self.up_point[i],
                            self.down_point[i],
                        )

                self.hsv_cropped = self.frame_HSV[
                    self.down_point[1] : self.up_point[1],
                    self.down_point[0] : self.up_point[0],
                ]
                if not self.hsv_cropped.any():
                    return
                if self.threshold_content_limit < 50:
                    warnings.warn(
                        "threshold_content_limit must be greater than 50, setting to 50."
                    )
                    self.threshold_content_limit = 50

                self.update_confidence()
                # self.up_point = None
                # self.down_point = None

            if self.down_point != None:
                self.color_rect = (self.down_point, self.up_point)

        if self.in_click:
            self.color_rect = (self.down_point, (x, y))

        self.update_all_trackbars()

    def update_confidence(self):
        if self.hsv_cropped is not None:
            low_b = np.percentile(
                self.hsv_cropped, 100 - self.threshold_content_limit, axis=(0, 1)
            )
            high_b = np.percentile(
                self.hsv_cropped, self.threshold_content_limit, axis=(0, 1)
            )

            self.low_H = int(low_b[0])
            self.low_S = int(low_b[1])
            self.low_V = int(low_b[2])
            self.high_H = int(high_b[0])
            self.high_S = int(high_b[1])
            self.high_V = int(high_b[2])

    def on_confidence_trackbar(self, val):
        if val < 50:
            val = 50
        self.threshold_content_limit = val
        self.update_confidence()
        cv2.setTrackbarPos(
            self.confidence_name,
            self.window_detection_name,
            self.threshold_content_limit,
        )

    def on_low_H_thresh_trackbar(self, val):
        self.low_H = val
        cv2.setTrackbarPos(self.low_H_name, self.window_detection_name, self.low_H)

    def on_high_H_thresh_trackbar(self, val):
        self.high_H = val
        cv2.setTrackbarPos(self.high_H_name, self.window_detection_name, self.high_H)

    def on_low_S_thresh_trackbar(self, val):
        self.low_S = min(self.high_S - 1, val)
        cv2.setTrackbarPos(self.low_S_name, self.window_detection_name, self.low_S)

    def on_high_S_thresh_trackbar(self, val):
        self.high_S = max(val, self.low_S + 1)
        cv2.setTrackbarPos(self.high_S_name, self.window_detection_name, self.high_S)

    def on_low_V_thresh_trackbar(self, val):
        self.low_V = min(self.high_V - 1, val)
        cv2.setTrackbarPos(self.low_V_name, self.window_detection_name, self.low_V)

    def on_high_V_thresh_trackbar(self, val):
        self.high_V = max(val, self.low_V + 1)
        cv2.setTrackbarPos(self.high_V_name, self.window_detection_name, self.high_V)


def main():
    cap = cv2.VideoCapture(sys.argv[1])
    mySess = ColorTuner()
    ret, img = cap.read()
    img = cv2.resize(img, None, fx=0.5, fy=0.5)
    mySess.set_source(img)
    x = mySess.run()
    print(x)


if __name__ == "__main__":
    main()

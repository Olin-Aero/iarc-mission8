#!/usr/bin/env python2
import rospy
import sys
import os
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from std_msgs.msg import Int32
from math import *
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from iarc_forebrain.msg import ImageBin
from iarc_fuses.detect.qr_detector import QRDetector
from iarc_fuses.detect.qr_combiner import QRCombiner

def convert_nparray_to_PIL(x):
    cv2_im = x
    cv2_im = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
    return PILImage.fromarray(cv2_im)


class QRNode():

    def __init__(self):
        # processing handles
        self.detector = QRDetector()
        self.combiner = QRCombiner()

        # ROS Handles
        rospy.init_node('QRDetector', anonymous=True)
        self.imageSub = rospy.Subscriber('/qr_image', ImageBin, self.image_cb)
        self.qrNumberPub = rospy.Publisher('/qr_number', Int32, queue_size=10)
        self.bridge = CvBridge()
        self.currentImage = None
        self.images = [None for _ in range(4)]
        self.number = None
        self.nextBin = 0

    def image_cb(self, data):
        # Gets the image from the respective drone, and puts it in the specified bin
        # If the "bin number" is 0, it puts it in the first empty bin
        if(data.bin == 0):
            for index, image in enumerate(self.images):
                if(image is None):
                    self.images[index] = self.bridge.imgmsg_to_cv2(
                        data.image, desired_encoding="passthrough")
                    return
            rospy.logwarn("Already have 4 images, call reset")
        elif(data.bin <= 4):
            self.images[data.bin - 1] = self.bridge.imgmsg_to_cv2(
                data.image, desired_encoding="passthrough")
        else:
            rospy.logwarn("Not a valid bin number")

    def reset_cb(self):
        # TODO : have a ros topic, service, something that's going to call this
        self.images = [None for _ in range(4)]
        self.number = None

    def process(self):
        # saves the images so that QRCombiner can access them.
        images = self.images
        finalImages = []

        # check if all images are available
        for image in images:
            if (image is None):
                return

        if self.number is None:
            # don't have the number yet
            for image in images:
                croppedImage = self.detector(image)
                finalImages.append(
                    croppedImage)  # adds the image to the collection of final images

            # combine and decode QR Codes
            pil_images = [convert_nparray_to_PIL(e) for e in finalImages]
            number = self.combiner(pil_images)
            print('QR decoded to : {}'.format(number))
            self.number = number

        # now broadcast the result
        if self.number is not None:
            self.qrNumberPub.publish(number)
        else:
            # indicate failure
            self.qrNumberPub.publish(-1)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()


def main():
    det = QRNode()
    det.run()

if __name__ == '__main__':
    main()

#!/usr/bin/env python2
import rospy
import sys
import os
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from math import *
from cv_bridge import CvBridge, CvBridgeError

from iarc_fuses.qr_detector import QRDetector
from iarc_fuses.qr_combiner import QRCombiner

def convert_nparray_to_PIL(x):
    # Nathan will do this
    pass

class QRNode():
    def __init__(self):
        # processing handles
        self.detector = QRDetector()
        self.combiner = QRCombiner()

        # ROS Handles
        rospy.init_node('QRDetector', anonymous=True)
        self.drone1ImageSub = rospy.Subscriber('/drone1/image_raw',Image,self.drone1CB) # The ros interfacing won't work yet as I haven't figured out how to convert a ROS image to PIL Image
        self.drone2ImageSub = rospy.Subscriber('/drone2/image_raw',Image,self.drone2CB)
        self.drone3ImageSub = rospy.Subscriber('/drone3/image_raw',Image,self.drone3CB)
        self.drone4ImageSub = rospy.Subscriber('/drone4/image_raw',Image,self.drone4CB)
        self.qrNumberPub = rospy.Publisher('/qr_number', Int32, queue_size=10)
        self.bridge = CvBridge()

        self.images = [None for _ in range(4)]
        self.number = None

    def drone1CB(self,imageMessage):
        self.images[0] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")
    def drone2CB(self,imageMessage):
        self.images[1] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")
    def drone3CB(self,imageMessage):
        self.images[2] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")
    def drone4CB(self,imageMessage):
        self.images[3] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")

    def resetCB(self):
        # TODO : have a ros topic, service, something that's going to call this
        self.images = [None for _ in range(4)]
        self.number=None

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
                qr = self.detector(image)
                finalImages.append(croppedImage) # adds the image to the collection of final images

            # combine and deocde QR Codes
            # TODO: convert images to PIL Image before passing to combiner
            pil_images = [convert_nparray_to_PIL(e) for e in finalImages]
            number = self.combiner(pil_images)

        # now broadcast the result
        number = self.number
        self.qrNumberPub.publish(number)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()

def main():
    det = QRNode()
    det.run()

if __name__ == '__main__':
    main()

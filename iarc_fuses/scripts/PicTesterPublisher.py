#!/usr/bin/env python2
import rospy
import sys
import os
import cv2
import numpy as np
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class PicTester():
    def __init__(self):
        rospy.init_node('PicTester', anonymous=True)
        self.picPub = rospy.Publisher('/drone1/image_raw',Image,queue_size=10)
        self.bridge = CvBridge()
    
    def process(self):
        rospack = rospkg.RosPack() 
        pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
        for index in range(4):
            rospy.loginfo(index)
            file = os.path.join(pkgRoot,"figs","QROfficial","%s_2.jpg" %(index+1))
            cv_image = cv2.imread(file)
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            self.picPub.publish(image_message)
            rospy.sleep(5)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()

def main():
    det = PicTester()
    det.run()

if __name__ == '__main__':
    main()

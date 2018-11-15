#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import *
import numpy as np
import math
import cv2
import sys
from pointing_detection import pointing_detection
from util.Drone import Drone

CAM_PITCH = math.pi/2
D = 2

class FollowGesture:

    def __init__(self):
        rospy.init_node('follow_gesture')
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/gesture_direction", Float64, queue_size=10)
        self.drone = Drone()
        rate = rospy.Rate(1) # 1 Hz
        rate.sleep()
        rospy.Subscriber("/ardrone/front/image_raw", Image, self.image_raw_callback)

    def image_raw_callback(self, msg):
        try:
          # frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = msg

            pos = self.drone.get_pos()
            
            o = pos.pose.orientation
            orientation = euler_from_quaternion([o.w, o.x, o.y, o.z])

            direction, helmet = pointing_detection(frame, -orientation[1]+CAM_PITCH, pos.pose.position.z, True)
            self.pub.publish(direction)


            dx = D*math.cos(direction+orientation[2])
            dy = D*math.sin(direction+orientation[2])
            self.drone.move_to(pos.pose.position.x+dx, pos.pose.position.y+dy)

        except CvBridgeError as e:
            print(e)
      
    def run(self):
        # Keep the program running
        # TODO wait for signal
        rate = rospy.Rate(1) # 1 Hz
        rate.sleep()
        while True:
            rate.sleep()
            # ok, image = self.camera.read()
            # if not ok:
            #     print("Unable to open webcam...")
            #     return
            # # self.image_raw_callback(image)
            # key = cv2.waitKey(1)
            # if key == 27:
            #     return
            # break

        # rospy.spin()

# Start the node
if __name__ == '__main__':
    test = FollowGesture()
    # test.camera = cv2.VideoCapture(0)  # "0" here means "the first webcam on your system"
    test.run()
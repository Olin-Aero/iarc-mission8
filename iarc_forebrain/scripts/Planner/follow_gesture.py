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
from mode import Mode
from Drone import Drone

SAMPLE_PERIOD = .1
CAM_PITCH = math.pi/2

class FollowGesture(Mode):

    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/gesture_direction", Float64, queue_size=10)
        self.drone = Drone()
        self.prevTime = rospy.Time.now()
        self.distance = 0
        rate = rospy.Rate(1) # 1 Hz
        rate.sleep()
        rospy.Subscriber("/bebop/image_raw", Image, self.image_raw_callback)

    def image_raw_callback(self, msg):
    	if not self.is_active():
    		return
        try:
            if((rospy.Time.now()-self.prevTime).to_sec()<SAMPLE_PERIOD):
                return
            self.prevTime = rospy.Time.now()
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            pos = self.drone.get_pos("odom")
            o = pos.pose.orientation
            orientation = euler_from_quaternion([o.w, o.x, o.y, o.z])
            direction, helmet = pointing_detection(frame, -orientation[1]+CAM_PITCH, pos.pose.position.z, True)
            if direction is None:
                return
            self.pub.publish(direction)

            dx = self.distance*math.cos(direction+orientation[2])
            dy = self.distance*math.sin(direction+orientation[2])
            print(pos.pose.position.x+dx, pos.pose.position.y+dy)
            self.drone.move_towards(pos.pose.position.x+dx, pos.pose.position.y+dy)
            self.disable()
            key = cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def enable(self, distance='0', units='meters'):
        self.active = True
        self.distance = self.parse(distance, units)
        print('distance: ' + str(self.distance))

# Start the node
if __name__ == '__main__':
    f = FollowGesture()
    f.test()

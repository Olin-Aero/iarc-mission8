#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import *
import numpy as np
import math
import cv2
import sys
from pointing_detection import pointing_detection
from mode import Mode
from move import Move
from iarc_arbiter.drone import Drone

SAMPLE_PERIOD = .1
CAM_PITCH = math.pi/2


class FollowGesture(Mode):

    def __init__(self, drone, translate=True, debug=True):
        self.bridge = CvBridge()
        self.dir_pub = rospy.Publisher(
            drone.namespace+"gesture_direction", Float64, queue_size=10)
        self.helm_pub = rospy.Publisher(
            drone.namespace+"helmet_pos", PointStamped, queue_size=10)
        self.drone = drone
        self.prevTime = rospy.Time.now()
        self.distance = 0
        self.translate = translate
        self.move = None
        self.detected = False
        self.debug = debug
        rate = rospy.Rate(1)  # 1 Hz
        rate.sleep()
        rospy.Subscriber(drone.namespace+"image_raw", Image, self.image_raw_callback)
        # rospy.Subscriber("/ardrone/front/image_raw", Image, self.image_raw_callback)

    def image_raw_callback(self, msg):
        if self.detected or not self.is_active():
            return
        try:
            if((rospy.Time.now()-self.prevTime).to_sec() < SAMPLE_PERIOD):
                return
            self.prevTime = rospy.Time.now()
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            pos = self.drone.get_pos("odom")
            o = pos.pose.orientation
            orientation = euler_from_quaternion([o.x, o.y, o.z, o.w])
            direction, h_pos = pointing_detection(
                frame, -orientation[1]+CAM_PITCH, pos.pose.position.z, self.debug)
            if direction is None:
                return
            self.dir_pub.publish(direction)
            helmet = PointStamped()
            helmet.point.x = h_pos[0] * math.cos(orientation[2]-math.pi/2) - h_pos[1] * math.sin(
                orientation[2]-math.pi/2) + pos.pose.position.x
            helmet.point.y = h_pos[0] * math.sin(orientation[2]-math.pi/2) + h_pos[1] * math.cos(
                orientation[2]-math.pi/2) + pos.pose.position.y
            helmet.point.z = h_pos[2] + pos.pose.position.z
            helmet.header.frame_id = self.drone.prefix("odom")
            self.helm_pub.publish(helmet)
            if self.translate:
                self.move = Move(self.drone, direction +
                                 orientation[2]-math.pi/2)
                self.move.enable(self.distance)
                self.detected = True
            if self.debug:
                key = cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def enable(self, distance='0', units='meters'):
        self.distance = self.parse(distance, units)
        self.active = True
        self.detected = False
        if self.translate:
            print('FOLLOW GESTURE: dist = ' + str(self.distance))
        else:
            print('Player Detection Activated')

    def update(self, look_direction=0, obstacles=[]):
        if self.detected:
            self.move.update(look_direction, obstacles)


# Start the node
if __name__ == '__main__':
    f = FollowGesture(Drone())
    f.test()

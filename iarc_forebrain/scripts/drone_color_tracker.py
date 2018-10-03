#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from util.Drone import Drone


rospy.init_node('drone_color_tracker')

def callback():
	print(Point.x,Point.y)

sub = rospy.Subscriber("/color_target_coordinates", Point, callback())
while(1):
	rospy.spin()
#!/usr/bin/env python2
import rospy
from util.Drone import Drone

cam = Drone()
cam.go_to_camera()
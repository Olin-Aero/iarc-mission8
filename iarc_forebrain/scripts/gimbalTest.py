#!/usr/bin/env python2
import rospy
from util.Drone import Drone

cam = Drone()
cam.go_to_camera(-45.0,0.0,0.0)
cam.go_to_camera(-30.0,-15.0,0.0)
cam.go_to_camera(-30.0,0.0,-15.0)

#!/usr/bin/env python2
import rospy
import math
from util.Drone import Drone

drone = Drone()
drone.takeoff()
drone.travel_and_look(des_x=4.0, des_y=-4.0, focus_x=1.0, focus_y=1.0);
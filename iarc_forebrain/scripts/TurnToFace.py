#!/usr/bin/env python2
import rospy
import math
from util.Drone import Drone

drone = Drone()
drone.takeoff()
drone.look_at(1.5,-2);
#!/usr/bin/env python2
import rospy
import math
from util.Drone import Drone
import tf

drone = Drone()
drone.takeoff()
drone.turn_to(1.0);
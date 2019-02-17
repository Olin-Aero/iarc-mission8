#!/usr/bin/env python2
import rospy
import math
from util.Drone import Drone
import tf

drone = Drone()
drone.takeoff()
quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 1.0)
drone.turn_to(quat);
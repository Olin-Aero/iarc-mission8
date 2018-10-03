#!/usr/bin/env python2

from util.Drone import Drone
import rospy

drone = Drone()

drone.takeoff()
drone.move_with_velocity(0, 0, 1, 0)
rospy.sleep(2)
drone.hover(3)
drone.land()
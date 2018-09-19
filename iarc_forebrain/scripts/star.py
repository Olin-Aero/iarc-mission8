#!/usr/bin/env python2

import rospy
from util.Drone import Drone

drone = Drone()

rospy.logdebug("Drone about to take off...")
drone.takeoff()

rospy.logdebug("Moving to origin 1...")
drone.move_to(0.0, 0.0)
drone.hover(1)

rospy.logdebug("Moving to point 2...")
drone.move_to(-3.5, -3.0)
drone.hover(1)

rospy.logdebug("Moving to point 3...")
drone.move_to(-2.0, 2.0)
drone.hover(1)

rospy.logdebug("Moving to point 4...")
drone.move_to(-0.5, -3.0)
drone.hover(1)

rospy.logdebug("Moving to point 5...")
drone.move_to(-4.0, 0.0)
drone.hover(1)

rospy.logdebug("Moving back to origin...")
drone.move_to(0.0, 0.0)
drone.hover(3)

rospy.logdebug("Landing...")
drone.land()

#!/usr/bin/env python2

import rospy
from util.Drone import Drone

drone = Drone()

rospy.loginfo("Drone about to take off...")
drone.takeoff()

rospy.loginfo("Moving to point 1...")
drone.move_to(0.0, 0.0)
drone.hover(1)

rospy.loginfo("Moving to point 2...")
drone.move_to(-3.5, -3.0)
drone.hover(1)

rospy.loginfo("Moving to point 3...")
drone.move_to(-2.0, 2.0)
drone.hover(1)

rospy.loginfo("Moving to point 4...")
drone.move_to(-0.5, -3.0)
drone.hover(1)

rospy.loginfo("Moving to point 5...")
drone.move_to(-4.0, 0.0)
drone.hover(1)

rospy.loginfo("Moving back to point 1...")
drone.move_to(0.0, 0.0)
drone.hover(3)

rospy.loginfo("Landing...")
drone.land()

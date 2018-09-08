#!/usr/bin/env python2
import rospy
from Drone import Drone

rospy.init_node('behaviors')
drone = Drone()

def fly_hover():
    rospy.loginfo('Running Hover Behavior')
    rospy.sleep(1.0)

    rospy.loginfo('Taking off to 1.5 meters')
    drone.takeoff(1.5)

    rospy.loginfo('Hovering')
    drone.hover(5)

    rospy.loginfo('Landing')
    drone.land()

    rospy.loginfo('Done!')

def fly_forward():
    rospy.loginfo('Running Square Behavior')
    rospy.sleep(1.0)

    rospy.loginfo('Taking off to 1.5 meters')
    drone.takeoff(1.5)

    rospy.loginfo('Hovering')
    drone.hover(2)

    rospy.loginfo('Moving forward one meter (relative to launch site)')
    drone.move_to(1.0, 0.0, 'launch')
    rospy.loginfo('Hovering')
    drone.hover(2)

    rospy.loginfo('Landing')
    drone.land()

def fly_your_behavior():
    rospy.loginfo('Running ??? Behavior')
    rospy.sleep(1.0)

    rospy.loginfo('Taking off to 1.5 meters')
    drone.takeoff(1.5)

    rospy.loginfo('Hovering')
    drone.hover(2)

    # TODO(you) Implement something cool here!

    rospy.loginfo('Landing')
    drone.land()


fly_hover()

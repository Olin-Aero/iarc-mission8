#!/usr/bin/env python2
import rospy
from util.Drone import Drone

rospy.init_node('behaviors')
drone = Drone()


def fly_hover():
    """ The drone flies to a height of 1.5 meters for 5 seconds. """
    
    rospy.loginfo('Running Hover Behavior')
    rospy.sleep(1.0) # Delays the code from running by one second.

    rospy.loginfo('Taking off')
    drone.takeoff() # Makes the drone lift off from the ground. Default height is 1.5 meters. Putting a number in the pharentheses will change the height.

    rospy.loginfo('Hovering')
    drone.hover(5) # Makes the drone keep it's current position for 5 seconds. Changing the 5 to a different number will make it hover for that many seconds.

    rospy.loginfo('Landing')
    drone.land() # Makes the drone land back on the ground.

    rospy.loginfo('Done!')


def fly_forward():
    # Makes the drone hover at 1.5 meters for 2 seconds, fly one meter forward, then hover for another 2 seconds, then land.
    rospy.loginfo('Running Forward Behavior')
    rospy.sleep(1.0) # Delays the code from running by one second.

    rospy.loginfo('Taking off')
    drone.takeoff() # Makes the drone lift off from the ground. Default height is 1.5 meters. Putting a number in the pharentheses will change the height.

    rospy.loginfo('Hovering')
    drone.hover(2) # Makes the drone keep it's current position for 2 seconds. Changing the 2 to a different number will make it hover for that many seconds.

    rospy.loginfo('Moving forward one meter (relative to launch site)')
    drone.move_to(1.0, 0.0, 'launch') # Moves the robot one meter forward relative to the launch site. The first number is the x coordinate, the second is the y coordinate.
    rospy.loginfo('Hovering')
    drone.hover(2)

    rospy.loginfo('Landing')
    drone.land()


def fly_your_behavior():
    rospy.loginfo('Running ??? Behavior')
    rospy.sleep(1.0)

    rospy.loginfo('Taking off')
    drone.takeoff()

    rospy.loginfo('Hovering')
    drone.hover(2)

    # TODO(you) Implement something cool here!
    # Some ideas:
    # Make the drone fly in a square, then land.
    # Make the drone fly in an equilateral triange, then land.
    #
    rospy.loginfo('Landing')
    drone.land()


fly_forward()

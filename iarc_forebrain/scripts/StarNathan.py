#!/usr/bin/env python2
import rospy
from util.Drone import Drone

def moveDrone(drone,x,y):
	rospy.loginfo("moving to (%d , %d)"% (x, y))
	drone.move_to(x,y,'map')
	drone.hover(1)

jj_the_jet_plane = Drone()
rospy.loginfo("taking off")
jj_the_jet_plane.takeoff()
jj_the_jet_plane.hover(1)
moveDrone(jj_the_jet_plane,-4.0,0.0)
moveDrone(jj_the_jet_plane,-0.5,-3.0)
moveDrone(jj_the_jet_plane,-2.0,2.0)
moveDrone(jj_the_jet_plane,-3.5,-3.0)
moveDrone(jj_the_jet_plane,0.0,0.0)
rospy.loginfo("landing")
jj_the_jet_plane.land()
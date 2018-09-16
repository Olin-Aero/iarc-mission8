#!/usr/bin/env python2
import rospy
from util.Drone import Drone
JJ_The_Jet_Plane = Drone()

JJ_The_Jet_Plane.takeoff()
JJ_The_Jet_Plane.hover(1)
JJ_The_Jet_Plane.move_to(-4.0,0.0,'map')
JJ_The_Jet_Plane.hover(1)
JJ_The_Jet_Plane.move_to(-0.5,-3.0,'map')
JJ_The_Jet_Plane.hover(1)
JJ_The_Jet_Plane.move_to(-2.0,2.0,'map')
JJ_The_Jet_Plane.hover(1)
JJ_The_Jet_Plane.move_to(-3.5,-3.0,'map')
JJ_The_Jet_Plane.hover(1)
JJ_The_Jet_Plane.move_to(0.0,0.0,'map')
JJ_The_Jet_Plane.hover(1)
JJ_The_Jet_Plane.land()										
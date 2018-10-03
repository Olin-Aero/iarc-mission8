#!/usr/bin/env python2
""" Drone runs a path that will take off, move in a star formation (only outside points)
and will then return to the starting point and land"""
import rospy
from util.Drone import Drone
drone = Drone()
drone.takeoff()
drone.move_to(1.0,0.0,'launch') #launches
drone.hover(5)
drone.move_to(2.0,1.0,'launch') #moves to the outer edge of the star (right side)
drone.move_to(0.75,1.0,'launch') #ready to move to top point of star
drone.move_to(0.0,2.5,'launch') #moves to top point of the star
drone.move_to(-0.75,1.0,'launch')
drone.move_to(-2.0,1.0,'launch')#moves to left point of star
drone.move_to(-0.5,0.0,'launch')
drone.move_to(-1.5,-2.0,'launch')#bottom left point of star
drone.move_to(0.0,-0.5,'launch')
drone.move_to(1.5,-2.0,'launch')#bottom right point of star
drone.move_to(1.0,0.0,'launch')#returns to edge of star where it started
drone.hover(5)
drone.move_to(0.0,0.0,'launch')#moves to center of the started
drone.hover(3)
drone.land()#lands

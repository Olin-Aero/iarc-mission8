#!/usr/bin/env python2
import rospy
import time
from util.Drone import Drone

droney = Drone()
droney.takeoff()
r = rospy.Rate(10)
for _ in range(100):
    droney.travel_and_look(0.5,-0.5,0.0,0.0)
    r.sleep()
for _ in range(100):
    droney.travel_and_look(0.5,0.5,0.0,0.0)
    r.sleep()
for _ in range(100):
    droney.travel_and_look(-0.5,0.5,0.0,0.0)
    r.sleep()
for _ in range(100):
    droney.travel_and_look(-0.5,-0.5,0.0,0.0)
    r.sleep()
droney.land()

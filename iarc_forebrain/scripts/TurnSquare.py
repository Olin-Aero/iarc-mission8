#!/usr/bin/env python2
import rospy
from util.Drone import Drone

droney = Drone()
droney.takeoff()
droney.travel_and_look(1.0,-1.0,0.5,0.5)
droney.travel_and_look(1.0,1.0,0.5,0.5)
droney.travel_and_look(-1.0,1.0,0.5,0.5)
droney.travel_and_look(-1.0,-1.0,0.5,0.5)
#!/usr/bin/env python2

from util.Drone import Drone

drone = Drone()

drone.takeoff()

drone.move_to(0.0, 0.0)
drone.hover(1)
drone.move_to(-3.5, -3.0)
drone.hover(1)
drone.move_to(-2.0, 2.0)
drone.hover(1)
drone.move_to(-0.5, -3.0)
drone.hover(1)
drone.move_to(-4.0, 0.0)
drone.hover(1)
drone.move_to(0.0, 0.0)
drone.hover(3)
drone.land()

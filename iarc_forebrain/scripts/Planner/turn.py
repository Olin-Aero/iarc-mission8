#!/usr/bin/env python2

''' A simple one-time motion command '''
import rospy
import math
from mode import Mode
import numpy as np
from tf.transformations import *

class Turn(Mode):
    def __init__(self, drone, direction=0):
        Mode.__init__(self, drone)
        self.direction = direction # -1 = CW, 1 = CCW, 0 = absolute angle
        self.angle = 0

    def enable(self, target=0):
        self.active = True
        self.target = target
        if self.direction == 0:
            if target == "north":
                self.angle = 0
            elif target == "west":
                self.angle = np.pi/2
            elif target == "south":
                self.angle = np.pi
            elif target == "east":
                self.angle = 3*np.pi/2

    def get_look_direction(self, current_orientation, enable=False):
        ''' Returns drone orientation desired by mode as an angle in the odom frame (radians)
            The enable flag is true the first time this function is called'''
        if self.direction == 0:
            return self.angle
        else:
            if enable:
                return current_orientation + np.radians(float(self.target))*self.direction
            return current_orientation

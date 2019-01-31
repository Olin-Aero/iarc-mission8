#!/usr/bin/env python2
import numpy as np

import config as cfg
import rospy
import tf
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from random import choice
'''
robots.py

Contains two classes to represent target and obstacle roombas.

TODO : vel3d is bullshit
'''

class Drone(object):
    """
    Represents the drone in the simulation
    """

    def __init__(self, pos2d, heading, tag = ''):
        """
        Initialize the Drone object where:
        pos3d is a vector [x,y,z]
        vel3d is a vector [x',y',z']
        heading is an angle in radians (0 is +x and pi/2 is +y)
        """
        initial_height = 0 #Make this an arugment at some point
        self.vel3d = Twist()

        self.pos3d = [pos2d[0], pos2d[1], initial_height]
        self.heading = heading
        self.tag = tag

        self.obsticle_bump_count = 0

        # makes it so that drone has to distance itself before it can collide again
        self.can_collide = True
        self.collision_center = []

        self.OutOfBounds = False
        self.TimeOOB = 0

    def limitSpeed(self, speedLimit):
        currentSpeed = np.linalg.norm(self.vel3d)
        if currentSpeed > speedLimit:
            self.vel3d = self.vel3d * speedLimit/currentSpeed

    def collision(self, self_pos, self_heading, other_pos, other_heading, self_radius=cfg.DRONE_RADIUS, other_radius=cfg.OBSTACLE_POLE_RADIUS):

        dy = other_pos[1] - self_pos[1]
        dx = other_pos[0] - self_pos[0]

        d = np.sqrt(dx**2 + dy**2)

        if self.can_collide == True:
            if d < self_radius + other_radius:
                print("Collision with obsticle roomba pole")
                self.obsticle_bump_count += 1

                self.collision_center = [other_pos[0], other_pos[1]]
                self.can_collide = False

        if self.can_collide == False:
            if np.sqrt((self.collision_center[0] - self.pos3d[0]) ** 2 + (self.collision_center[1] - self.pos3d[1]) ** 2) > self_radius + other_radius + 0.35:
                self.can_collide = True
                self.collision_center = []

    def set_vel(self, data):
        self.vel3d = data

    def update(self, delta, elapsed):
        z_vel = self.vel3d.linear.z
        self.pos3d[2] += z_vel * delta
        if self.pos3d[2] < 0:
            self.pos3d[2] = 0

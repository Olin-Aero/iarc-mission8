#!/usr/bin/env python2

''' A simple one-time motion command '''
import rospy
import math
from mode import Mode

import numpy as np


class Move(Mode):
    TARGET_DIST_LOOK_THRESH = 0.5
    def __init__(self, drone, angle=0, dz=0, relative=False):
        Mode.__init__(self, drone)
        self.angle = angle
        self.dz = dz
        self.obstacles = []
        self.relative = relative
        self.name = drone.namespace.strip('/')

    def enable(self, distance=0, units=0):
        self.distance = self.parse(distance, units)
        pos = self.drone.get_pos("map").pose.position
        angle = self.angle
        if self.relative:
            angle += self.yaw
        if self.dz == 0:
            dx = self.distance*math.cos(angle)
            dy = self.distance*math.sin(angle)
            self.target = (pos.x+dx, pos.y+dy, pos.z)
            print('MOVE: dx = %s, dy = %s' % (dx, dy))
        else:
            self.target = (pos.x, pos.y, pos.z+self.dz*self.distance)
            print('MOVE: dz = %s' % (self.dz*self.distance))
        self.active = True

    def update(self, look_direction=0, obstacles=[]):
        pos = self.drone.get_pos("map").pose.position
        v = self.get_move_direction(
            [self.target[0]-pos.x, self.target[1]-pos.y], [(o[0]-pos.x, o[1]-pos.y) for o in obstacles])
        vec_to_target = np.array(self.target[:2]) - np.array([pos.x, pos.y])
        if np.linalg.norm(vec_to_target) >= self.TARGET_DIST_LOOK_THRESH:
            look_direction = math.atan2(vec_to_target[1], vec_to_target[0])
        dest = [v[0]+pos.x, v[1]+pos.y]
        # TODO: account for vertical obstacle distance
        x, y = pos.x+math.cos(look_direction), pos.y+math.sin(look_direction)
        self.drone.travel_and_look(
            dest[0], dest[1], x, y, "map", self.target[2])

    def get_move_direction(self, target=(0, 0), obstacles=[]):
        '''
        Returns optimal (vx, vy) based on gradient of potential field determined by target and 
        obstacle coordinates relative to current drone location
        '''
        AVOID_RADIUS = 1.0  # Closest that drone should approach an obstacle
        D_FULL = 10.0  # Minimum distance at which velocity maxes out
        STUCK_THRESHOLD = 0  # 0.01 # If v is below this value, move orthogonal to target direction
        DECAY = 3.0  # how quickly the repulsivity of obstacles decays with distance

        k_avoid = AVOID_RADIUS**DECAY  # Repulsivity of obstacles
        dist = math.sqrt(target[0]**2 + target[1]**2)
        if dist:
            v = [target[0]/dist, target[1]/dist]  # normalized gradient
        else:
            v = [0, 0]
        dmax = 0
        for ob in obstacles:
            r = math.sqrt(ob[0]**2 + ob[1]**2)
            if r == 0:
                continue
            d = k_avoid/r**DECAY
            if d > dmax:
                dmax = d
            v[0] -= d*ob[0]/r
            v[1] -= d*ob[1]/r
        if v[0]**2 + v[1]**2 == 0:
            return (0, 0)
        vbar = (dist+dmax*D_FULL)/math.sqrt(v[0]**2 + v[1]**2)
        if math.sqrt(v[0]**2 + v[1]**2) < STUCK_THRESHOLD and dist:
            v = [-target[1]/dist, target[0]/dist]
        return (v[0]*vbar, v[1]*vbar)


if __name__ == '__main__':
    m = Move(None)
    target = (2, 4)
    obstacles = [(1, 1)]
    print(m.get_move_direction(target, obstacles))

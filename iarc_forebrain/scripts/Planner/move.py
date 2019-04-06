#!/usr/bin/env python2

''' A simple one-time motion command '''
import rospy
import math
from mode import Mode
from Drone import Drone

class Move(Mode):
    def __init__(self, drone, angle=0, dz=0):
    	self.active = False
        self.angle = angle
        self.dz = dz
        self.drone = drone
        self.obstacles = []

    def enable(self, distance=0, units=0):
        self.distance = self.parse(distance, units)
        pos = self.drone.get_pos("odom").pose.position
        if self.dz == 0:
            dx = self.distance*math.cos(self.angle)
            dy = self.distance*math.sin(self.angle)
            self.target = (pos.x+dx, pos.y+dy, pos.z)
            print('MOVE: dx = %s, dy = %s' % (dx, dy))
        else:
            self.target = (pos.x, pos.y, pos.z+self.dz*self.distance)
            print('MOVE: dz = %s' % (self.dz*self.distance))
        self.active = True

    def update(self, look = False, obstacles = []):
        # TODO: make look code robust to non-odom tf frames
        pos = self.drone.get_pos("odom").pose.position
        v = self.get_move_direction([self.target[0]-pos.x, self.target[1]-pos.y], [(o[0]-pos.x, o[1]-pos.y) for o in obstacles])
        # TODO: account for vertical obstacles
        if look and look.header.frame_id == 'odom':
            x, y = look.point.x, look.point.y
            self.drone.travel_and_look(v[0], v[1], x, y, 'odom', self.target[2])
        else:
            self.drone.move_towards(v[0], v[1], 'odom', self.target[2])

    def get_move_direction(self, target=(0,0), obstacles=[]):
        '''
        Returns optimal (vx, vy) based on gradient of potential field determined by target and 
        obstacle coordinates relative to current drone location
        '''
        AVOID_RADIUS = 1 # Closest that drone should approach an obstacle
        D_FULL = 10 # Minimum distance at which velocity maxes out
        STUCK_THRESHOLD = 0.1 #0.01 # If v is below this value, move orthogonal to target direction
        DECAY = 2 # how quickly the repulsivity of obstacles decays with distance

        k_avoid = AVOID_RADIUS**DECAY # Repulsivity of obstacles
        dist = math.sqrt(target[0]**2 + target[1]**2)
        v = [target[0]/dist, target[1]/dist] # normalized gradient
        dmax = 0
        for ob in obstacles:
            r = math.sqrt(ob[0]**2 + ob[1]**2)
            d = k_avoid/r**DECAY
            if d > dmax:
                dmax = d
            v[0] -= d*ob[0]/r
            v[1] -= d*ob[1]/r
        vbar = (dist+dmax*D_FULL)/math.sqrt(v[0]**2 + v[1]**2)
        if math.sqrt(v[0]**2 + v[1]**2) < STUCK_THRESHOLD:
            v = [-target[1]/dist, target[0]/dist]
        return (v[0]*vbar, v[1]*vbar)

if __name__ == '__main__':
    m = Move(None)
    target = (2, 4)
    obstacles = [(1, 1)]
    print(m.get_move_direction(target, obstacles))

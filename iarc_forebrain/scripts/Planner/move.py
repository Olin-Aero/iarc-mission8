''' A simple one-time motion command '''
import rospy
import math
from mode import Mode
from Drone import Drone

class Move(Mode):
    def __init__(self, drone, angle=0, dz=0, look=False):
    	self.active = False
        self.angle = angle
        self.dz = dz
        self.drone = drone
        self.look = look

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
            print('MOVE: dz = %s' % self.dz*self.distance)
        self.active = True

    def update(self, look = False):
        if self.look and look and look.header.frame_id == 'odom':
            x, y = look.point.x, look.point.y
            self.drone.travel_and_look(self, self.target[0], self.target[1], x, y, 'odom', self.target[2]):
        elif self.distance == 0:
            self.drone.hover()
        else:
            self.drone.move_towards(self.target[0], self.target[1], 'odom', self.target[2])
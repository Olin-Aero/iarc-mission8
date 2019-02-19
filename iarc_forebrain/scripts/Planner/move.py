''' A simple one-time motion command '''
import rospy
import math
from mode import Mode
from Drone import Drone

class Move(Mode):
    def __init__(self, angle=0, dz=0):
    	self.active = False
        self.angle = angle
        self.dz = dz
        self.drone = Drone()

    def enable(self, distance=0, units=0):
        self.distance = self.parse(distance, units)
        self.active = True

    def update(self):
        if self.distance == 0:
            self.drone.hover()
            print('HOVER')
        elif self.dz == 0:
            dx = self.distance*math.cos(self.angle)
            dy = self.distance*math.sin(self.angle)

            self.drone.move_relative(dx, dy)
            print('MOVE: %s, %s' % (dx, dy))
        else:
            self.drone.move_relative(0, 0,rel_height=self.dz*self.distance)
        self.disable()
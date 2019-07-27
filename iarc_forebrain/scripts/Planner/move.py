''' A simple one-time motion command '''
import rospy
import math
from mode import Mode
from Drone import Drone
import tf

class Move(Mode):
    def __init__(self, drone, angle=0, dz=0):
    	self.active = False
        self.angle = angle
        self.dz = dz
        self.drone = drone
        # print drone.prev_orient

    def enable(self, distance=0, units=0):
        self.distance = self.parse(distance, units)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.drone.prev_orient)
        pos = self.drone.get_pos("odom").pose.position

        if (yaw is not 0.0):
            self.angle = yaw + self.angle
            print "ADJUSTED ANGLE: " + str(self.angle)

        if self.dz == 0:
            dx = self.distance*math.cos(self.angle)
            dy = self.distance*math.sin(self.angle)
            self.target = (pos.x+dx, pos.y+dy, pos.z)
            print('MOVE: %s, %s' % (dx, dy))
        else:
            self.target = (pos.x, pos.y, pos.z+self.dz*self.distance)
        self.active = True

    def update(self):
        if self.distance == 0:
            self.drone.hover()
        else:
            self.drone.move_towards(self.target[0], self.target[1], height=self.target[2])
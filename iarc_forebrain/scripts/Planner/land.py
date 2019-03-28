''' A high level behavior for the drone to carry out '''
import rospy
from mode import Mode
from Drone import Drone

class Land(Mode):
    def __init__(self, drone, takeoff=False):
    	self.active = False
    	self.takeoff = takeoff
        self.drone = drone

    def enable(self, look=False):
        if self.takeoff:
        	self.drone.takeoff()
        	print('TAKEOFF')
        else:
        	self.drone.land()
        	print('LAND')

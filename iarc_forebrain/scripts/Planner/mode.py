''' A high level behavior for the drone to carry out '''
import rospy
from Drone import Drone

class Mode:
    def __init__(self, drone):
    	''' Called once when program starts '''
    	self.active = False
        self.drone = drone
        self.player_pos = None

    def enable(self):
    	''' Called once each time mode becomes active '''
        self.active = True

    def disable(self):
    	''' Called once each time mode stops being active '''
        self.active = False

    def update(self, look=False):
    	''' Called iteratively while mode is active '''
        pass

    def is_active(self):
    	''' Called iteratively while mode is active '''
    	return hasattr(self, "active") and self.active

    def test(self):
    	''' Run the mode in current thread '''
    	self.enable()
    	rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def parse(self, value='0', units='meters'):
        value = float(value)
        if units == 'inches' or units == 'inch':
            value *= 0.0254
        if units == 'feet' or units == 'foot':
            value *= 0.3048
        if units == 'centimeters' or units == 'centimeter':
            value *= 0.01
        return value
''' A high level behavior for the drone to carry out '''
import rospy


class Mode(object):
    def __init__(self, drone):
        ''' Called once when program starts '''
        self.active = False
        self.yaw = 0
        self.drone = drone

    def enable(self, *args):
        ''' Called once each time mode becomes active with 
            additional parameters from the voice command '''
        self.active = True

    def disable(self):
        ''' Called once each time mode stops being active '''
        self.active = False

    def update(self, look_direction=0, obstacles=[]):
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
        ''' Applies distance unit conversions to meters '''
        value = float(value)
        if units == 'inches' or units == 'inch':
            value *= 0.0254
        if units == 'feet' or units == 'foot':
            value *= 0.3048
        if units == 'centimeters' or units == 'centimeter':
            value *= 0.01
        return value

    def get_look_direction(self, current_orientation, enable=False):
        ''' Returns drone orientation desired by mode as an angle in the odom frame (radians)
            The enable flag is true the first time this function is called'''
        self.yaw = current_orientation
        return current_orientation
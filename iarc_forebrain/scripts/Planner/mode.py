''' A high level behavior for the drone to carry out '''
import rospy

class Mode:
    def __init__(self):
    	''' Called once when program starts '''
    	self.active = False

    def enable(self):
    	''' Called once each time mode becomes active '''
        self.active = True

    def disable(self):
    	''' Called once each time mode stops being active '''
        self.active = False

    def update(self):
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
''' Show off your moves! Create a new "dance" mode for your drone to perform,
    using the following code as a template.

    To run your code in the simulator, run each of the following
    commands in its own window. You will need to restart all three
    commands each time you change and save your code.
        roslaunch iarc_main brain.launch
        rosrun iarc_main bebop_keyboard_multi.py
        roslaunch iarc_main n_drones_sim.launch n:=1

    Select the bebop_keyboard_multi window to move the drone around with
    key commands. Press h to enter autonomous mode when you want to run your
    code.

'''
import rospy
import math
from mode import Mode

class Dance(Mode):
    def __init__(self, drone):
        ''' Called once when program starts. '''
        Mode.__init__(self, drone)
	self.point_index = 0

    def enable(self, *args):
        ''' Called once each time this mode becomes active with
            additional parameters from the voice command
            contained in the args array (just ignore args for now). '''
        self.active = True

    def disable(self):
        ''' Called once each time this mode stops being active.
            Use this function to perform cleanup operations. '''
        self.active = False

    def update(self, look_direction=0, obstacles=[]):
        ''' Called repeatedly while this mode is active. 
            Use this function for repeated operations. 
            Ignore the other parameters for now. '''

	if self.point_index > 3:
		return

	goal = ([1,1,0,0][self.point_index], [0,1,1,0][self.point_index])

	pos = (self.get_position().x, self.get_position().y)

	dist_to_goal = math.sqrt(math.pow(goal[0] - pos[0], 2) + math.pow(goal[1] - pos[1], 2))
	if dist_to_goal < 0.1:
		self.point_index += 1

	self.move_towards(goal[0], goal[1])



    ### Helper functions ###

    def get_position(self):
        ''' Return a vector containing drone position in meters
            position.x is forward, position.y is left, and position.z is up. '''
        return self.drone.get_pos("map").pose.position

    def move_towards(self, x=0.0, y=0.0, z=None):
        ''' Move drone towards a new location. This function must be called repeatedly in update.
            If given height is None, the drone will maintain its current altitude. '''
        self.drone.move_towards(des_x=x, des_y=y, frame='map', height=z)

    def get_time(self):
        ''' Return the current time in seconds '''
        return rospy.get_time()

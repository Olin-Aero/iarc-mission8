''' Show off your moves! Create a new "dance" mode for your drone to perform,
    using the following code as a template.

    To run your code in the simulator, run each of the following
    commands in its own window. You will need to restart the brain.launch
    command each time you change the code.
        roslaunch iarc_main brain.launch
        rosrun iarc_main bebop_keyboard_multi.py n:=1
        roslaunch iarc_main n_drones_sim.launch n:=1

    Select the bebop_keyboard_multi window to move the drone around with
    key commands. Press h to enter autonomous mode when you want to run your
    code.

'''
import rospy
from mode import Mode

class Dance(Mode):
    def __init__(self, drone):
        ''' Called once when program starts. '''
        Mode.__init__(self, drone)

    def enable(self, *args):
        ''' Called once each time this mode becomes active with
            additional parameters from the voice command
            contained in the args array. '''
        self.active = True
        self.p0 = self.get_position()
        self.i = 0

    def disable(self):
        ''' Called once each time this mode stops being active.
            Use this function to perform cleanup operations. '''
        self.active = False

    def update(self, look_direction=0, obstacles=[]):
        ''' Called repeatedly while this mode is active. 
            Use this function for repeated operations. '''
        p = self.get_position()
        if self.i == 0:
            self.move_towards(self.p0.x+1, self.p0.y)
            if p.x-self.p0.x > .9:
                self.i+=1
                return

        if self.i == 1:
            self.move_towards(self.p0.x+1, self.p0.y+1)
            if p.y-self.p0.y > .9:
                self.i+=1
                return

        if self.i == 2:
            self.move_towards(self.p0.x, self.p0.y+1)
            if p.x-self.p0.x < .1:
                self.i+=1
                return

        if self.i == 3:
            self.move_towards(self.p0.x, self.p0.y)
            if p.y-self.p0.y < .1:
                self.i = 0
                return
        pass



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
''' A high level behavior for the drone to carry out '''
import rospy
from mode import Mode


class TakeoffLand(Mode):
    def __init__(self, drone, takeoff=False):
        Mode.__init__(self, drone)
        self.takeoff = takeoff

    def enable(self):
        if self.takeoff:
            self.drone.takeoff(tol=0)
            print('TAKEOFF')
        else:
            self.drone.land(block=False)
            print('LAND')

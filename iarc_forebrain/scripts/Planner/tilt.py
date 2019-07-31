import rospy
from mode import Mode


class Tilt(Mode):
    def __init__(self, drone):
        Mode.__init__(self, drone)

    def enable(self, angle, units=0):

        if angle == "up":
            self.drone.move_camera(-30, 0)
            print('TILT UP')
        elif angle == "down":
            self.drone.move_camera(-90, 0)
            print('TILT DOWN')
        else:
            self.drone.move_camera(-float(angle), 0)

#!/usr/bin/env python2
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty, String

import sys, select, termios, tty

""" Class for representing a bebop drone """

msgTeleop = """
--------------------------
Moving around (strafing):
   u    i    o
   j    k    l
   m    ,    .
--------------------------
Up, Down, Turning
    w
  a   d
    s
--------------------------
For Turning mode, hold down the shift key:
   U    I    O
   J    K    L
   M    <    >
--------------------------
Select drones: 0 for all, 1-4 for individual
takeoff: t
land: spacebar
Emergency stop (cut all motors): '=' or '+'
Switch to autonomous mode: 'h' or 'H'
anything else : stop

q/z : increase/decrease max speeds by 10%
r/v : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

takeoffLand = {
    't': (1, 0),
    ' ': (0, 1)
}

moveBindings = {
    'I': (1, 0, 0, 0),
    'O': (1, 0, 0, -1),
    'J': (0, 0, 0, 1),
    'L': (0, 0, 0, -1),
    'U': (1, 0, 0, 1),
    '<': (-1, 0, 0, 0),
    '>': (-1, 0, 0, 1),
    'M': (-1, 0, 0, -1),
    'o': (1, -1, 0, 0),
    'i': (1, 0, 0, 0),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (1, 1, 0, 0),
    ',': (-1, 0, 0, 0),
    '.': (-1, -1, 0, 0),
    'm': (-1, 1, 0, 0),
    'w': (0, 0, 1, 0),
    's': (0, 0, -1, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'r': (1.1, 1),
    'v': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class Drone(object):

    def __init__(self, namespace, bind):
        """ initializes the drone class
        namespace: The namespace associated with this drone
        bind: char that represents control key for this drone instance
        """
        self.flight_status = False #initializes in teleop
        self.namespace = str(namespace) #namespace for subs
        self.bind = bind #number bind assigned to controlling drone

        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.speed = rospy.get_param("~speed", 0.1)
        self.turn = rospy.get_param("~turn", 0.2)

        self.cmd_pub = rospy.Publisher('/' + self.namespace + '/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/' + self.namespace + '/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/' + self.namespace + '/land', Empty, queue_size=1)
        self.reset_pub = rospy.Publisher('/' + self.namespace + '/reset', Empty, queue_size=1)

        self.cmd_sub = rospy.Subscriber('/' + self.namespace + '/low_level/cmd_vel', Twist, self.callback_cmd_vel)
        self.takeoff_sub = rospy.Subscriber('/' + self.namespace +'/low_level/takeoff', Empty, self.callback_takeoff)
        self.land_sub = rospy.Subscriber('/' + self.namespace + '/low_level/land', Empty, self.callback_land)


    def callback_cmd_vel(self, msg):
        if self.flight_status == True:
            self.cmd_pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "velocity command ignored\r")

    def callback_takeoff(self, msg):
        if self.flight_status == True:
            self.takeoff_pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "takeoff command ignored\r")

    def callback_land(self, msg):
        if self.flight_status == True:
            self.land_pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "land command ignored\r")

    def get_vels(self):
        """
        returns the current linear and angular velocities
        """
        return "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

    def finish(self):
        """
        Does the commands to hover, then land the drone upon pressing CTRL-C
        """
        self.cmd_pub.publish(Twist())
        self.land_pub.publish()

    def handle_key(self, key):
        """
        Takes in the key and handles what to do with it
        If nothing is pressed, do nothing
        """
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.z = moveBindings[key][2]
            self.th = moveBindings[key][3]
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]
            self.turn = self.turn * speedBindings[key][1]

            print self.get_vels()
            if self.status == 14:
                print (msgTeleop)
            self.status = (self.status + 1) % 15
        else:
            self.x = 0
            self.y = 0
            self.z = 0
            self.th = 0

        self.cmd_pub.publish(Twist(
                linear=Vector3(x=self.x * self.speed, y=self.y * self.speed, z=self.z * self.speed),
                angular=Vector3(z=self.th * self.turn)
                ))

    def __str__(self):
        return "Drone controlling " + str(self.namespace)

    __repr__ = __str__

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    print msgTeleop
    rospy.init_node('estop')

    # init all drones used
    all_drones = [  Drone('alexa', '1'),
                    Drone('google', '2'),
                    Drone('siri', '3'),
                    Drone('clippy', '4'),
                    ]

    # start with all drones active
    active_drones = all_drones

    try:
        while not rospy.is_shutdown():
            key = getKey()

            # CTRL-C quits and lands drone
            if key == '\x03':
                break

            # change drone from teleop to autonomous
            if key == 'h' or key == "H":

                for d in active_drones:
                    d.flight_status = True

                rospy.loginfo("In autonomous mode!")
            else:
                # a teleop key is pressed, kick all all_drones
                # out of autonomous
                for d in all_drones:
                    d.flight_status = False

                # handles reset
                if key == '=' or key == '+':
                    for d in active_drones:
                        d.reset_pub.publish()

                # handle landing and taking off
                if key in takeoffLand.keys():
                    if takeoffLand[key][0] == 1:
                        for d in active_drones:
                            d.takeoff_pub.publish()
                    elif takeoffLand[key][1] == 1:
                        for d in active_drones:
                            d.land_pub.publish()

                # handles keys for all active drones
                if key == '0':
                    active_drones = all_drones
                    print active_drones
                else:
                    selected_drones = [drone for drone in all_drones if drone.bind == key]
                    if selected_drones:
                        active_drones = selected_drones
                        print active_drones
                    for d in active_drones:
                        d.handle_key(key)

    except Exception as e:
        print (e)

    # hovers and then lands drone if run loop is quit
    finally:
        for d in active_drones:
            d.finish()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

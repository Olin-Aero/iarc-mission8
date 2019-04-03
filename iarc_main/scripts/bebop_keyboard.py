#!/usr/bin/env python2
import roslib

roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty, String

import sys, select, termios, tty

msgTeleop = """
You are now running teleop!

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around (strafing):
   u    i    o
   j    k    l
   m    ,    .
--------------------------
Up, Down, Turning

    w
  a   d
    s

For Turning mode, hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

takeoff: t
land: spacebar
Cut all motors: '=' or '+'
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



def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def land():
    for p in land_pubs:
        p.publish()

######### Movement key handling
x = 0
y = 0
z = 0
th = 0
status = 0

speed = rospy.get_param("~speed", 0.1)
turn = rospy.get_param("~turn", 0.2)

def get_movement(key):
    global speed, turn, x, y, z, th, status
    if key in moveBindings.keys():
        x = moveBindings[key][0]
        y = moveBindings[key][1]
        z = moveBindings[key][2]
        th = moveBindings[key][3]
    elif key in speedBindings.keys():
        speed = speed * speedBindings[key][0]
        turn = turn * speedBindings[key][1]

        print vels(speed, turn)
        if status == 14:
            print msgTeleop
        status = (status + 1) % 15
    else:
        x = 0
        y = 0
        z = 0
        th = 0

    return Twist(
            linear=Vector3(x=x * speed, y=y * speed, z=z * speed),
            angular=Vector3(z=th * turn)
            )

#####################


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    print msgTeleop
    rospy.init_node('estop')

    pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
    takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    land_pubs = [rospy.Publisher('/bebop/land', Empty, queue_size=1),
                 rospy.Publisher('/ardrone/land', Empty, queue_size=1)]
    reset_pub = rospy.Publisher('/bebop/reset', Empty, queue_size=1)

    flightStatus = False #autonomous code is in control when true

    def callback_cmd_vel(msg):
        if flightStatus == True:
            pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "velocity command ignored\r")

    def callback_takeoff(msg):
        if flightStatus == True:
            takeoff_pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "takeoff command ignored\r")

    def callback_land(msg):
        if flightStatus == True:
            land()
        else:
            rospy.loginfo_throttle(1, "land command ignored\r")

    takeoff_sub = rospy.Subscriber('/bebop/low_level/takeoff', Empty, callback_takeoff)
    land_sub = rospy.Subscriber('/bebop/low_level/land', Empty, callback_land)
    command_sub = rospy.Subscriber('/bebop/low_level/cmd_vel', Twist, callback_cmd_vel)

    try:
        while True: #while no issues
            key = getKey()

            if key == '\x03':
                break

            if key == 'h' or key == 'H':
                flightStatus = True
                rospy.loginfo("In autonomous mode!")
            else:
                flightStatus = False

                # Handle reset
                if key == '=' or key == '+':
                    reset_pub.publish()

                # Handle takeoff and landing
                if key in takeoffLand.keys():
                    if takeoffLand[key][0] == 1:
                        takeoff_pub.publish()
                    elif takeoffLand[key][1] == 1:
                        land()

                pub.publish(get_movement(key))

    except Exception as e:
        print e

    finally:
        pub.publish(Twist())
        land()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#!/usr/bin/env python2

import rospy
import numpy as np
import math
import cv2
import sys
from std_msgs.msg import String
from mode import Mode
from follow_gesture import FollowGesture


class Planner:

    def __init__(self, drone):
        self.drone = drone
        rospy.init_node(str(drone)+'_planner')
        self.modes = {"idle": Mode(),"follow_gesture": FollowGesture()}
        self.pub = rospy.Publisher("/"+drone+"_current_mode", String, queue_size=10)
        self.current_mode = self.modes["idle"]
        rospy.Subscriber("/whistle", String, self.whistle_callback)

    def whistle_callback(self, msg):
        ''' Whistle command format: [drone] [behavior] [parameters...] '''
        args = msg.data.split(" ")
        if args[0] != self.drone:
            return
        if args[1] in self.modes:
            if self.current_mode.is_active():
                self.current_mode.disable()
            self.current_mode = self.modes[args[1]]
            self.current_mode.enable(*args[2:])
            print(args[1])
            self.pub.publish(args[1])
        else:
            print("Invalid mode requested: %s" % msg.data)

    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            self.current_mode.update() # is this thread safe?
            rate.sleep()

# Start the node
if __name__ == '__main__':
    p = Planner('alpha')
    msg = String()
    msg.data = "alpha follow_gesture 10"
    p.whistle_callback(msg)
    p.run()
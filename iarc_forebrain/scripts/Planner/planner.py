#!/usr/bin/env python2

import rospy
import numpy as np
import math
import cv2
import sys
from std_msgs.msg import String
from mode import Mode
from follow_gesture import FollowGesture
from land import Land
from move import Move


class Planner:

    def __init__(self, drone='red'):
        self.drone = drone
        rospy.init_node(str(drone)+'_planner')
        self.modes = {"idle": Mode(),"follow": FollowGesture(),
                        "land": Land(),"takeoff": Land(takeoff=True),\
                        "north": Move(0),"east": Move(3*math.pi/2),\
                        "south": Move(math.pi), "west": Move(math.pi/2), \
                        "stop": Move(0), "duck": Move(0,-1),\
                        "jump": Move(0,1)}
        self.pub = rospy.Publisher("/"+drone+"_current_mode", String, queue_size=10)
        self.current_mode = self.modes["idle"]
        rospy.Subscriber("/voice", String, self.voice_callback)

    def voice_callback(self, msg):
        ''' Voice command format: [drone] [command] [parameters...] '''
        args = msg.data.split(" ")
        if args[0] != self.drone:
            return
        if args[1] in self.modes:
            if self.current_mode.is_active():
                self.current_mode.disable()
            self.current_mode = self.modes[args[1]]
            try:
                self.current_mode.enable(*args[2:])
            except:
                print("Invalid parameters provided: %s" % msg.data)
                return
            print(args[1])
            self.pub.publish(args[1])
        else:
            print("Invalid mode requested: %s" % msg.data)

    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if self.current_mode.is_active():
                self.current_mode.update() # is this thread safe?
            rate.sleep()

# Start the node
if __name__ == '__main__':
    p = Planner('red')
    msg = String()
    msg.data = "red follow 10"
    # p.voice_callback(msg)
    p.run()
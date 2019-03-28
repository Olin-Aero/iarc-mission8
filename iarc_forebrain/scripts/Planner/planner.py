#!/usr/bin/env python2

import rospy
import numpy as np
import math
import cv2
import sys
from std_msgs.msg import String
from mode import Mode
from follow_gesture import FollowGesture
from geometry_msgs.msg import PointStamped
from land import Land
from move import Move
from Drone import Drone


class Planner:

    def __init__(self, color='red'):
        self.color = color
        print(color+'_planner')
        rospy.init_node(color+'_planner')
        self.drone = Drone()
        drone = self.drone
        self.modes = {"idle": Mode(drone),"follow": FollowGesture(drone),
                        "land": Land(drone),"takeoff": Land(drone, takeoff=True),\
                        "north": Move(drone, 0),"east": Move(drone, 3*math.pi/2),\
                        "south": Move(drone, math.pi), "west": Move(drone, math.pi/2), \
                        "stop": Move(drone,0), "duck": Move(drone, 0,-1),\
                        "jump": Move(drone,0,1)}
        self.pub = rospy.Publisher("/"+color+"_current_mode", String, queue_size=10)
        self.current_mode = self.modes["idle"]
        self.player_pos = False
        self.look_mode = FollowGesture(drone, False)
        self.look_mode.enable()
        rospy.Subscriber("/voice", String, self.voice_callback)
        rospy.Subscriber("/player", PointStamped, self.player_callback)

    def voice_callback(self, msg):
        ''' Voice command format: [color] [command] [parameters...] '''
        args = msg.data.split(" ")
        if args[0] != self.color:
            return
        if args[1] in self.modes:
            if self.current_mode.is_active():
                self.current_mode.disable()
            self.current_mode = self.modes[args[1]]
            try:
                self.current_mode.player_pos = self.player_pos
                self.current_mode.enable(*args[2:])
            except TypeError as e:
                print("Invalid parameters provided: %s" % msg.data)
                return
            except Exception as e:
                print(e)
                return
            print(args[1])
            self.pub.publish(args[1])
        else:
            print("Invalid mode requested: %s" % msg.data)

    def player_callback(self, msg):
        self.player_pos = msg.data

    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if self.current_mode.is_active():
                self.current_mode.update(self.player_pos) # is this thread safe?
            rate.sleep()

# Start the node
if __name__ == '__main__':
    p = Planner('red')
    msg = String()
    msg.data = "red follow 10"
    # p.voice_callback(msg)
    p.run()
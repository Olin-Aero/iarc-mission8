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
from takeoff_land import TakeoffLand
from move import Move
from iarc_arbiter.drone import Drone


class Planner(object):

    def __init__(self, colors):
        rospy.init_node('planner')
        self.colors = colors
        self.drones = {}
        for c in self.colors:
            self.drones[c] = SubPlanner(c)
        self.player_pos = False
        self.obstacles = []
        rospy.Subscriber("/voice", String, self.voice_callback)
        rospy.Subscriber("/helmet_pos", PointStamped, self.player_callback)
        # TODO: change message type
        rospy.Subscriber("/obstacles", PointStamped, self.obstacle_callback)

    def voice_callback(self, msg):
        ''' Voice command format: [color] [command] [parameters...] '''
        args = msg.data.split(" ")
        if args[0] == 'swarm':
            for drone in self.drones.values():
                self.command_drone(drone, args)
            return
        if not args[0] in self.colors:
            rospy.loginfo("Drone not recognized: %s" % args[0])
            return
        drone = self.drones[args[0]]
        self.command_drone(drone, args)

    def command_drone(self, drone, args):
        if args[1] in drone.modes:
            if drone.current_mode.is_active():
                drone.current_mode.disable()
            drone.current_mode = drone.modes[args[1]]
            try:
                drone.current_mode.enable(*args[2:])
            except TypeError as e:
                rospy.loginfo("Invalid parameters provided: %s" % args)
                return
            except Exception as e:
                rospy.loginfo(e)
                return
            print(args[1])
            drone.current_mode_pub.publish(args[1])
        else:
            rospy.loginfo("Invalid mode requested: %s" % args)

    def player_callback(self, msg):
        self.player_pos = msg

    def obstacle_callback(self, msg):
        self.obstacles = msg
        # TODO: parse obstacle message into proper format

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            for drone in self.drones.values():
                if drone.current_mode.is_active():
                    drone.current_mode.update(
                        self.player_pos, self.obstacles)  # is this thread safe?
                    drone.look_mode.update(self.player_pos, self.obstacles)
            rate.sleep()


class SubPlanner:

    def __init__(self, color):
        self.drone = Drone('/'+color)
        self.color = color
        drone = self.drone
        self.modes = {"idle": Mode(drone),            "follow": FollowGesture(drone),
                      "land": TakeoffLand(drone),     "takeoff": TakeoffLand(drone, takeoff=True),
                      "north": Move(drone, 0),        "east": Move(drone, 3*math.pi/2),
                      "south": Move(drone, math.pi),  "west": Move(drone, math.pi/2),
                      "stop": Move(drone, 0),         "duck": Move(drone, 0, -1),
                      "jump": Move(drone, 0, 1)}
        self.current_mode_pub = rospy.Publisher(
            "/"+color+"_current_mode", String, queue_size=10)
        self.current_mode = self.modes["idle"]
        self.look_mode = FollowGesture(drone, False)
        self.look_mode.enable()
        print('Drone ' + color + ' initialized')


# Start the node
if __name__ == '__main__':
    p = Planner(['alexa', 'google'])
    p.run()
#!/usr/bin/env python2

import rospy
import numpy as np
import math
import cv2
import sys
from collections import defaultdict

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PointStamped, Point, Vector3
from tf.transformations import *
from visualization_msgs.msg import Marker, MarkerArray

from mode import Mode
from follow_gesture import FollowGesture
from takeoff_land import TakeoffLand
from move import Move
from photo import Photo
from turn import Turn
from iarc_arbiter.drone import Drone


class Planner(object):

    def __init__(self, colors):
        rospy.init_node('planner')
        self.colors = colors
        self.drones = {}
        for c in self.colors:
            self.drones[c] = SubPlanner(c)
        self.player_pos = False
        # Map of drone names to lists of obstacles
        self.obstacles = defaultdict(list)
        rospy.Subscriber("/voice", String, self.voice_callback)
        rospy.Subscriber("/helmet_pos", PointStamped, self.player_callback)
        # TODO: change message type
        rospy.Subscriber("/obstacles", PointStamped, self.obstacle_callback)
        rospy.Subscriber("/rangefinder", Int32MultiArray,
                         self.rangefinder_callback)

        self.obstacle_vis_pub = rospy.Publisher('obstacle_visualizations', Marker, queue_size=10)

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
                drone.current_mode.yaw = drone.look_direction
                drone.current_mode.enable(*args[2:])
            except TypeError as e:
                rospy.loginfo("Invalid parameters provided: %s, %s" % args, e)
                return
            except Exception as e:
                rospy.logwarn(e)
                return
            print(args[1])
            drone.current_mode_pub.publish(args[1])
        elif args[1] in drone.look_modes:
            if drone.look_mode.is_active():
                drone.look_mode.disable()
            drone.look_mode = drone.look_modes[args[1]]
            try:
                drone.look_mode.enable(*args[2:])
                drone.look_direction = drone.look_mode.get_look_direction(
                    drone.look_direction, True)
            except TypeError as e:
                rospy.loginfo("Invalid parameters provided: %s, %s" % args, e)
                return
            except Exception as e:
                rospy.logwarn(e)
                return
            print(args[1])
            drone.look_mode_pub.publish(args[1])
        else:
            rospy.loginfo("Invalid mode requested: %s" % args)

    def player_callback(self, msg):
        self.player_pos = msg

    def obstacle_callback(self, msg):
        pass
        # TODO: parse obstacle message into proper format

    def rangefinder_callback(self, msg):
        OBJECT_DESPAWN_TIME = 2.0 # Despawn all old obstacles immediately
        drones = ["alexa", "google", "siri", "clippy"]
        # TODO: actual mounting angles in degrees CCW from forward
        angles = [60, 0, -60]
        maxrange = 2.0  # maximum distance for finding objects with rangefinder in meters
        beamwidth = 60  # angular coverage of each ultrasonic in degrees
        US_PER_METER = 5787.0
        vals = msg.data
        name=drones[vals[0]]
        pose = self.drones[name].drone.get_pos("map").pose
        orient = pose.orientation
        p = pose.position
        yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])[2]
        self.obstacles[name] = []
        obstacles = self.obstacles[name]
        # for o in obstacles:
        #     obstacles.remove(0)
        #     continue
        #     # Clear out old obscacles
        #     ang = np.arctan2(o[1]-p.y, o[0]-p.x)
        #     dist = np.sqrt((o[0]-p.x)**2+(o[1]-p.y)**2)
        #     # if dist < maxrange and any(abs(a+yaw-ang) < np.radians(beamwidth)/2 for a in angles):
        #     #     obstacles.remove(o)
        #     # obstacles removed after timeout
        #     if rospy.get_time() - o[3] > OBJECT_DESPAWN_TIME:
        #         obstacles.remove(o)
        for i, val in enumerate(vals[1:]):
            if val > 0:
                dist = val/US_PER_METER
                ang = np.radians(angles[i])+yaw
                obstacles += [(dist*np.cos(ang)+p.x,
                               dist*np.sin(ang)+p.y, p.z, rospy.get_time())]

        # Output visualizations
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.SPHERE_LIST
        m.scale = Vector3(0.5, 0.5, 0.5)
        m.ns = 'planner'
        m.id = vals[0]
        m.color.a = 1.0
        m.color.r = vals[0]/4.0
        m.color.g = 1-vals[0]/4.0
        for x,y,z,t in obstacles:
            m.points.append(Point(x,y,z))
        self.obstacle_vis_pub.publish(m)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            for name, drone in self.drones.iteritems():
                if drone.look_mode.is_active():
                    drone.look_direction = drone.look_mode.get_look_direction(
                        drone.look_direction)
                    drone.look_mode.update(
                        drone.look_direction, self.obstacles[name])
                if drone.current_mode.is_active():
                    drone.current_mode.yaw = drone.look_direction
                    drone.current_mode.update(
                        drone.look_direction, self.obstacles[name])  # is this thread safe?
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
                      "stop": Move(drone, 0),         "forward": Move(drone, 0, relative=True),
                      "duck": Move(drone, 0, -1),     "jump": Move(drone, 0, 1),
                      "analyze": Photo(drone)}
        self.look_modes = {"look": Turn(drone),
                           "right": Turn(drone, -1),
                           "left": Turn(drone, 1)}
        self.look_direction = 0
        self.current_mode_pub = rospy.Publisher(
            "/"+color+"_current_mode", String, queue_size=10)
        self.look_mode_pub = rospy.Publisher(
            "/"+color+"_look_mode", String, queue_size=10)
        self.current_mode = self.modes["idle"]
        self.look_mode = self.modes["idle"]
        print('Drone ' + color + ' initialized')


# Start the node
if __name__ == '__main__':
    p = Planner(['siri'])
    p.run()

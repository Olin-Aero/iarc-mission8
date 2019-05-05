#!/usr/bin/env python2

from __future__ import print_function

import rospy

from tf import TransformListener, TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged


class LaunchObserver(object):
    """
    Keeps track of the flying/landed state of a single drone, and publishes 
    a tf message keeping track of the coordinates from which the drone most recently launched.
    """

    def __init__(self):
        self.tfl = TransformListener()
        self.tfb = TransformBroadcaster()
        self.flying_state_sub = rospy.Subscriber(
            'states/ardrone3/PilotingState/FlyingStateChanged', Ardrone3PilotingStateFlyingStateChanged, self.on_flying_state)

        self.is_flying = True

        self.RATE = 5  # republish hz

        self.saved_translation = [0, 0, 0]  # In meters
        self.saved_yaw = 0  # In radians

        self.name = rospy.get_namespace().strip('/')
        self.base_link = self.name + '/base_link'
        self.launch = self.name + '/launch'
        self.odom = self.name + '/odom'

    def on_flying_state(self, msg):
        self.is_flying = msg.state in [Ardrone3PilotingStateFlyingStateChanged.state_takingoff,
                                       Ardrone3PilotingStateFlyingStateChanged.state_hovering,
                                       Ardrone3PilotingStateFlyingStateChanged.state_flying,
                                       Ardrone3PilotingStateFlyingStateChanged.state_landing,
                                       Ardrone3PilotingStateFlyingStateChanged.state_emergency_landing]

    def spin(self):
        r = rospy.Rate(self.RATE)
        self.tfl.waitForTransform(self.odom, self.base_link, rospy.Time(
            0), rospy.Duration.from_sec(99999))
        while not rospy.is_shutdown():
            if not self.is_flying:
                # On the ground, update the transform
                pos, quat = self.tfl.lookupTransform(
                    self.base_link, self.odom, rospy.Time(0))

                pos[2] = 0
                self.saved_translation = pos
                _, _, self.saved_yaw = euler_from_quaternion(quat)

            time = max(rospy.Time.now(), self.tfl.getLatestCommonTime(
                self.odom, self.base_link)) + (2*rospy.Duration.from_sec(1.0/self.RATE))

            self.tfb.sendTransform(self.saved_translation,
                                   quaternion_from_euler(0, 0, self.saved_yaw),
                                   time,
                                   self.odom, self.launch)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('launch_observer')
    LaunchObserver().spin()

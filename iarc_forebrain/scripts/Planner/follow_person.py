#!/usr/bin/env python2

import sys
import math
import cv2
import numpy as np

import rospy
from tf.transformations import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float64

from pointing_detection import pointing_detection
from mode import Mode
from move import Move
from iarc_arbiter.drone import Drone
from iarc_msgs.msg import Identifier
from iarc_msgs.srv import TrackRequest, TrackResponse


class FollowPerson(Mode):
    """
    Follow person.
    """
    def __init__(self, drone, source):
        self.drone_ = drone
        self.source_ = source

        # TODO(yycho0108): automatically figure out source
        # self.source_ = drone.namespace[1:-1] ??
        # the issue here is using namespace string is unstable.

        self.target_ = rospy.get_param('~target', 'tracked_objects')
        self.timeout_ = float(rospy.get_param('~timeout', 1.0))

        self.track_srv_ = rospy.ServiceProxy('/track', Track)
        self.target_sub_ = rospy.Subscriber(
            self.target_, IARCObjects, self.track_cb)

        # data
        self.target_pos_ = None
        self.target_stamp_ = None

    def track_cb(self, msg):
        # TODO(yycho0108): track_cb() will be robustified
        # after the refactor of tracker_node.py to handle namespaces
        if not self.is_active():
            return

        for obj in msg.objects:
            if obj.source is not self.source_:
                continue
            if (rospy.Time.now() - obj.stamp).to_sec() > self.timeout_:
                continue

            # TODO(yycho0108): depends on centroid being semi-correct-ish
            # ideally will not kill the operator ...
            self.target_pos_ = obj.centroid
            self.target_stamp_ = obj.stamp
            break

    def has_target(self):
        # checks if target data exists and stamp is not stale
        try:
            c1 = (self.target_pos_ is not None)
            c2 = (self.target_stamp_ is not None)
            if not c1 and c2:
                return False
            dt = (rospy.Time.now() - self.target_stamp_).to_sec()
            return (dt < self.timeout_)
        except Exception as e:
            return False

    def compute_target_direction(self):
        if self.target_pos_ is None:
            return None
        # sanity check - transform target w.r.t. drone
        # delegate tf transforms to the Drone() class
        # NOTE(yycho0108): check target_pos_ stamp is populated correctly.
        target_point = self.drone.tf.transformPoint(
                self.drone.FRAME_ID, # why is this all caps?
                self.target_pos_)

        # TODO(yycho0108): it will be GREAT is drone also had camera_frame_
        # as a field. It will make pitch/yaw more accurate.

        # look_direction will simply compute the best camera gimbal position
        # that will center the target in the field of view in the current
        # position - assuming that current position will ultimately converge to
        # the desired positional offset from the target, look_direction will
        # also converge to the optimal desired pitch angle.

        pitch = np.arcsin(target_point.z / target_point.x)
        yaw   = np.arctan(target_point.y, target_point.x)
        look_direction = np.float32([
                np.cos(pitch) * np.cos(yaw),
                np.cos(pitch) * np.sin(yaw),
                np.sin(pitch)
                ])
        return look_direction

    def reset(self):
        self.target_pos_ = None
        self.target_stamp_ = None

    def enable(self, *args):
        self.reset()

        # TODO(yycho0108): this would be a good place to lock on to the target
        # and run tracking + data association, ideally.
        # self.target_id_ = self.get_target()
        # abort_if_target_None()
        # see TODO under iarc_fuses/track/track_manager.py:TrackData
        # about the required updates.


        # TODO(yycho0108): ideally, track_srv_ should return:
        # 1) track_id_
        # 2) track_pos_
        # 2) track_success_
        self.track_srv_(TrackRequest(
            source=self.source_,
            stamp=rospy.Time(0),
            thresh=0.5,
            timeout=self.timeout_,
            obj=Identifier.OBJ_PERSON
            ))

        # NOTE(yycho0108): alternatively could set
        # retry_count_ = 0 and retry tracking initiation
        # pseudocode:
        # while(retry_count_++ < max_retry_):
        #   self.call_tracker_srv()
        self.active = True

    def disable(self, *args):
        # TODO(yycho0108): return_pitch_to_default()
        pass

    def update(self, look_direction=0, obstacles=[]):
        if self.has_target():
            look_direction = self.compute_target_direction()
            if look_direction is None:
                return
            self.move.update(look_direction, obstacles)
        else:
            # TODO(yycho0108): is_lost() logic here
            # if lost, abort() == self.active=False?


def main()
    # Start the node
    f = FollowPerson(Drone())
    f.test()

if __name__ == '__main__':
    main()

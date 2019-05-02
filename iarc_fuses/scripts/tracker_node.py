#!/usr/bin/env python2
"""
Object Tracker Node.

Sample Usage:
    rosrun iarc_fuses tracker_node.py _srcs:=['cv_camera'] _use_gpu:=True _dbg:=False
    roslaunch iarc_fuses tracker_node.py config
"""
import numpy as np
import cv2
import os
import rospy
import time
from collections import deque

import rospkg
import actionlib
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
#from iarc_msgs.srv import Detect, DetectRequest, DetectResponse
#from iarc_msgs.srv import Track, TrackRequest, TrackResponse
from iarc_msgs.msg import *
from iarc_msgs.srv import *
from iarc_fuses.object_detection_tf import ObjectDetectorTF
from iarc_fuses.object_track import Object_Tracker
from iarc_fuses.track_manager import TrackManager, TrackData
from iarc_fuses.utils import draw_bbox, iarc_root, BoxUtils, box_iou
from iarc_fuses.camera_handle import CameraHandle

class Guess3D(object):
    @staticmethod
    def get_info(obj_id, box):
        pass

class TrackerNode(object):
    def __init__(self):
        # parse sources and parameters
        self.srcs_    = rospy.get_param('~srcs', [])
        self.dbg_     = rospy.get_param('~dbg', False)
        self.gpu_     = rospy.get_param('~gpu', 0.4)
        self.root_    = rospy.get_param('~root', os.path.join(iarc_root(), 'data'))
        self.tmodel_  = rospy.get_param('~tmodel', '') # tracker model; not used right now
        self.dmodel_  = rospy.get_param('~dmodel', 'person')
        self.max_dt_  = rospy.get_param('~max_dt', 0.5) # max threshold for stale data
        self.min_p_   = rospy.get_param('~min_p',  0.2) # minimum score to keep tracks

        # human-friendly default model specifications
        # currently only supports detector models, since there's only one tracker model (DaSiamRPN)
        self.mmap_ = {
                'person' : 'ssd_mobilenet_v1_ppn_shared_box_predictor_300x300_coco14_sync_2018_07_03',
                'drone'  : 'model2-drone-300x300'
                }
        # rectify model names based on model map
        if self.tmodel_ in self.mmap_:
            self.tmodel_ = self.mmap_[self.tmodel_]
        if self.dmodel_ in self.mmap_:
            self.dmodel_ = self.mmap_[self.dmodel_]

        # logging
        rospy.loginfo("""
        Tracker Configuration :
            Sources : {}
            Debug   : {}
            GPU     : {}
            Root    : {}
            DModel  : {}
            TModel  : {}
        """.format(self.srcs_,self.dbg_,self.gpu_,self.root_,self.dmodel_,self.tmodel_))

        # Processing Handles
        # person config
        # alternatively, self.det_ = rospy.ServiceProxy(...)
        # TODO : add more detectors
        self.det_ = ObjectDetectorTF(
                root=self.root_,
                model=self.dmodel_,
                cmap={1:Identifier.OBJ_PERSON},
                gpu=0.0#self.gpu_
                )
        # drone config
        #self.det_ = ObjectDetectorTF(
        #        root=os.path.join(iarc_root(), 'data'),
        #        model='model3-drone-640x640',
        #        cmap={1:DetectRequest.CID_DRONE},
        #        use_gpu=self.use_gpu_)

        self.trk_ = Object_Tracker(use_gpu=self.gpu_)
        self.mgr_ = TrackManager(self.trk_)

        # ROS Handles
        self.cvbr_ = CvBridge()

        self.srv_ = rospy.Service('track', Track, self.track_cb)
        #self.srv_ = actionlib.SimpleActionServer('track',
        #        TrackAction, execute_cb=self.track_cb,
        #        auto_start=False)
        self.trk_pub_ = rospy.Publisher('tracked_objects',
                IARCObjects, queue_size=10)

        # Register Camera Handlers
        self.cam_ = {k : CameraHandle(k, self.cvbr_, self.data_cb) for k in self.srcs_}

        # data
        self.queue_ = {k:deque(maxlen=8) for k in self.srcs_}
        self.q_idx_ = 0
        self.reqs_  = []
        self.tdata_ = []

        # Things to detect:
        # - Person ( <= 1 )
        # - Friendly Drone ( <= 4 )
        # - Enemy Drone ( <= ? )
        # - Bin/QR ( <= 4 )

    @staticmethod
    def match_class(cls_a, cls_b):
        agn_a = (cls_a is Identifier.OBJ_NULL)
        agn_b = (cls_b is Identifier.OBJ_NULL)
        eq_ab = (cls_a == cls_b)
        return (agn_a or agn_b or eq_ab)

    def match_detection(self, req, din):
        cid_req = req.obj.obj_id.obj_id
        cid, box, score = din
        if not TrackerNode.match_class(cid_req, cid):
            # class mismatch
            return False
        if score < req.thresh: # TODO : support classwise detection confidence threshold?
            # insufficient confidence
            return False
        return True

    def track_cb(self, req):
        """
        Tracking Callback; Only handles registration.
        Actual processing happens inside step()
        """
        src      = req.source
        tracking = True
        if req.stamp.to_sec() == 0:
            req.stamp = rospy.Time.now()
        self.reqs_.append( req )
        return TrackResponse(success=True)

    def data_cb(self, src, img, stamp):
        self.queue_[src].append( (src, img, stamp) )

    def filter_reqs(self, stamp, reqs):
        res = []
        for r in reqs:
            if (stamp - r.stamp).to_sec() > r.timeout:
                # current observation is now older than request timeout
                continue
            res.append( r)
        return res

    def process(self, src, img, stamp):
        # source validation
        cam = self.cam_[src]
        if not cam.has_info_:
            # should usually not reach here, maybe the first few frames
            return

        obs_new = []

        # remove stale requests
        self.reqs_ = self.filter_reqs(stamp, self.reqs_)

        #if len(self.reqs_) > 0:
        #    print len(self.reqs_)

        req_msk = np.full(len(self.reqs_), True, dtype=np.bool)

        if len(self.reqs_) > 0:
            # TODO : looking out into the future with multiple parallel detectors ...
            #d_res = []
            #for d_cid, det in self.det_:
            #    if not np.any(d_cid in self.reqs_):
            #        continue
            #    d_res.append( det.detect(img) )

            d_res   = self.det_.detect(img,
                    is_bgr = True
                    )
            for d in zip(d_res['class'], d_res['box'], d_res['score']):
                d_add = False # flag to add detection to observations
                for i_r, r in enumerate(self.reqs_):
                    if not req_msk[i_r]:
                        # already matched request
                        continue
                    if self.match_detection(r, d):
                        req_msk[i_r] = False
                        d_add = True
                        break

                if not d_add:
                    continue

                obs_new.append(TrackData(
                    src=src,
                    cid=d[0],
                    img=img,
                    box=BoxUtils.convert(d[1],
                        BoxUtils.FMT_NYXYX,
                        BoxUtils.FMT_NCCWH),
                    stamp=stamp.to_sec(),
                    meta=None # << will be populated by TrackManager() if the observation is accepted
                    ))

        self.reqs_ = [r for (r,m) in zip(self.reqs_, req_msk) if m]

        # append!
        self.mgr_.append( obs_new )
        self.mgr_.process(src, img, stamp.to_sec())

        # TODO : publish >> multiple << tracks simultaneously
        for t in self.mgr_.get_tracks():
            if (t.src_ != src):
                continue
            draw_bbox(img, t.box_,
                    fmt=BoxUtils.FMT_NCCWH,
                    cls=None)

            iw, ih = cam.model_.width, cam.model_.height
            cx, cy = np.multiply(t.box_[:2], [iw,ih])
            ray3d  = cam.model_.projectPixelTo3dRay([cx, cy])

            #self.pub_.publish(PointStamped(
            #    header=Header(frame_id=cam.model_.tfFrame(), stamp=stamp),
            #    point=Point(*ray3d)
            #    ))

        if self.dbg_:
            cv2.imshow('win', img) 
            cv2.waitKey(1)

        # pixel --> ray
        # x_rel, y_rel = loc2d
        # x, y = x_rel * cam.model_.width, y_rel * cam.model_.height
        # ray3d = cam.model_.projectPixelTo3dRay([x,y])

    def publish(self):
        msg = IARCObjects()
        msg.header.frame_id = 'map' # ??
        msg.header.stamp    = rospy.Time.now()

        for t in self.mgr_.get_tracks():
            # TODO : fill geometric information
            #res_3d = Guess3D.get_info(obj_id, box)
            #cov, vol, dmin, dmax, centroid = res_3d
            msg.objects.append(
                    IARCObject(
                        source = t.src_,
                        stamp  = rospy.Time(secs=t.stamp_),
                        obj_id = Identifier(obj_id=t.cid_),
                        box    = Box( # DaSiamRPN encoding
                            format=Box.FMT_CCWH,
                            data=t.box_,
                            normalized=True
                            ),
                        #covariance=
                        #volume=
                        #distance_range=
                        #centroid=
                        )
                    )
        self.trk_pub_.publish(msg)

    def step(self):
        # loop through sources sequentially
        src = self.srcs_[ self.q_idx_ ]
        q = self.queue_[ src ]
        self.q_idx_ = ((self.q_idx_ + 1) % len(self.srcs_))

        if len(q) <= 0:
            # no data in queue
            return

        data = q.pop()
        src, img, stamp = data
        dt = (rospy.Time.now() - stamp).to_sec()
        if (dt > self.max_dt_):
            # stale data
            rospy.loginfo_throttle(1.0,
                    'incoming data too old: [{}]-{}'.format(src, stamp))
            return
        self.process(src, img, stamp)
        self.publish()

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.step()
            r.sleep()

def main():
    rospy.init_node('tracker')
    node = TrackerNode()
    node.run()

if __name__ == "__main__":
    main()

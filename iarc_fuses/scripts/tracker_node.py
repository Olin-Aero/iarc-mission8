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

import rospkg
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
from iarc_msgs.srv import Detect, DetectRequest, DetectResponse
from iarc_msgs.srv import Track, TrackRequest, TrackResponse
from iarc_fuses.object_detection_tf import ObjectDetectorTF
from iarc_fuses.object_track import Object_Tracker
from iarc_fuses.utils import draw_bbox, iarc_root, BoxUtils
from iarc_fuses.camera_handle import CameraHandle

class TrackData(object):
    def __init__(self, src, cid, img, box, stamp, meta=None):
        self.src_ = src
        self.cid_ = cid
        self.img_ = img
        self.box_ = box
        self.stamp_ = stamp
        self.meta_ = meta

        self.cnt_ = 1 # count the number of frames seen
    def __repr__(self):
        return '[{}]{}-({})'.format(self.src_, self.cid_, self.box_)

class TrackerNode(object):
    def __init__(self):
        # parse sources and parameters
        srcs          = rospy.get_param('~srcs', [])
        self.dbg_     = rospy.get_param('~dbg', False)
        self.gpu_ = rospy.get_param('~gpu', 0.4)
        self.root_    = rospy.get_param('~root', os.path.join(iarc_root(), 'data'))
        self.tmodel_  = rospy.get_param('~tmodel', '') # tracker model; not used right now

        # human-friendly default model specifications
        # currently only supports detector models, since there's only one tracker model (DaSiamRPN)
        self.mmap_ = {
                'person' : 'ssd_mobilenet_v1_ppn_shared_box_predictor_300x300_coco14_sync_2018_07_03',
                'drone'  : 'model2-drone-300x300'
                }
        # rectify model names based on model map
        if self.tmodel_ in self.mmap_:
            self.tmodel_ = self.mmap_[self.tmodel_]

        # logging
        rospy.loginfo("""
        Tracker Configuration :
            Sources : {}
            Debug   : {}
            GPU     : {}
            Root    : {}
            DModel  : {}
            TModel  : {}
        """.format(srcs,self.dbg_,self.gpu_,self.root_,self.tmodel_))

        # Processing Handles
        # person config
        self.det_ = ObjectDetectorTF(
                root=self.root_,
                model=self.dmodel_,
                cmap={1:DetectRequest.CID_PERSON},
                gpu=self.gpu_
                )
        # drone config
        #self.det_ = ObjectDetectorTF(
        #        root=os.path.join(iarc_root(), 'data'),
        #        model='model3-drone-640x640',
        #        cmap={1:DetectRequest.CID_DRONE},
        #        use_gpu=self.use_gpu_)

        self.trk_ = Object_Tracker(use_gpu=self.use_gpu_)

        # ROS Handles
        self.cvbr_ = CvBridge()
        self.srv_ = rospy.Service('track', Track, self.detect_cb)
        self.pub_ = rospy.Publisher('dbg', PointStamped, queue_size=10)

        # Register Camera Handlers
        self.cam_ = {k : CameraHandle(k, self.cvbr_, self.data_cb) for k in srcs}

        # data
        self.track_ = []

        # Things to detect:
        # - Person ( <= 1 )
        # - Friendly Drone ( <= 4 )
        # - Enemy Drone ( <= ? )
        # - Bin/QR ( <= 4 )

    def filter_detection(self, req, din):
        dout = []
        for (cid, box, score) in zip(din['class'], din['box'], din['score']):
            if (req.cid is not req.CID_NULL) and (str(req.cid) is not str(cid)):
                # class mismatch
                continue
            if score < 0.5:
                # insufficient confidence
                continue
            box = BoxUtils.convert(box, BoxUtils.FMT_NYXYX, BoxUtils.FMT_NCCWH)
            dout.append( (cid, box) )
        return dout

    def detect_cb(self, req):
        src = req.source
        res_fail = DetectResponse(success=False)
        if not src in self.cam_:
            rospy.loginfo('Got Detection Request for [{}] but data does not exit yet'.format(src))
            return res_fail

        # parse input data
        img   = self.cam_[src].img_
        stamp = self.cam_[src].stamp_
        if (img is None) or (stamp is None):
            return res_fail
        
        dt = (rospy.Time.now() - self.cam_[src].stamp_).to_sec()
        if dt > 0.5:  # << TODO : MAGIC
            # more than 500ms passed since last image received:
            # image cannot be processed.
            # TODO : support post-request detection as action?(timeout)
            return res_fail

        dres = self.det_( img )
        if dres is None:
            rospy.loginfo('Detector failed for request : {}'.format(req))
            return res_fail

        # TODO : maybe support multi-class detection in the future
        dres = self.filter_detection(req, dres)

        if len(dres) <= 0:
            rospy.loginfo('Detector has found no matching objects for request: {}'.format(req))
            return res_fail

        # sort by box area
        # TODO : this code depends on box encoding
        cid, box = sorted(dres, key=lambda (_,box):(box[2]*box[3]))[-1]
        x,y,w,h = box

        if (req.track):
            # initialize tracking that object
            # TODO : filter for already tracked objects?
            print('box->', box)
            meta = self.trk_.init(img, box)
            self.track_.append( TrackData(src, cid, img, box, stamp, meta) )

        # finally, success!
        return DetectResponse(x=x,y=y,w=w,h=h, cid=int(cid), success=True)

    def data_cb(self, src, img, stamp):
        # TODO : save data in cam_ and run data_cb in step()
        # instead of inside the callback. May run into strange race conditions.
        now = rospy.Time.now()
        if (now - stamp).to_sec() > 0.5:
            rospy.loginfo_throttle(1.0, 'incoming data too old: [{}]-{}'.format(src, stamp) )
            # too old
            return

        if not (src in self.cam_):
            # should usually not reach here - error
            return
        cam = self.cam_[src]

        if not cam.has_info_:
            # should usually not reach here, maybe the first few frames
            return

        for t in self.track_:
            tres = self.trk_(img, t.box_, t.meta_)
            tscore = self.trk_.get_confidence(t.meta_)

            if tscore > 0.2:
                t.box_, t.meta_ = tres
                t.cnt_ += 1
                t.stamp_ = stamp
            else:
                # lost track
                t.box_ = None

        # filter tracks
        # TODO : support tenacious tracking? (recovery from loss / re-detection)
        # TODO : set timeout (10.0 = MAGIC)
        self.track_ = [t for t in self.track_ if (t.box_ is not None) and (stamp - t.stamp_).to_sec() < 10.0]

        #if len(self.track_) > 0:
        #    print self.track_

        # TODO : publish >> multiple << tracks simultaneously
        for t in self.track_:
            draw_bbox(img, BoxUtils.convert(t.box_, BoxUtils.FMT_NCCWH, BoxUtils.FMT_NYXYX), cls=None)

            iw, ih = cam.model_.width, cam.model_.height
            cx, cy = np.multiply(t.box_[:2], [iw,ih])
            ray3d  = cam.model_.projectPixelTo3dRay([cx, cy])

            self.pub_.publish(PointStamped(
                header=Header(frame_id=cam.model_.tfFrame(), stamp=stamp),
                point=Point(*ray3d)
                ))

        if self.dbg_:
            cv2.imshow('win', img) 
            cv2.waitKey(1)

        # pixel --> ray
        # x_rel, y_rel = loc2d
        # x, y = x_rel * cam.model_.width, y_rel * cam.model_.height
        # ray3d = cam.model_.projectPixelTo3dRay([x,y])

    def publish(self):
        pass

    def step(self):
        pass

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.step()
        #rospy.spin()

def main():
    rospy.init_node('tracker')
    node = TrackerNode()
    node.run()

if __name__ == "__main__":
    main()

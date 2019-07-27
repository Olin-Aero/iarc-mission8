#!/usr/bin/env python2

import rosparam
import rospy
import os
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from iarc_fuses.camera_handle import CameraHandle
from iarc_fuses.object_detection_tf import ObjectDetectorTF
from iarc_fuses.object_track import Object_Tracker
from iarc_fuses.utils import iarc_root

# currently doesn't do anything
class BinDetector(object):
    def __init__(self):
        pass
class QRQuadrantDetector(object):
    def __init__(self):
        pass

class DetectorFactory(object):
    def __init__(self):
        self.mmap_ = {
                'model-all-300x300' : ObjectDetectorTF,
                'bin' : BinDetector,
                'dasiam' : Object_Tracker,
                'qr' : QRQuadrantDetector
                }
        self.models_ = {}

    def __call__(self, model, cfg):
        if model in self.models_:
            return self.models_[model]
        # create
        if model not in self.mmap_:
            raise ValueError("Invalid Model Request : {}".format(model))
        self.models_[model] = self.mmap_[model]()
        self.models_[model].initialize( cfg )
        return self.models_[model]

class DetectorNode(object):
    def detect(self, img):
        return obj_id, bbox, score

class PerceptionNode(object):
    def __init__(self):
        # Parameters
        # global settings
        self.cfg_ = rospy.get_param('~cfg', rosparam.load_file(
            os.path.join(iarc_root(), 'config', 'perception.yaml'))[0][0] )
        print 'cfg'
        print '==============='
        print self.cfg_
        print '==============='

        # parameter overrides
        self.src_ = rospy.get_param('~src', self.cfg_['src'])
        self.dbg_ = rospy.get_param('~dbg', self.cfg_['dbg'])
        self.root_ = rospy.get_param('~root', os.path.join(iarc_root(), 'data'))
        self.max_dt_ = rospy.get_param('~max_dt', self.cfg_['max_dt'])
        self.dfac_ = DetectorFactory()

        # Data
        self.data_ = {}

        # ROS Handles
        self.cvbr_  = CvBridge()
        self.tf_buf_ = tf2_ros.Buffer()
        self.tfl_ = tf2_ros.TransformListener(self.tf_buf_)
        self.cam_ = {k : CameraHandle(k, self.cvbr_, self.data_cb) for k in self.src_}
        self.modules_ = {m : self.dfac_(m['model'], self.cfg_[m]) for m in self.cfg_['modules']}

    def data_cb(self, src, img, stamp):
        self.data_[src] = (img, stamp)
        return

    def step(self):
        # filter stale data
        self.data_ = {k:v for (k,v) in self.data_.iteritems() if (now - v.stamp).to_sec() > self.max_dt_}

        for d in self.data_:


        # TODO : save data in cam_ and run data_cb in step()
        # instead of inside the callback. May run into strange race conditions.
        now = rospy.Time.now()
        if (now - stamp).to_sec() > self.max_dt_:
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
        for m in self.modules_:
            pass

    def run(self):
        for m in self.modules_:
            m.init(self.src_)

def main():
    rospy.init_node('perception_node')
    node = PerceptionNode()
    node.run()

if __name__ == '__main__':
    main()

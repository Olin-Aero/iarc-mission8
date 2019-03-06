#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from iarc_msgs.srv import Detect, DetectRequest, DetectResponse
from iarc_fuses.object_detection_tf import ObjectDetectorTF
from iarc_fuses.object_track import Object_Tracker
import numpy as np
import cv2

class NullDetector(object):
    def __init__(self):
        pass
    def __call__(self, img):
        return (0.0, 0.0)

class NullTracker(object):
    def __init__(self):
        pass
    def init(self, img, box):
        """ 
        Initialize the tracker with the input image and the bounding box.

        Returns:
            any state-related metadata required for later tracking 
        """
        return None
    def __call__(self, img, box, meta):
        """
        Returns:
            box, state
        """
        return box, meta

def convert_box(box_in):
    """
    TODO : handle format arguments, i.e.
    #format_in='y,x,y,x',
    #format_out='cx,cy,w,h'
    """
    y0, x0, y1, x1 = box_in
    cx = (x0+x1)/2.0
    cy = (y0+y1)/2.0
    w  = float(y1-y0)
    h  = float(x1-x0)
    return [cx,cy,w,h]

def convert_box_2(box_in):
    """
    TODO : handle format arguments, i.e.
    #format_in='cx,cy,w,h'
    #format_out='y,x,y,x',
    """
    cx,cy,w,h = box_in
    x0 = cx - w / 2.0
    y0 = cy - h / 2.0
    x1 = cx + w / 2.0
    y1 = cy + h / 2.0

    return [y0,x0,y1,x1]

def draw_tfbox(img, box, cls=None):
    h,w = img.shape[:2]
    yxyx = box
    yxyx = np.multiply(yxyx, [h,w,h,w])
    yxyx = np.round(yxyx).astype(np.int32)
    y0,x0,y1,x1 = yxyx
    cv2.rectangle(img, (x0,y0), (x1,y1), (255,0,0), thickness=2)
    if cls is not None:
        org = ( max(x0,0), min(y1,h) )
        cv2.putText(img, cls, org, 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255),
                1, cv2.LINE_AA
                )

class CameraHandle(object):
    def __init__(self, src, bridge, callback):
        # parameters
        self.src_   = src
        self.model_ = PinholeCameraModel()
        self.has_info_ = False

        # data
        self.img_ = None
        self.stamp_ = None

        # handles
        self.bridge_ = bridge
        self.sub_   = rospy.Subscriber(
                '{}/image_raw'.format(src),
                Image,
                self.data_cb
                )
        self.sub_i_ = rospy.Subscriber(
                '{}/camera_info'.format(src),
                CameraInfo,
                self.info_cb
                )
        self.callback_ = callback

    def info_cb(self, info):
        self.model_.fromCameraInfo(info)
        self.has_info_ = True
        # no longer care about camera_info
        self.sub_i_.unregister()

    def data_cb(self, data):
        img = self.bridge_.imgmsg_to_cv2(data, 'bgr8')
        self.img_ = img
        self.stamp_ = data.header.stamp
        self.callback_(self.src_, img, data.header.stamp)

class Track(object):
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
        # parse sources
        srcs = rospy.get_param('~srcs', [])
        rospy.loginfo('Tracker Received Sources : {}'.format(srcs))

        # Processing Handles
        #self.det_ = NullDetector()
        self.det_ = ObjectDetectorTF(cmap={1:DetectRequest.CID_PERSON})
        self.trk_ = Object_Tracker()
        #self.trk_ = NullTracker()

        # ROS Handles
        self.cvbr_ = CvBridge()
        self.srv_ = rospy.Service('detect', Detect, self.detect_cb)

        # Register Camera Handlers
        self.cam_ = {k : CameraHandle(k, self.cvbr_, self.data_cb) for k in srcs}

        # data
        self.track_ = []

        # Things to detect:
        # - Person ( <= 1 )
        # - Friendly Drone ( <= ? )
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
            box = convert_box(box)
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
            self.track_.append( Track(src, cid, img, box, stamp, meta) )

        # finally, success!
        return DetectResponse(x=x,y=y,w=w,h=h, cid=int(cid), success=True)

    def data_cb(self, src, img, stamp):
        # TODO : save data in cam_ and run data_cb in step()
        # instead of inside the callback. May run into strange race conditions.

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
            print('tscore', tscore)

            if tscore > 0.2:
                t.box_, t.meta_ = tres
                t.cnt_ += 1
                t.stamp_ = stamp
                print('box-->', t.box_)

        # filter tracks
        # TODO : support tenacious tracking? (recovery from loss / re-detection)
        # TODO : set timeout (10.0 = MAGIC)
        self.track_ = [t for t in self.track_ if (t.box_ is not None) and (stamp - t.stamp_).to_sec() < 10.0]

        #if len(self.track_) > 0:
        #    print self.track_

        # TODO : publish tracks
        for t in self.track_:
            draw_tfbox(img, convert_box_2(t.box_), cls=None)

        cv2.imshow('win', img) 
        cv2.waitKey(1)

        # pixel --> ray
        # x_rel, y_rel = loc2d
        # x, y = x_rel * cam.model_.width, y_rel * cam.model_.height
        # ray3d = cam.model_.projectPixelTo3dRay([x,y])

    def publish(self):
        pass

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('tracker')
    node = TrackerNode()
    node.run()

if __name__ == "__main__":
    main()

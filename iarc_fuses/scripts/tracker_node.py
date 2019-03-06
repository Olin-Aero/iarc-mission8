#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from iarc_msgs.srv import Detect, DetectRequest, DetectResponse
from iarc_fuses.object_detection_tf import ObjectDetectorTF

class NullDetector(object):
    def __init__(self):
        pass
    def __call__(self, img):
        return (0.0, 0.0)

class NullTracker(object):
    def __init__(self):
        pass
    def __call__(self, img, box):
        return box

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
    def __init__(self, src, cid, img, box, stamp):
        self.src_ = src
        self.cid_ = cid
        self.img_ = img
        self.box_ = box
        self.stamp_ = stamp

        self.cnt_ = 1 # count the number of frames seen

class TrackerNode(object):
    def __init__(self):
        # parse sources
        srcs = rospy.get_param('~srcs', [])
        rospy.loginfo('Tracker Received Sources : {}'.format(srcs))

        # Processing Handles
        #self.det_ = NullDetector()
        self.det_ = ObjectDetectorTF(cmap={1:DetectRequest.CID_PERSON})
        self.trk_ = NullTracker()

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
        # filter by class match
        dres = [(cid, box) for (cid, box) in zip(dres['class'], dres['box'])
                if (req.cid is req.CID_NULL) or (str(req.cid) is str(cid))]

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
            self.track_.append( Track(src, cid, img, box, stamp) )

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

        for track in self.track_:
            track.box_ = self.trk_(img, track.box_)
            track.cnt_ += 1
            track.stamp_ = rospy.Time.now()

        # filter tracks
        # TODO : support tenacious tracking? (recovery from loss / re-detection)
        # TODO : set timeout (10.0 = MAGIC)
        self.track_ = [t for t in self.track_ if (t.box_ is not None) and (rospy.Time.now() - track.stamp_).to_sec() < 10.0]

        # TODO : publish tracks

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

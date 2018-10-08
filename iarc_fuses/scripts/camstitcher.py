#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from tf_conversions import posemath as pm
import tf
import message_filters

from iarc_fuses import utils as U

def ground_area(cm, tf_x):
    """ Compute 4-corners of ground-plane from camera.
    Args:
        cm(image_geometry.PinholeCameraModel): reference camera model
        tf_x(tuple): (txn,qxn) as returned by tf.TransformListener.lookupTransform()
    """

    # from http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
    # tfl.lookupTransform(target, source, time)
    # if applied to data, the result will transform data in the source_frame into the target_frame. See 
    txn, qxn = tf_x

    # K = 3x3 camera projection 
    w,h = cm.width, cm.height
    M_x = pm.toMatrix(pm.fromTf(tf_x))

    uv4 = [[0,0], [h,0], [h,w], [0,w]] # [4,2] formatted (u,v)
    cray = [cm.projectPixelTo3dRay(uv) for uv in uv4] # [4,3] formatted (x,y,z) ??
    # convert ray to map coord
    mray = np.dot(cray, M_x[:3,:3].T) # right-multiply rotation matrix

    # extend ray to groundplane
    l = - txn[2] / mray[:,2]
    # need to flip z since mray is pointing downwards
    # i.e. same as mray[:,2].dot([0,0,-1]) which is the correct distance
    gray = l[:,np.newaxis] * mray
    return gray[:,:2] + np.reshape(txn[:2], [-1,2])

class CamStitcher(object):
    def __init__(self):
        # global mapping parameters
        self.map_width_ = rospy.get_param('~map_width', default=28.0)
        self.map_height_ = rospy.get_param('~map_width', default=15.0)
        self.map_res_ = rospy.get_param('~map_res', default=0.02) # 5cm resolution
        n = np.ceil(self.map_height_ / self.map_res_).astype(np.int32)
        m = np.ceil(self.map_width_  / self.map_res_).astype(np.int32)
        self.map_shape_ = (n,m,3)
        self.map_ = np.zeros(self.map_shape_, dtype=np.uint8)

        # camera sources
        self.sources_ = rospy.get_param('~sources', default='')
        self.slop_ = rospy.get_param('~slop', default=0.01)
        self.sources_ = [e for e in self.sources_.split(';') if len(e)>0]

        # ROS Handles
        self.br_ = CvBridge()
        self.tfl_ = tf.TransformListener()

        # setup camera subscribers
        self.sub_   = {k:None for k in self.sources_}
        self.data_   = {k:None for k in self.sources_}
        self.model_   = {k:None for k in self.sources_}

        for src in self.sources_:
            img_sub  = message_filters.Subscriber('{}/image_raw'.format(src), Image) 
            info_sub = message_filters.Subscriber('{}/camera_info'.format(src), CameraInfo)
            self.sub_[src] = message_filters.ApproximateTimeSynchronizer(
            [info_sub, img_sub], 10, self.slop_, allow_headerless=True)
            self.sub_[src].registerCallback(lambda i,m : self.data_cb(src,i,m))

    def xy2uv(self, xy):
        # convert x,y positions on the map
        x, y = np.transpose(xy)
        n, m = self.map_shape_[:2]
        u = U.rint(n/2. + (y / self.map_res_))
        v = U.rint(m/2. + (x / self.map_res_))
        return np.stack([v,u], axis=-1)

    def data_cb(self, cam_id, info, msg):
        try:
            cv_img = self.br_.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr_throttle(1.0,
                    'CamStitcher CV Bridge Error : {}'.format(e))

        self.data_[cam_id] = (msg.header, cv_img)
        if self.model_[cam_id] is None:
            self.model_[cam_id] = PinholeCameraModel()
            self.model_[cam_id].fromCameraInfo(info)

    def proc(self, cam_id, reset=True):
        if self.data_[cam_id] is None:
            return
        header, img = self.data_[cam_id] 
        cm = self.model_[cam_id]
        stamp = header.stamp
        try:
            tf_x = self.tfl_.lookupTransform('odom', cm.tf_frame, rospy.Time(0)) # tf_frame is optical frame
        except tf.Exception as e:
            rospy.loginfo_throttle(1.0, 'Cam Stitcher TF Exception : {}'.format(e))
            return

        w,h = cm.width, cm.height
        src_ar = np.asarray([[0,0], [h,0], [h,w], [0,w]], dtype=np.float32)
        dst_ar = ground_area(cm, tf_x)
        dst_ar = self.xy2uv(dst_ar).astype(np.float32)

        # TODO : evaluate if roi-based approach is more efficient
        M = cv2.getPerspectiveTransform(src_ar, dst_ar)
        cv2.warpPerspective(img, M, self.map_.shape[:2][::-1], dst=self.map_,
                borderMode=cv2.BORDER_TRANSPARENT)

        #cv2.polylines(self.map_, U.rint([dst_ar]), True, (255,255,255))
        #rospy.loginfo_throttle(1.0, 'dst_ar:{}'.format(dst_ar))

        if reset:
            self.data_[cam_id] = None

    def step(self):
        for src in self.sources_:
            self.proc(src)

    def run(self):
        rate = rospy.Rate(50)
        cv2.namedWindow('map', cv2.WINDOW_NORMAL)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()
            cv2.imshow('map', np.flipud(self.map_))
            cv2.waitKey(10)

def main():
    rospy.init_node('cam_stitcher')
    node = CamStitcher()
    node.run()

if __name__ == "__main__":
    main()

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
from functools import partial
import shapely.geometry as sp

def horizon_area(cm, tf_x, inv=False):
    """ Compute area under the horizon in the image according to current camera transformation.

    Args:
        cm(image_geometry.PinholeCameraModel): camera model for image.
        tf_x(tuple): (txn,qxn) tuple for translation and rotation(quaternion).
            This is the transformation that converts camera to map frame.
            Refer to results from tf.TransformListener.lookupTransform()
        inv(bool): if True, returns area above the horizon instead.

    Returns:
        points(np.ndarray): [N,2] sorted array formatted (x,y),
            indicating the polygonal area above or below the horizon.
    """
    # cm parameters unwrap
    w, h = cm.width, cm.height
    fx,fy = cm.fx(), cm.fy()
    cx,cy = cm.cx(), cm.cy()

    # transformations
    R = tf.transformations.quaternion_matrix(tf_x[1])[:3,:3]
    Rz = R[2,:3]
    a,b,c = Rz

    # process corners
    crn = np.asarray([[0,0], [0,h], [w,h], [w,0]], dtype=np.float32) # 4 corners
    cray = [cm.projectPixelTo3dRay(e) for e in crn] # [4,3] formatted (x,y,z) ??

    # convert ray to map coord
    mray = np.dot(cray, R.T) # right-multiply rotation matrix
    if inv:
        # corner select for non-ground
        sel = (mray[:,2] > 0)
    else:
        # corner select for ground plane
        sel = (mray[:,2] < 0)

    if(np.all(sel)):
        return crn

    # horizon line segment from x=0 to x=w
    v0 = (fy/b) * (a*cx/fx-c) + cy
    m = (cy - v0 + (fy/b)*(-c-a*(w-cx)/fx))/w
    v1 = (v0 + m*w)
    pa, pb = [0, v0], [w, v1]

    # intersect bounds rectification for y index
    if v0 < 0:
        pa = [(0-v0)/m, 0]
    elif v0 > h:
        pa = [(h-v0)/m, h]
    if v1 < 0:
        pb = [w+(0-v1)/m, 0]
    elif v1 > h:
        pb = [w+(h-v1)/m, h]

    crn = crn[sel]

    ps = np.concatenate([crn, [pa,pb]], axis=0)

    # sort by some order
    center = np.mean(ps, axis=0)
    delta = ps - center
    ang = np.arctan2(delta[:,0], delta[:,1])
    ps = ps[np.argsort(-ang)]

    return ps

def ground_area(cm, src, tf_x):
    """ Compute 4-corners of ground-plane from camera.

    Args:
        cm(image_geometry.PinholeCameraModel): camera model for image.
        src(np.ndarray): [4,2] array of four corners, formatted (u,v).
            Represents the area in the frame to compute the groundplane projection.
        tf_x(tuple): (txn,qxn) tuple for translation and rotation(quaternion).
            Refer to results from tf.TransformListener.lookupTransform()
    Returns:
        points(np.ndarray): [4,2] array of four corners, formatted (x,y)
            z value is implicitly zero w.r.t. the source tf frame.
    """

    txn, qxn = tf_x

    # K = 3x3 camera projection 
    w,h = cm.width, cm.height
    M_x = pm.toMatrix(pm.fromTf(tf_x))

    cray = [cm.projectPixelTo3dRay(e) for e in src] # [4,3] formatted (x,y,z) ??
    # convert ray to map coord
    mray = np.dot(cray, M_x[:3,:3].T) # right-multiply rotation matrix

    # extend ray to groundplane
    l = - txn[2] / mray[:,2]
    # need to flip z since mray is pointing downwards
    # i.e. same as mray[:,2].dot([0,0,-1]) which is the correct distance
    gray = l[:,np.newaxis] * mray
    return gray[:,:2] + np.reshape(txn[:2], [-1,2])

class CamStitcher(object):
    """
    Combines multiple camera sources and outputs a single global top-down map image.
    
    Note:
        All sources are specified with a comma-separated list to the ~sources ROS parameter.
        Example: `rosrun iarc_fuses camstitcher.py _sources:='/ardrone/bottom;/ardrone/front'`
    """
    def __init__(self):
        """ Load parameters from ROS server and initializes data cache and ROS handles.
        
        """
        # global mapping parameters
        self.map_frame_ = rospy.get_param('~map_frame', default='odom')
        self.map_width_ = rospy.get_param('~map_width', default=28.0)
        self.map_height_ = rospy.get_param('~map_width', default=15.0)
        self.map_res_ = rospy.get_param('~map_res', default=0.02) # 5cm resolution
        n = np.ceil(self.map_height_ / self.map_res_).astype(np.int32)
        m = np.ceil(self.map_width_  / self.map_res_).astype(np.int32)
        self.map_shape_ = (n,m,3)
        self.map_ = np.zeros(shape=(n,m,3), dtype=np.uint8)
        self.viz_ = np.zeros(shape=(n,m,3), dtype=np.uint8)
        self.tmp_ = np.zeros(shape=(n,m,1), dtype=np.uint8)
        self.tmp2_ = self.map_.copy()
        self.mask_ = None

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
            self.sub_[src].registerCallback(partial(self.data_cb, src))

    def xy2uv(self, xy):
        """ convert physical x,y positions to map coordinates.
        Args:
            xy(np.ndarray): [N,2] array formatted (x,y)

        Returns:
            uv(np.ndarray): [N,2] array formatted (u,v).
                Note that x->u and y->v.
        """

        # convert x,y positions on the map
        x, y = np.transpose(xy)
        n, m = self.map_shape_[:2]
        u = U.rint(m/2. + (x / self.map_res_))
        v = U.rint(n/2. + (y / self.map_res_))
        return np.stack([u,v], axis=-1)

    def data_cb(self, cam_id, info, msg):
        """ callback for data; immediately stores data to member cache.

        Args:
            cam_id(str): camera identifier assigned in initialization.
            info(sensor_msgs.CameraInfo): Camera calibration/parameters information.
            msg(sensor_msgs.Image): Formatted ROS Image data.
        """
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
        """ process data from a camera.

        Args:
            cam_id(str): unique identifier for a camera, automatically generated from ~sources.
                Beware that cam_id is NOT the same as the tf frame corresponding to the camera.
            reset(bool): Whether data will be cleared after processing. (default: True)
                It is recommended to keep this parameter untouched to prevent duplicate processing.
        """
        if self.data_[cam_id] is None:
            return

        # unpack metadata ...
        header, img = self.data_[cam_id] 
        cm = self.model_[cam_id]
        stamp = header.stamp

        try:
            # note from http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
            # tfl.lookupTransform(target, source, time)
            # if applied to data, the result will transform data in the source_frame into the target_frame.
            self.tfl_.waitForTransform(self.map_frame_, cm.tf_frame, stamp, rospy.Duration(0.5))
            tf_x = self.tfl_.lookupTransform(self.map_frame_, cm.tf_frame, stamp) # tf_frame is optical frame
        except tf.Exception as e:
            rospy.loginfo_throttle(1.0, 'Cam Stitcher TF Exception : {}'.format(e))
            return

        w,h = cm.width, cm.height
        n,m = self.map_shape_[:2]

        # four corners source
        src_ar = np.asarray([[0,0], [0,h], [w,h], [w,0]], dtype=np.float32)

        # four corners groundplane projection
        dst_ar = ground_area(cm, src_ar, tf_x)
        dst_ar = self.xy2uv(dst_ar).astype(np.float32)
        M = cv2.getPerspectiveTransform(src_ar, dst_ar)

        # horizon area mask to prevent skylines/wall-like objects from getting written on the map
        hor_ar = horizon_area(cm, tf_x, inv=False)

        # fill map!
        if self.mask_ is None:
            self.mask_ = np.zeros((h,w,1), dtype=np.uint8)
        self.mask_.fill(0)
        cv2.fillConvexPoly(self.mask_, U.rint(hor_ar), [255])# create image mask
        cv2.warpPerspective(self.mask_, M, (m,n), dst=self.tmp_) # image mask to map update mask
        cv2.warpPerspective(img, M, (m,n), dst=self.tmp2_) # warped content
        sel = np.broadcast_to(self.tmp_ == 255, self.tmp2_.shape) # mask index
        self.map_[sel] = self.tmp2_[sel] # finally, update data

        # visualization
        #np.copyto(self.viz_, self.map_)
        #dbg_ar = ground_area(cm, hor_ar, tf_x)
        #print('dbg_ar', dbg_ar)
        #ix_ar = sp.Polygon(dst_ar).intersection(sp.Polygon(dbg_ar))
        #print(ix_ar)


        if reset:
            self.data_[cam_id] = None

    def step(self):
        """ single step through all known camera sources"""
        for src in self.sources_:
            self.proc(src)

    def run(self):
        """ actually run the node """
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

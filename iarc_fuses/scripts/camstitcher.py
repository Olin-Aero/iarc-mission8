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

def uvec(x):
    return x / np.linalg.norm(x,axis=-1)

def ray_seg_ix(rayOrigin, rayDirection, point1, point2):
    """
    from https://stackoverflow.com/a/29020182
    """

    # Convert to numpy arrays
    rayOrigin = np.array(rayOrigin, dtype=np.float)
    rayDirection = np.array(uvec(rayDirection), dtype=np.float32)
    point1 = np.array(point1, dtype=np.float)
    point2 = np.array(point2, dtype=np.float)

    # Ray-Line Segment Intersection Test in 2D
    # http://bit.ly/1CoxdrG
    v1 = rayOrigin - point1
    v2 = point2 - point1
    v3 = np.array([-rayDirection[1], rayDirection[0]])
    if np.dot(v2,v3) == 0:
        return None
    t1 = np.cross(v2, v1) / np.dot(v2, v3)
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
        return rayOrigin + t1 * rayDirection
    return None

def order_points(pts):
    # sort the points based on their x-coordinates
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # now that we have the top-left coordinate, use it as an
    # anchor to calculate the Euclidean distance between the
    # top-left and right-most points; by the Pythagorean
    # theorem, the point with the largest distance will be
    # our bottom-right point
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="float32")

def horizon_area(cm, tf_x, inv=False):
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

    #v0 = (a * cm.cx()/cm.fx() + b * cm.cy()/cm.fy() - c) / b
    #m  = (-a * (cm.width - cm.cx()/cm.fx()) - c - b*v0) * (cm.fy() / (b*cm.width))

    v0 = (fy/b) * (a*cx/fx-c) + cy
    m = (cy - v0 + (fy/b)*(-c-a*(w-cx)/fx))/w

    v1 = (v0 + m*w)
    pa, pb = [0, v0], [w, v1]

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


#def horizon_area(cm, tf_x):
#    tvec = np.asarray([np.cos(roll)*cm.fx(), np.sin(roll)*cm.fy()])
#    pvec = np.asarray([-tvec[1], tvec[0]])
#    tvec /= np.linalg.norm(tvec)
#
#    dp = pitch * pvec
#    c = np.asarray([cm.cx(), cm.cy()])
#
#    w,h = cm.width, cm.height
#
#    p0 = c + dp
#
#    corners = np.asarray([[0,0], [w,0], [w,h], [0,h]], dtype=np.float32)
#
#    det = np.cross(tvec, corners - p0) #2 x [4,2]
#
#    if np.all(det < 0):
#        # all corners are below horizon!
#        return corners
#
#    if np.all(det > 0):
#        # all corners are above horizon
#        return None
#
#    # only some corners are below horizon
#
#    ix_t = ray_seg_ix(p0, tvec, corners[0], corners[1])
#    ix_r = ray_seg_ix(p0, tvec, corners[1], corners[2])
#    ix_l = ray_seg_ix(p0, tvec, corners[2], corners[3])
#    ix_b = ray_seg_ix(p0, tvec, corners[3], corners[0])
#    pa = [e for e in [ix_t,ix_r,ix_l,ix_b] if e is not None][0]
#    print('pa', pa)
#
#    ix_t = ray_seg_ix(p0, -tvec, corners[0], corners[1])
#    ix_r = ray_seg_ix(p0, -tvec, corners[1], corners[2])
#    ix_l = ray_seg_ix(p0, -tvec, corners[2], corners[3])
#    ix_b = ray_seg_ix(p0, -tvec, corners[3], corners[0])
#    pb = [e for e in [ix_t,ix_r,ix_l,ix_b] if e is not None][0]
#    print('pb', pb)
#
#    ps = np.concatenate([corners[det<0], [pa,pb]],axis=0)
#
#    ups = (ps - p0)
#    ups /= np.linalg.norm(ups, axis=-1, keepdims=True)
#    ang = U.anorm(np.arctan2(ups[:,1], ups[:,0]) - (-roll))
#    ps = ps[np.argsort(ang)]
#    print('ps', np.around(ps,2))
#    return ps[:,::-1] # swap x-y

def ground_area(cm, src, tf_x):
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
    def __init__(self):
        # global mapping parameters
        self.map_width_ = rospy.get_param('~map_width', default=28.0)
        self.map_height_ = rospy.get_param('~map_width', default=15.0)
        self.map_res_ = rospy.get_param('~map_res', default=0.02) # 5cm resolution
        n = np.ceil(self.map_height_ / self.map_res_).astype(np.int32)
        m = np.ceil(self.map_width_  / self.map_res_).astype(np.int32)
        self.map_shape_ = (n,m,3)
        self.map_ = np.zeros(self.map_shape_, dtype=np.uint8)
        self.tmp_ = self.map_.copy()
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
            abs_frame = 'odom' # should be map
            tf_x = self.tfl_.lookupTransform(abs_frame, cm.tf_frame, rospy.Time(0)) # tf_frame is optical frame
        except tf.Exception as e:
            rospy.loginfo_throttle(1.0, 'Cam Stitcher TF Exception : {}'.format(e))
            return

        w,h = cm.width, cm.height
        n,m = self.map_shape_[:2]

        src_ar = np.asarray([[0,0], [0,h], [w,h], [w,0]], dtype=np.float32)

        dst_ar = ground_area(cm, src_ar, tf_x)
        dst_ar = self.xy2uv(dst_ar).astype(np.float32)

        r,p,y = tf.transformations.euler_from_quaternion(tf_x[1])

        roi = [np.min(dst_ar,axis=0), np.max(dst_ar,axis=0)]
        [j0,i0], [j1,i1] = U.rint(roi)

        hor_ar = horizon_area(cm, tf_x, inv=False)
        if self.mask_ is None:
            self.mask_ = np.zeros((h,w,1), dtype=np.uint8)
        self.mask_.fill(0)
        cv2.fillConvexPoly(self.mask_, U.rint(hor_ar), 255)
        
        mask = np.tile(self.mask_, (1,1,3))
        M = cv2.getPerspectiveTransform(src_ar, dst_ar)
        cv2.warpPerspective(mask, M, (m,n), dst=self.tmp_,
                )
        cv2.warpPerspective(img, M, (m,n), dst=self.tmp2_,
                )
        sel = (self.tmp_ == 255)
        self.map_[sel] = self.tmp2_[sel]

        #sel = (self.tmp_ >= 255)
        #cv2.warpPerspective(img, M, (mw,mh), dst=self.tmp2_)
        #self.map_[sel] = self.tmp2_[sel]

        # == OPT : GOOD ==
        #hor_ar = horizon_area(cm, tf_x, inv=True)
        #cv2.fillConvexPoly(img, U.rint(hor_ar), 0)
        #M = cv2.getPerspectiveTransform(src_ar, dst_ar)
        #h,w = self.map_shape_[:2]
        #cv2.warpPerspective(img, M, (w,h), dst=self.map_,
        #        borderMode=cv2.BORDER_TRANSPARENT)
        # ================

        #hor_ar = horizon_area(cm, tf_x) # account for optical pitch-yaw
        #hor_ar = ground_area(cm, hor_ar, tf_x)
        #hor_ar = self.xy2uv(hor_ar).astype(np.float32)

        ## create mask
        #(j0,i0), (j1,i1) = np.min(hor_ar, axis=0), np.max(hor_ar, axis=0)
        #[i0,j0,i1,j1] = U.rint([i0,j0,i1,j1])
        #mask = self.mask_[i0:i1,j0:j1]
        #mask.fill(0)
        #print('poly', U.rint(hor_ar - np.reshape([j0,i0], [-1,2]) ))
        #cv2.fillConvexPoly(mask, U.rint(hor_ar - np.reshape([j0,i0], [-1,2]) ), 255)

        #(j0,i0), (j1,i1) = np.min(dst_ar, axis=0), np.max(dst_ar, axis=0)
        #M = cv2.getPerspectiveTransform(src_ar, dst_ar - [j0,i0] )
        #cv2.warpPerspective(img, M, (j1-j0,i1-i0), dst=self.tmp_[i0:i1,j0:j1])
        #self.map_[i0:i1,j0:j1] = np.where(mask[...,np.newaxis], self.tmp_[i0:i1,j0:j1], self.map_[i0:i1,j0:j1])

        ##mask = mask.astype(np.bool)
        ##mask = mask[...,np.newaxis]
        ##print('ms', mask.shape)

        ##hor_ar = ground_area(cm, hor_ar, tf_x)
        ##hor_ar = self.xy2uv(hor_ar).astype(np.float32)

        ### TODO : evaluate if roi-based approach is more efficient

        #M = cv2.getPerspectiveTransform(src_ar, dst_ar)
        #self.tmp_.fill(0)
        #cv2.warpPerspective(img, M, self.map_.shape[:2][::-1], dst=self.tmp_,
        #        borderMode=cv2.BORDER_TRANSPARENT)

        #mask = (self.mask_ == 255)
        ##self.map_[mask] = self.tmp_[mask] #self.mask_ == 255] = self.tmp_[self.mask_==
        #self.map_ = np.where(mask[...,np.newaxis], self.tmp_, self.map_)
        #rospy.loginfo_throttle(1.0, "hmm?")

        #self.map_[mask] = tmp

        #cv2.polylines(self.map_, U.rint([hor_ar]), True, (255,0,0),thickness=5)
        ##rospy.loginfo_throttle(1.0, 'dst_ar:{}'.format(dst_ar))

        #cv2.polylines(img, U.rint([hor_ar]), True, (255,0,0),thickness=5)
        #cv2.imshow('img', img)

        if reset:
            self.data_[cam_id] = None

    def step(self):
        for src in self.sources_:
            self.proc(src)

    def run(self):
        rate = rospy.Rate(50)
        cv2.namedWindow('map', cv2.WINDOW_NORMAL)
        cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()
            cv2.imshow('map', np.flipud(self.map_))
            #cv2.imshow('mask', self.mask_)
            cv2.waitKey(10)

def main():
    rospy.init_node('cam_stitcher')
    node = CamStitcher()
    node.run()

if __name__ == "__main__":
    main()

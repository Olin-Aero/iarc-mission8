# --------------------------------------------------------
# DaSiamRPN
# Licensed under The MIT License
# Written by Qiang Wang (wangqiang2015 at ia.ac.cn)
# --------------------------------------------------------
#!/usr/bin/python

import glob, cv2, torch
import numpy as np
import utils
from os.path import realpath, dirname, join

from DaSiamRPN.net import SiamRPNvot
from DaSiamRPN.run_SiamRPN import SiamRPN_init, SiamRPN_track
from DaSiamRPN.utils import get_axis_aligned_bbox, cxy_wh_2_rect


class Object_Tracker():
    def __init__(self):
        # Hard-coded sub-directory (TODO: maybe change this?)
        daSiamDir = 'DaSiamRPN/'

        # load net
        self.net = SiamRPNvot()
        self.net.load_state_dict(torch.load(join(realpath(dirname(__file__)),
            daSiamDir + 'SiamRPNVOT.model')))
        self.net.eval().cuda()
        self.state = None
 
    def get_confidence(self):
        """Returns confidence (score) of the current tracker output
        
        Returns:
            float: Score between 0 and 1 (where 1 is more confident)
        """
        return self.state["score"]

    def start_tracking(self, image, bounding_box):
        """Begins tracking of an object contained in a bounding box
        
        Args:
            image (numpy.ndarray): Input image, given as a numpy 
                    array (3D array with form (axis1, axis2, axis3) == (Xpos, Ypos, Color))
            bounding_box (4-tuple): Bounding box of object, in 
                    form (center_x, center_y, width, height)
        """

        if np.any( np.greater(np.asarray(bounding_box), 1.0)):
            # input is probably not relative
            pass
        [cx, cy, w, h] = bounding_box
        # tracker init
        target_pos, target_sz = np.array([cx, cy]), np.array([w, h])
        self.state = SiamRPN_init(image, target_pos, target_sz, self.net)

    def track(self, next_image, display_tracking_speed=False):
        """Track the object in an image using the pre-calculated internal state
        and a new image. (start_tracking must be run before this function.)
        
        Args:
            next_image (numpy.ndarray): Input image, given as a numpy 
                    array (3D array with form (axis1, axis2, axis3) == (Xpos, Ypos, Color))
            display_tracking_speed (bool, optional): Option to print trakcing fps
        
        Returns:
            array[float]: Returns a relative bounding box of type [x_corner, y_corner, width, height] 
        """
        # tracking and visualization
        toc = 0

        tic = cv2.getTickCount()
        self.state = SiamRPN_track(self.state, next_image)  # track
        toc += cv2.getTickCount()-tic
        res = (self.state['target_pos'][0] + self.state['target_sz'][0]/2, 
                self.state['target_pos'][1] + self.state['target_sz'][1]/2, 
                self.state['target_sz'][0], self.state['target_sz'][1])
        
        res = utils.convert_to_relative(np.shape(next_image), res)

        if display_tracking_speed:
            print('Tracking Speed {:.1f}fps'.format((len(image_files)-1)/(toc/cv2.getTickFrequency())))
        return res

    

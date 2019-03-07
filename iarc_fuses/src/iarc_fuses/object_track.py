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
    def __init__(self,
            root='DaSiamRPN/',
            model='SiamRPNVOT.model',
            use_gpu=False
            ):
        """
        root(str): Persistent model data Directory.
        model(str): name of the model[file] to run.
        use_gpu(bool): enable gpu.
        """
        # cache args
        self.root_ = root
        self.model_ = model
        self.use_gpu_ = use_gpu

        # Hard-coded sub-directory (TODO: maybe change this?)
        model_path = join(realpath(dirname(__file__)), root, model)

        # load net
        self.net = SiamRPNvot()
        self.net.load_state_dict(torch.load(model_path))
        if self.use_gpu_:
            self.net.eval().cuda()
        else:
            self.net.eval()

    @staticmethod
    def get_confidence(state):
        """Returns confidence (score) of the current tracker output
        
        Returns:
            float: Score between 0 and 1 (where 1 is more confident)
        """
        return state["score"]

    def init(self, image, bounding_box):
        """Begins tracking of an object contained in a bounding box
        
        Args:
            image(A(H,W,3), uint8): Input image
            bounding_box (4-tuple): Bounding box of object, in 
                    form (center_x, center_y, width, height);
                    relative coordinates [0-1]
        """

        if np.any( np.greater(np.asarray(bounding_box), 1.0)):
            # input is probably not relative.
            # we could resolve this, but we choose to throw an error.
            raise ValueError("Please Make sure that the input box is relative !!")

        img_h, img_w = np.shape(image)[:2]
        [cx, cy, w, h] = utils.convert_to_pixels([img_w,img_h], bounding_box) 
        # tracker init
        target_pos, target_sz = np.array([cx, cy]), np.array([w, h])
        state = SiamRPN_init(image, target_pos, target_sz, self.net, use_gpu=self.use_gpu_)

        return state

    def __call__(self, image, box, state, display_tracking_speed=False):
        """Track the object in an image using the pre-calculated internal state
        and a new image. (init must be run before this function.)
        
        Args:
            image(A(H,W,3), uint8): Input image
            box : Unused; only here for compatibility.
            display_tracking_speed (bool, optional): Option to print trakcing fps
        
        Returns:
            box(A(4)): Returns a relative bounding box of type [center_x, center_y, width, height] 
            state(?): Returns meta-information required for maintaining tracking state.

        Note:
            A(shape[,dtype=float]) in the above documentation is a shorthand for numpy.array.
        """
        # tracking and visualization
        toc = 0

        tic = cv2.getTickCount()
        state = SiamRPN_track(state, image, use_gpu=self.use_gpu_)  # track
        toc += cv2.getTickCount()-tic

        box = (state['target_pos'][0],
                state['target_pos'][1],
                state['target_sz'][0], 
                state['target_sz'][1])

        img_h, img_w = np.shape(image)[:2]
        box = utils.convert_to_relative([img_w,img_h], box)

        return box, state

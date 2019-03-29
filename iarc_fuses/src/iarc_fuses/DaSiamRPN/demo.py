# --------------------------------------------------------
# DaSiamRPN
# Licensed under The MIT License
# Written by Qiang Wang (wangqiang2015 at ia.ac.cn)
# --------------------------------------------------------
#!/usr/bin/python

import glob, cv2, torch
from PIL import Image
import numpy as np
from os.path import realpath, dirname, join
import os
from net import SiamRPNvot
from run_SiamRPN import SiamRPN_init, SiamRPN_track
from utils import get_axis_aligned_bbox, cxy_wh_2_rect



def main():

    vid_file = os.path.expanduser("~/Videos/VID_20190327_195111.mp4")

    cap = cv2.VideoCapture(vid_file)

    # load net
    net = SiamRPNvot()
    net.load_state_dict(torch.load(join(realpath(dirname(__file__)), 'SiamRPNVOT.model')))
    net.eval().cuda()

    # # image and init box
    # image_files = sorted(glob.glob('./bag/*.jpg'))
    init_rbox = [334.02,128.36,438.19,188.78,396.39,260.83,292.23,200.41]
    [cx, cy, w, h] = get_axis_aligned_bbox(init_rbox)

    # tracker init
    target_pos, target_sz = np.array([cx, cy]), np.array([w, h])
    ret, im = cap.read()

    state = SiamRPN_init(im, target_pos, target_sz, net, use_gpu=True)

    toc = 0
    while(True):
        # Capture frame-by-frame
        ret, im = cap.read()

        tic = cv2.getTickCount()
        state = SiamRPN_track(state, im, use_gpu=True)  # track
        toc += cv2.getTickCount()-tic
        res = cxy_wh_2_rect(state['target_pos'], state['target_sz'])
        res = [int(l) for l in res]
        cv2.rectangle(im, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 3)
        cv2.imshow('SiamRPN', im)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()

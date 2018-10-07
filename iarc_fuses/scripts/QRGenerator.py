#!/usr/bin/env python2
import qrcode
import os
import rospkg
from random import randint

rospack = rospkg.RosPack() 
pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
for i in range(20):
    code = ""
    # makes a random 4 digit code
    for i in range(4):
        code = code + str(randint(0,9))
    img = qrcode.make(code) # makes a qr code from that 4 digit code
    img.save(os.path.join(pkgRoot,"QRCodes",code +'.png'),"PNG") # saves the qr code

    imgWidth, imgHeight = img.size 
    for i in range(2):
        for j in range(2):
            box = (imgWidth / 2 * i, int(imgHeight / 2 * j),imgWidth / 2 * (1+i), imgHeight / 2 * (1+j)) # selects a box to crop
            quart = img.crop(box) # crops the image into one quarter
            section = str(1 + i + j*2) # the section number of the image, 1 = top left, 2 = top right, 3 = bottom left, 4 = bottom right
            quart.save(os.path.join(pkgRoot,"QRCodes",code + "-" + section + '.png'),"PNG") # saves the quarter image
#!/usr/bin/env python2
import sys
import os
import cv2
import numpy as np
import rospkg
from math import *

rospack = rospkg.RosPack() 
pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
file = os.path.join(pkgRoot,"QROfficial","1st2.jpg")
image = cv2.imread(file)
# 	image = cv2.resize(image, (960*2, 540*2))
cv2.imshow("Raw",image)


blackBound1 = (249,210,160)
blackBound2 = (255,255,255)
b1 = cv2.inRange(image, blackBound1, blackBound2)
cv2.imshow("White",b1)


ret, thresh = cv2.threshold(b1, 127, 255, 0)
_, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

min_area = 0
for i in contours:
    a = cv2.contourArea(i)
    if a > min_area:
        maxcnt = i
        min_area = a

boxes = []
rect = cv2.minAreaRect(maxcnt)
width, height = rect[1]

box = cv2.boxPoints(rect)
box = np.int0(box)

a1 = float(height) / width
a2 = float(width) / height
aspect = max(a1,a2)

boxes.append(box)
cv2.drawContours(image, [box], 0, (0,0,255), 2)
deltay = box[1][1] - box[0][1]
deltax = box[1][0] - box[0][0]
print(deltay)
print(deltax)
alpha = atan(float(deltax) / float(deltay))
print(alpha)
cv2.imshow("Conuturs",image)
cv2.waitKey(0)

# rotate it
# crop it
# stitch it
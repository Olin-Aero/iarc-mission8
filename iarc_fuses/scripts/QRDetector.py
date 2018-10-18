#!/usr/bin/env python2
import sys
import os
import cv2
import numpy as np
import rospkg

rospack = rospkg.RosPack() 
pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
file = os.path.join(pkgRoot,"QRCutPics","0145_2_TableFlat.jpg")
image = cv2.imread(file)
imageR = cv2.resize(image, (960, 540))
#imageR = cv2.GaussianBlur(imageR,(1,1),0) 
# cv2.imshow("Image",image)
# cv2.waitKey(0)
blackBound1 = (0,0,0)
blackBound2 = (55,55,55)
b1 = cv2.inRange(imageR, blackBound1, blackBound2)
cv2.imshow("Black",b1)
cv2.waitKey(0)


min_area = 0.001
ret, thresh = cv2.threshold(b1, 127, 255, 0)
_, contours, hierarchy = cv2.findContours(thresh,
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Get bounding box from contours
maxcnt = []
for i in contours:
    a = cv2.contourArea(i)
    if a > min_area:
        maxcnt.append(i)

# Draw coutours on the image
boxes = []
for i in maxcnt:
    rect = cv2.minAreaRect(i)
    width, height = rect[1]

    box = cv2.boxPoints(rect)
    box = np.int0(box)

    a1 = float(height) / width
    a2 = float(width) / height
    aspect = max(a1,a2)

    # if self._aspect_bound[0] < aspect < self._aspect_bound[1]:
    boxes.append(box)
    cv2.drawContours(imageR, [box], 0, (0,0,255), 2)

# left = [-1,-1]
# right = [-1,-1]
# top = [-1,-1]
# bot = [-1,-1]
# for boxxx in boxes:
# 	for vert in boxxx:
# 		if vert[0] < left[0] or left[0] == -1:
# 			left = vert
# 		if vert[0] > right[0] or right[0] == -1:
# 			right = vert
# 		if vert[1] < bot[1] or bot[1] == -1:
# 			bot = vert
# 		if vert[1] > top[1] or top[1] == -1:
# 			top = vert
# bigBox = [left,top,right,bot]
# nbigBox = np.asarray(bigBox)
#cv2.drawContours(imageR,[nbigBox],0,(0,255,0),2)
# cv2.imshow("Box",imageR)
# cv2.waitKey(0)


# I have to convert this to a box
#!/usr/bin/env python2
import sys
import os
import cv2
import numpy as np
import rospkg
from math import *


class Detector():
	def __init__(self):
		rospack = rospkg.RosPack() 
		pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
		images = []
		for index in range(4):
			file = os.path.join(pkgRoot,"QROfficial","%s_2.jpg" %(index+1))
			images.append(cv2.imread(file))
		self.images = images

	def getImages(self):
		images = self.images
		for image in images:
			blackBound1 = (249,210,160)
			blackBound2 = (255,255,255)
			b1 = cv2.inRange(image, blackBound1, blackBound2)


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
			alpha = atan(float(deltax) / float(deltay))
			width,height, _ = image.shape
			diagonal = int(sqrt(width**2 + height**2))
			blank_image = np.zeros((diagonal,diagonal,3), np.uint8)
			x_offset= diagonal/2 - width / 2
			y_offset = diagonal/2 - height / 2
			blank_image[y_offset:y_offset+image.shape[0], x_offset:x_offset+image.shape[1]] = image
			rows,cols,_= blank_image.shape
			M = cv2.getRotationMatrix2D((cols/2,rows/2),degrees(-alpha),1)
			dst = cv2.warpAffine(blank_image,M,(cols,rows))

			blackBound1 = (249,210,160)
			blackBound2 = (255,255,255)
			b1 = cv2.inRange(dst, blackBound1, blackBound2)

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
			# if index == 2:
			crop_img = dst[box[1][1]:box[3][1], box[1][0]:box[3][0]]
			crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
			height, width = crop_img.shape
			if width > height:
				selection1 = crop_img[0:height, 0:height]
				selection2 = crop_img[0:height, width-height:width]
				mean1, _ = cv2.meanStdDev(selection1)
				mean2, _ = cv2.meanStdDev(selection2)
				if mean1 > mean2:
					finalImage = selection1
				else:
					finalImage = selection2
			if height > width:
				selection1 = crop_img[0:width, 0:width]
				selection2 = crop_img[height - width:height, 0:width]
				mean1, _ = cv2.meanStdDev(selection1)
				mean2, _ = cv2.meanStdDev(selection2)
				if mean1 > mean2:
					finalImage = selection1
				else:
					finalImage = selection2
			if height == width:
				finalImage = crop_img

			finalImage = cv2.resize(finalImage, (200, 200))
			images.append(finalImage)
		 

		for i in range(4):	
			cv2.imwrite("FinalImage%d.png" %(i),images[i])
	def run(self):
		self.getImages()
if __name__ == '__main__':
	det = Detector()
	det.run()
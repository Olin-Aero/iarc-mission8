#!/usr/bin/env python2
import rospy
import sys
import os
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from math import *
from cv_bridge import CvBridge, CvBridgeError

class Detector():
	def __init__(self):
		rospy.init_node('QRDetector', anonymous=True)
		self.drone1ImageSub = rospy.Subscriber('/drone1/image_raw',Image,self.drone1CB) # The ros interfacing won't work yet as I haven't figured out how to convert a ROS image to PIL Image
		self.drone2ImageSub = rospy.Subscriber('/drone2/image_raw',Image,self.drone2CB)
		self.drone3ImageSub = rospy.Subscriber('/drone3/image_raw',Image,self.drone3CB)
		self.drone4ImageSub = rospy.Subscriber('/drone4/image_raw',Image,self.drone4CB)
		self.drone1QRpub = rospy.Publisher('/drone1/QRImage', Image, queue_size=10)
		self.drone2QRpub = rospy.Publisher('/drone2/QRImage', Image, queue_size=10)
		self.drone3QRpub = rospy.Publisher('/drone3/QRImage', Image, queue_size=10)
		self.drone4QRpub = rospy.Publisher('/drone4/QRImage', Image, queue_size=10)
		self.bridge = CvBridge()
		rospack = rospkg.RosPack() 
		pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
		images = []
		for index in range(4):
			file = os.path.join(pkgRoot,"QROfficial","%s_2.jpg" %(index+1))
			images.append(cv2.imread(file))
		self.images = images

	def drone1CB(self,imageMessage):
		self.images[0] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")
	def drone2CB(self,imageMessage):
		self.images[1] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")
	def drone3CB(self,imageMessage):
		self.images[2] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")
	def drone4CB(self,imageMessage):
		self.images[3] = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="passthrough")


	def imageToBinary(self,image):
		# inputse an image to get the whites of a digital screen
		whiteLowerBound = (249,210,160) # a high blue value was used because computers tend to make white very blue.
		whiteUpperBound = (255,255,255)
		b1 = cv2.inRange(image, whiteLowerBound, whiteUpperBound) # gets a binary image of the white in the picture
		return b1

	def getBox(self,b1):
		# inputs a binary image to get the largest rectangle in the image
		ret, thresh = cv2.threshold(b1, 127, 255, 0)
		_, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # gets the contours of the image
		maxArea = 0 # initializes the highest area
		for i in contours: # loops through the contours
		    a = cv2.contourArea(i) # gets the area of the contour
		    if a > maxArea: # if the area is greater than the past maximum area:
		        maxCnt = i # make the new contour the maximum area contour
		        maxArea = a # and make its area the maximum area
		rect = cv2.minAreaRect(maxCnt) # gets a rectangular bounding box
		box = cv2.boxPoints(rect)
		box = np.int0(box) # makes it into an array of points
		return box

	def getRotatedImage(self,image,box):
		# inputs an image and the box around the qr quadrant to get a rotated version of the image corresponding to the box
		deltay = box[1][1] - box[0][1] # gets the change in the y axis
		deltax = box[1][0] - box[0][0] # gets the change in the x axis
		alpha = atan(float(deltax) / float(deltay)) # determines the angle from horizontal of the box
		width,height, _ = image.shape # gets the width and height of the image
		diagonal = int(sqrt(width**2 + height**2)) # gets the diagonal so that no information is lost when rotating the image
		blankImage = np.zeros((diagonal,diagonal,3), np.uint8) # initializes a blank image with height and width equal to the diagonal of the original image
		x_offset = diagonal / 2 - width / 2 # the offset of the image, making the center of the image the same as the center of the blank image
		y_offset = diagonal / 2 - height / 2
		blankImage[y_offset:y_offset+image.shape[0], x_offset:x_offset+image.shape[1]] = image # adds the image to the blank image
		rows,cols,_= blankImage.shape # gets the numer of rows and columns of the bigger image
		M = cv2.getRotationMatrix2D((cols/2,rows/2),degrees(-alpha),1) # makes a rotation matrix using -alpha to get the box to be parellel to the x axis
		rotatedImage = cv2.warpAffine(blankImage,M,(cols,rows)) # multipies the rotation matrix by the image to get the final rotated image
		return rotatedImage

	def getCroppedImage(self,rotatedImage,box):
		# inputs a rotated image and box containing a qr quadrant and returns a 200 by 200 image containing the qr quadrant
		crop_img = rotatedImage[box[1][1]:box[3][1], box[1][0]:box[3][0]] # crops the image by the box
		crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY) #  converts the box to grayscale to easily calculate the mean
		height, width = crop_img.shape # width and height of the image
		if width > height: # if the width is greater than the height, it needs to be cut in one direction
			selection1 = crop_img[0:height, 0:height] # first selection is the left square
			selection2 = crop_img[0:height, width-height:width] # second selection is the right square
			mean1, _ = cv2.meanStdDev(selection1) # calculates the mean brightness of each selection
			mean2, _ = cv2.meanStdDev(selection2)
			if mean1 > mean2: # whichever selection is brighter on average becomes the final image
				finalImage = selection1
			else:
				finalImage = selection2
		if height > width: # same as above but in the case that the height needs to be cut
			selection1 = crop_img[0:width, 0:width]
			selection2 = crop_img[height - width:height, 0:width]
			mean1, _ = cv2.meanStdDev(selection1)
			mean2, _ = cv2.meanStdDev(selection2)
			if mean1 > mean2:
				finalImage = selection1
			else:
				finalImage = selection2
		if height == width: # in this case, the image is already square
			finalImage = crop_img
		finalImage = cv2.resize(finalImage, (200, 200)) # resizes the square image to 200 by 200 to make later processing easier
		return finalImage

	def getImages(self):
		# saves the images so that QRCombiner can access them.
		images = self.images
		finalImages = []
		for image in images:
			b1 = self.imageToBinary(image) # gets the white part of the image
			box = self.getBox(b1) # gets a rectangle around the box
			rotatedImage = self.getRotatedImage(image,box) # rotates the image so that the rectangle is flat
			b1Rotated = self.imageToBinary(rotatedImage) # converts the rotated image to a binary based on white
			boxRotated = self.getBox(b1Rotated) # uses this new image to create a box that is flat with the x-axis
			croppedImage = self.getCroppedImage(rotatedImage,boxRotated) # crops the image around the qr quadrant
			finalImages.append(croppedImage) # adds the image to the collection of final images
		 

		for i in range(4):	
			cv2.imwrite("FinalImage%d.png" %(i),finalImages[i]) # right now it just saves the file and the combiner can access the files.
			# Although in the future it would be great to have it interface with ros.

		imageToBeSent = self.bridge.cv2_to_imgmsg(finalImages[0], encoding="passthrough")
		self.drone1QRpub.publish(imageToBeSent)
		imageToBeSent = self.bridge.cv2_to_imgmsg(finalImages[1], encoding="passthrough")
		self.drone2QRpub.publish(imageToBeSent)
		imageToBeSent = self.bridge.cv2_to_imgmsg(finalImages[2], encoding="passthrough")
		self.drone3QRpub.publish(imageToBeSent)
		imageToBeSent = self.bridge.cv2_to_imgmsg(finalImages[3], encoding="passthrough")
		self.drone4QRpub.publish(imageToBeSent)
		

	def run(self):
		self.getImages()
if __name__ == '__main__':
	det = Detector()
	det.run()
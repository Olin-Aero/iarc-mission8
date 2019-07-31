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

class QRDetector():
    def __init__(self):
        pass

    def imageToBinary(self,image):
        # inputse an image to get the whites of a digital screen
        # whiteLowerBound = (249,210,160) # a high blue value was used because computers tend to make white very blue.
        # whiteUpperBound = (255,255,255) # IMPORTANT NOTE: I set it to (220,210,160) for paper. Testing it on ipads, it should be set to (249,210,160)!!
        whiteLowerBound = (128,128,128)
        whiteUpperBound = (255,255,255)
        b1 = cv2.inRange(image, whiteLowerBound, whiteUpperBound) # gets a binary image of the white in the picture

        return b1

    def getBox(self,b1,image):
        # inputs a binary image to get the largest rectangle in the image
        img = image.copy()
        ret, thresh = cv2.threshold(b1, 127, 255, 0)
        _, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # gets the contours of the image
        maxArea = 0 # initializes the highest area
        maxCnt = None
        for i in contours: # loops through the contours
            a = cv2.contourArea(i) # gets the area of the contour
            if a > maxArea: # if the area is greater than the past maximum area:
                maxCnt = i # make the new contour the maximum area contour
                maxArea = a # and make its area the maximum area
        if maxCnt is None:
            rospy.logwarn("No contours found in QR image")
            return None
        rect = cv2.minAreaRect(maxCnt) # gets a rectangular bounding box
        box = cv2.boxPoints(rect)
        box = np.int0(box) # makes it into an array of points
        cv2.drawContours(img, [box], 0, (0,255,0), 3)
        return box

    def getRotatedImage(self,image,box):
        # inputs an image and the box around the qr quadrant to get a rotated version of the image corresponding to the box
        deltay = box[1][1] - box[0][1] # gets the change in the y axis
        deltax = box[1][0] - box[0][0] # gets the change in the x axis
        if(deltay == 0):
            alpha = pi / 2
        else:
            alpha = atan(float(deltax) / float(deltay)) # determines the angle from horizontal of the box
        if(alpha > pi/4):
            alpha = alpha - pi/2
        width,height, _ = image.shape # gets the width and height of the image
        diagonal = int(sqrt(width**2 + height**2)) # gets the diagonal so that no information is lost when rotating the image
        blankImage = np.zeros((diagonal,diagonal,3), np.uint8) # initializes a blank image with height and width equal to the diagonal of the original image
        x_offset = diagonal / 2 - width / 2 # the offset of the image, making the center of the image the same as the center of the blank image
        y_offset = diagonal / 2 - height / 2
        blankImage[x_offset:x_offset+image.shape[0], y_offset:y_offset+image.shape[1]] = image # adds the image to the blank image
        rows,cols,_= blankImage.shape # gets the numer of rows and columns of the bigger image
        M = cv2.getRotationMatrix2D((cols/2,rows/2),degrees(-alpha),1) # makes a rotation matrix using -alpha to get the box to be parellel to the x axis
        rotatedImage = cv2.warpAffine(blankImage,M,(cols,rows)) # multipies the rotation matrix by the image to get the final rotated image
        return rotatedImage

    def getCroppedImage(self,rotatedImage,box):
        # inputs a rotated image and box containing a qr quadrant and returns a 200 by 200 image containing the qr quadrant
        if(box[3][1] < box[1][1]):
            crop_img = rotatedImage[box[3][1]:box[1][1], box[1][0]:box[3][0]] # crops the image by the box
        else:
            crop_img = rotatedImage[box[1][1]:box[3][1], box[1][0]:box[3][0]] # crops the image by the box
        bw_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY) #  converts the box to grayscale to easily calculate the mean
        height, width, _ = crop_img.shape # width and height of the image
        if width > height: # if the width is greater than the height, it needs to be cut in one direction
            selection1C = crop_img[0:height, 0:height] # first selection is the left square
            selection2C = crop_img[0:height, width-height:width] # second selection is the right square
            selection1B = bw_img[0:height, 0:height] # first selection is the left square
            selection2B = bw_img[0:height, width-height:width] # second selection is the right square
            mean1, _ = cv2.meanStdDev(selection1B) # calculates the mean brightness of each selection
            mean2, _ = cv2.meanStdDev(selection2B)
            if mean1 > mean2: # whichever selection is brighter on average becomes the final image
                    finalImage = selection1C
            else:
                    finalImage = selection2C
        if height > width: # same as above but in the case that the height needs to be cut
            selection1C = crop_img[0:width, 0:width]
            selection2C = crop_img[height - width:height, 0:width]
            selection1B = bw_img[0:width, 0:width]
            selection2B = bw_img[height - width:height, 0:width]
            mean1, _ = cv2.meanStdDev(selection1B)
            mean2, _ = cv2.meanStdDev(selection2B)
            if mean1 > mean2:
                    finalImage = selection1C
            else:
                    finalImage = selection2C
        if height == width: # in this case, the image is already square
                finalImage = crop_img
        finalImage = cv2.resize(finalImage, (200, 200)) # resizes the square image to 200 by 200 to make later processing easier
        # cv2.imshow("next",finalImage)
        # cv2.waitKey(1)
        return finalImage

    def __call__(self, image):
        # saves the images so that QRCombiner can access them.
        cv2.imwrite( 'Image.jpg', image )
        b1 = self.imageToBinary(image) # gets the white part of the image
        # cv2.imshow('b1', image)
        # cv2.waitKey(1)
        box = self.getBox(b1,image) # gets a rectangle around the box
        if box is None:
            return None
        rotatedImage = self.getRotatedImage(image,box) # rotates the image so that the rectangle is flat
        b1Rotated = self.imageToBinary(rotatedImage) # converts the rotated image to a binary based on white
        boxRotated = self.getBox(b1Rotated,rotatedImage) # uses this new image to create a box that is flat with the x-axis
        croppedImage = self.getCroppedImage(rotatedImage,boxRotated) # crops the image around the qr quadrant
        return croppedImage

def main():
    rospack = rospkg.RosPack() 
    pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
    images = []
    # images here are a placeholder until real detection callback exists
    for index in range(4):
        file = os.path.join(pkgRoot,"images", "qr%s.png" %(index+1))
        images.append(cv2.imread(file))

    detector = QRDetector()

    cropped_images = [detector(image) for image in images]
    # show image ...
    for i, img in enumerate(cropped_images):
        cv2.imshow('image-{}'.format(i), img)
        cv2.imwrite('/tmp/FinalImage{}.png'.format(i),img)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()

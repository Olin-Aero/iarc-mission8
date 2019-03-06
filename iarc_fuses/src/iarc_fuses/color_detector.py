# Interactive threshold setter uses much code from
# https://github.com/opencv/opencv/blob/3.4/samples/python/tutorial_code/imgProc/threshold_inRange/threshold_inRange.py


import numpy as np
import cv2

class Color_detector():
	def __init__(self):
		self.color_space = None # Set color_space using set_detection_color 

	def set_detection_color(self, lower_threshold, upper_threshold, color_space="HSV"):
		self.color_space = color_space
		self.color_lower = np.array(lower_threshold, np.uint8)
		self.color_upper = np.array(upper_threshold, np.uint8)

	def detect_color(image, bounding_box=None):
		if self.color_space == "HSV":
			image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
			color = cv2.inRange(image, self.color_lower, self.color_upper)

		elif self.color_space == "BGR":
			# TODO: Write BGR detector
			pass # Image is expected to be in BGR space (default for opencv)
		elif self.color_space == "LAB":
			# TODO: Write LAB detector
			image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
		else:
			raise Exception("color_space \"{}\" is not a valid color space." \
				"Select from supported types".format(color_space))

		#Morphological transformation, Dilation        
		kernal = np.ones((5 ,5), "uint8")
		blue=cv2.dilate(yellow, kernal)
		res=cv2.bitwise_and(img, img, mask = yellow)

		



class Interactive_Threshold_Setter():
	def __init__(self):
		self.max_value = 255
		self.max_value_H = 360//2
		self.low_H = 0
		self.low_S = 0
		self.low_V = 0
		self.high_H = self.max_value_H
		self.high_S = self.max_value
		self.high_V = self.max_value
		self.window_capture_name = 'Video Capture'
		self.window_detection_name = 'Object Detection'
		self.low_H_name = 'Low H'
		self.low_S_name = 'Low S'
		self.low_V_name = 'Low V'
		self.high_H_name = 'High H'
		self.high_S_name = 'High S'
		self.high_V_name = 'High V'
		self.create_window()
		self.create_trackbars()
		

	def set_capture(self, cap=None):
		if cap==None:
			self.cap = cv2.VideoCapture(0)
		else:
			self.cap = cap

	def create_window(self):
		cv2.namedWindow(self.window_capture_name)
		cv2.namedWindow(self.window_detection_name)

	def create_trackbars(self):
		cv2.createTrackbar(self.low_H_name,  self.window_detection_name , 
			self.low_H,  self.max_value_H, self.on_low_H_thresh_trackbar)
		cv2.createTrackbar(self.high_H_name, self.window_detection_name , 
			self.high_H, self.max_value_H, self.on_high_H_thresh_trackbar)
		cv2.createTrackbar(self.low_S_name,  self.window_detection_name , 
			self.low_S,  self.max_value,   self.on_low_S_thresh_trackbar)
		cv2.createTrackbar(self.high_S_name, self.window_detection_name , 
			self.high_S, self.max_value,   self.on_high_S_thresh_trackbar)
		cv2.createTrackbar(self.low_V_name,  self.window_detection_name , 
			self.low_V,  self.max_value,   self.on_low_V_thresh_trackbar)
		cv2.createTrackbar(self.high_V_name, self.window_detection_name , 
			self.high_V, self.max_value,   self.on_high_V_thresh_trackbar)

	def on_low_H_thresh_trackbar(self, val):
	    self.low_H = val
	    self.low_H = min(self.high_H-1, self.low_H)
	    cv2.setTrackbarPos(self.low_H_name, self.window_detection_name, self.low_H)

	def on_high_H_thresh_trackbar(self, val):
	    self.high_H = val
	    self.high_H = max(self.high_H, self.low_H+1)
	    cv2.setTrackbarPos(self.high_H_name, self.window_detection_name, self.high_H)

	def on_low_S_thresh_trackbar(self, val):
	    self.low_S = val
	    self.low_S = min(self.high_S-1, self.low_S)
	    cv2.setTrackbarPos(self.low_S_name, self.window_detection_name, self.low_S)

	def on_high_S_thresh_trackbar(self, val):
	    self.high_S = val
	    self.high_S = max(self.high_S, self.low_S+1)
	    cv2.setTrackbarPos(self.high_S_name, self.window_detection_name, self.high_S)

	def on_low_V_thresh_trackbar(self, val):
	    self.low_V = val
	    self.low_V = min(self.high_V-1, self.low_V)
	    cv2.setTrackbarPos(self.low_V_name, self.window_detection_name, self.low_V)

	def on_high_V_thresh_trackbar(self, val):
	    self.high_V = val
	    self.high_V = max(self.high_V, self.low_V+1)
	    cv2.setTrackbarPos(self.high_V_name, self.window_detection_name, self.high_V)



def main():
	mySess = Interactive_Threshold_Setter()
	mySess.set_capture()
	while :
		pass

if __name__ == '__main__':
	main()
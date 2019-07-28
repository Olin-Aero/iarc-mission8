#!/usr/bin/python2

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QIcon, QPixmap
 
import ntpath
import pickle
import glob, cv2, torch
from PIL import Image
import numpy as np
from os.path import realpath, dirname, join
import os
from net import SiamRPNvot
from run_SiamRPN import SiamRPN_init, SiamRPN_track
from utils import get_axis_aligned_bbox, cxy_wh_2_rect, rect_2_cxy_wh
from tkinter import *      

# from tkinter import Tk
from tkinter.filedialog import askopenfilename

# a hack until schommer's ROS works
import sys
sys.path.append('../')
from box_utils import BoxUtils

class Annotation():
	def __init__(self, file_reference="", frame_num=None, bounding_rect=[], redo=False):
		self.file_reference = file_reference
		self.frame_num = frame_num
		self.bounding_rect = bounding_rect
		self.redo = redo

	def set_label(self, file_reference="", frame_num=None, bounding_rect=[], redo=False):
		self.file_reference = file_reference
		self.frame_num = frame_num
		self.bounding_rect = bounding_rect
		self.redo = redo



class Video_Labeler():

	def __init__(self, vid_file=False, scale=.6, start_frame_num=0):
		self.labels = []
		self.scale = scale
		self.start_frame_num = start_frame_num
		self.frame_num = start_frame_num

		self.modes = ["ColorCut", "AdaptiveThreshold", "GrabCut"]
		self.mode_idx = 0
		self.mode = self.modes[self.mode_idx]
		self.use_grab_cut = False
		self.run_video = False
		self.vid_done = False
		self.redo_annotation = False
		self.root = Tk()      
		self.canvas = Canvas(self.root, width = 600, height = 600)      
		self.canvas.pack()      

		self.rect_pad = .1 # Amount to pad the DaSaiam rectangle for grabcut (relative units)
		self.threshold_content_limit = 90

		if not vid_file:
			self.vid_file = self.select_video()
		else:
			self.vid_file = vid_file
		
		self.disp_name = 'SiamRPN'
		self.create_window()
		
		self.down_point = None
		self.up_point = None
		self.sel_rect = None
		self.in_click = False
		self.x_cursor = None
		self.y_cursor = None
		cv2.setMouseCallback(self.disp_name, self.on_mouse)
		self.cap = cv2.VideoCapture(self.vid_file)
		self.init_rbox = None

		# load net
		self.net = SiamRPNvot()
		self.net.load_state_dict(torch.load(join(realpath(dirname(__file__)), 'SiamRPNVOT.model')))
		self.net.eval().cuda()

	def create_window(self):
		cv2.namedWindow(self.disp_name)


	def select_video(self):
		Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
		filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
		return filename

	def adaptive_cut(self,img,rect):
		img_crop = img[rect[1]:rect[1]+rect[3], 
						rect[0]:rect[0]+rect[2]]
		img_gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
		# ret, img_thresh =  cv2.threshold(img_gray,127,255,cv2.THRESH_BINARY_INV)
		frame_threshold = cv2.adaptiveThreshold(img_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,2)
		cv2.imshow("thresh", frame_threshold)
		rect_2 = self.get_bounding_box(frame_threshold)

		return [rect[0]+rect_2[0], rect[1] + rect_2[1], rect_2[2], rect_2[3]]

	def color_cut(self, img, rect):
		frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		hsv_cropped = frame_HSV[rect[1]:rect[1]+rect[3], 
								rect[0]:rect[0]+rect[2]]

		high_b =  np.asarray((180, 255, 120))
		low_b =    np.asarray((0, 0, 0))
		frame_threshold = cv2.inRange(hsv_cropped, low_b, high_b)
		cv2.imshow("thresh", frame_threshold)
		rect_2 = self.get_bounding_box(frame_threshold)
		return [rect[0]+rect_2[0], rect[1] + rect_2[1], rect_2[2], rect_2[3]]

	def edge_cut(self, img, rect):
		edges = cv2.Canny(img,100,200)


	def grab_cut(self, img, rect):
		mask = np.zeros(img.shape[:2],np.uint8)
		bgdModel = np.zeros((1,65),np.float64)
		fgdModel = np.zeros((1,65),np.float64)

		cv2.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
		frame_threshold = 255*np.where((mask==2)|(mask==0),0,1).astype('uint8')
		kernel = np.ones((5,5),np.uint8)
		
		frame_threshold = cv2.erode(frame_threshold,kernel,iterations = 2)

		cv2.imshow("thresh", frame_threshold[rect[1]:rect[1]+rect[3], 
								rect[0]:rect[0]+rect[2]])
		img = img*frame_threshold[:,:,np.newaxis]

		im,contours,hierarchy = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) != 0:
			contours_sorted = sorted(contours, key = self.hull_convexity_ratio)
			c = contours_sorted[0]
			x,y,w,h = cv2.boundingRect(c)
			return [x,y,w,h]
		# img += 255*(1 - mask2[:,:,np.newaxis])
		# return self.get_bounding_box(frame_threshold)

	def hull_convexity_ratio(self, cnt):
		return cv2.contourArea(cv2.convexHull(cnt))/cv2.contourArea(cnt)

	def get_bounding_box(self, mask):
		im,contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) != 0:
			#find the biggest area
			c = max(contours, key = cv2.contourArea)
			x,y,w,h = cv2.boundingRect(c)
			return [x,y,w,h]

	def check_rect(self, rect, im_size):
		# clip xywh-style rectangle to fit within image
		w, h = im_size

		rect = BoxUtils.convert(rect,
			BoxUtils.FMT_XYWH,
			BoxUtils.FMT_XYXY
			)

		rect[0] = np.clip(rect[0], 0, w)
		rect[1] = np.clip(rect[1], 0, h)
		rect[2] = np.clip(rect[2], 0, w)
		rect[3] = np.clip(rect[3], 0, h)

		rect = BoxUtils.convert(rect,
			BoxUtils.FMT_XYXY,
			BoxUtils.FMT_XYWH
			)

		return rect

	def optimize_rect(self, rect_xywh, im):
		height, width, channels = im.shape
		rect_xywh = np.int32(np.hstack([rect_xywh[0] - width*self.rect_pad / 2., rect_xywh[1] - height * self.rect_pad / 2., rect_xywh[2]  + 2*width*self.rect_pad , rect_xywh[3]  + 2*height*self.rect_pad]))
		rect_xywh = self.check_rect(rect_xywh, [width, height])
		if self.mode == "GrabCut":
			rect_xywh = self.grab_cut(im, tuple(np.int32(rect_xywh)))
		elif self.mode == "ColorCut":
			rect_xywh = self.color_cut(im, np.int32(rect_xywh)) # Perform color cut
		elif self.mode == "AdaptiveThreshold":
			rect_xywh = self.adaptive_cut(im, np.int32(rect_xywh))
		rect_xywh = self.check_rect(rect_xywh, (width, height))
		rect_xywh = np.int32(rect_xywh)
		return rect_xywh

	def save(self):
		print("File saved as: " + "{}_labels.p".format(os.path.splitext(ntpath.basename(self.vid_file))[0]))
		pickle.dump(self.labels, open("{}_labels.p".format(os.path.splitext(ntpath.basename(self.vid_file))[0]), "wb" ) )

	def start_labeling(self):
		self.cap.set(1,self.frame_num)
		
		ret, im = self.cap.read()
		im = cv2.resize(im,None, fx=self.scale, fy=self.scale)
		height, width, channels = im.shape
		# # image and init box
		self.draw_bbox()
		sel_rect_wh = (
			self.sel_rect[0][0], self.sel_rect[0][1],
			self.sel_rect[1][0]- self.sel_rect[0][0],
			self.sel_rect[1][1] - self.sel_rect[0][1])
		
		rect_gc = self.color_cut(im, sel_rect_wh)
		# tracker init
		target_pos, target_sz = np.array(rect_gc[0:2]) + np.array(rect_gc[2:4])/2, np.array(rect_gc[2:4])

		state = SiamRPN_init(im, target_pos, target_sz, self.net, use_gpu=True)

		state_hist = [state]
		toc = 0
		old_frame_num = self.frame_num - 1
		# self.frame_num = 0

		while(True):
			if old_frame_num != self.frame_num:
				old_frame_num = self.frame_num
				self.cap.set(1,self.frame_num)
				# Capture frame-by-frame
				ret, im = self.cap.read()
				if not ret:
					self.save()
					break
				im = cv2.resize(im,None, fx=self.scale, fy=self.scale)
				if len(self.labels) <= self.frame_num - self.start_frame_num:
					state_hist.append(SiamRPN_track(state_hist[-1], im, use_gpu=True))  # track)
					state = state_hist[-1]
					rect_ccwh = np.concatenate([state['target_pos'], state['target_sz']])
					rect_xywh = BoxUtils.convert(rect_ccwh,
						BoxUtils.FMT_CCWH,
						BoxUtils.FMT_XYWH
						)

					rect_xywh = self.optimize_rect(rect_xywh, im)
					rect_xyxy = BoxUtils.convert(rect_xywh,
					BoxUtils.FMT_XYWH,
					BoxUtils.FMT_XYXY
					)
					
				else:
					state = state_hist[self.frame_num - self.start_frame_num] # Use previous frame state
					rect_xyxy = BoxUtils.unnormalize(self.labels[self.frame_num - self.start_frame_num].bounding_rect, BoxUtils.FMT_XYXY, [height, width])
					
				
				pt0, pt1 = tuple(np.asarray(rect_xyxy[:2], np.int32)), tuple(np.asarray(rect_xyxy[2:], np.int32))


				cv2.rectangle(im, pt0, pt1, (0, 255, 255), 3)

			k =  cv2.waitKey(33)

			if k & 0xFF == ord('d'):
				# Drawing Mode
				self.draw_bbox()
				rect_xyxy = [self.sel_rect[0][0],self.sel_rect[0][1], self.sel_rect[1][0], self.sel_rect[1][1]]
				# drawn_norm = 
				rect_xywh = BoxUtils.convert(
					rect_xyxy,
					BoxUtils.FMT_XYXY,
					BoxUtils.FMT_XYWH)
				# rect_xywh = self.optimize_rect(rect_xywh, im)
				# # self.labels[self.frame_num].bounding_rect = BoxUtils.normalize(rect_xywh, BoxUtils.FMT_XYWH, [height, width])
				# rect_xyxy = BoxUtils.convert(
				# 	rect_xywh,
				# 	BoxUtils.FMT_XYWH,
				# 	BoxUtils.FMT_XYXY)
				rect_ccwh = BoxUtils.convert(rect_xywh,
					BoxUtils.FMT_XYWH,
					BoxUtils.FMT_CCWH
					)

				state['target_pos'] = rect_ccwh[:2]
				state['target_sz']  = rect_ccwh[2:]
				old_frame_num = self.frame_num - 1


			if k & 0xFF == ord('r'):
				self.redo_annotation = not self.redo_annotation
				print("Annotation redo set to: ", self.redo_annotation)
			# If a new frame is being annotated, add it
			if len(self.labels) <= self.frame_num - self.start_frame_num:
				print("Normalized rect: ", BoxUtils.normalize(rect_xyxy, BoxUtils.FMT_XYXY, [height, width]))
				self.labels.append(Annotation(self.vid_file, self.frame_num, BoxUtils.normalize(rect_xyxy, BoxUtils.FMT_XYXY, [height, width]), self.redo_annotation))
				self.redo_annotation = False
			else: # Check if the cache should be used
				if k & 0xFF == ord('d'):
					self.labels[self.frame_num] = Annotation(self.vid_file, self.frame_num, BoxUtils.normalize(rect_xyxy, BoxUtils.FMT_XYXY, [height, width]), self.redo_annotation)
					self.redo_annotation = False
			cv2.imshow(self.disp_name, im)

			if k & 0xFF == ord('j'):
				if self.frame_num > 0:
					self.frame_num = self.frame_num - 1

			if k & 0xFF == ord('m'):
				self.mode_idx = self.mode_idx + 1
				self.mode = self.modes[self.mode_idx%len(self.modes)]
				print("Toggeling mode: ", self.mode)
				

			if k & 0xFF == ord('v'):
				print("Toggeling Video Mode: ", not self.run_video)
				self.run_video = not self.run_video

			if k & 0xFF == ord('k') or self.run_video:
				self.frame_num = self.frame_num + 1

			if k & 0xFF == ord('s') :
				self.save()

			if k & 0xFF == ord('q'):
				break

		# When everything done, release the capture
		self.cap.release()
		cv2.destroyAllWindows()


	def draw_bbox(self):
		# preview
		self.sel_rect = None
		self.cap.set(1,self.frame_num)
		ret, im = self.cap.read()
		im = cv2.resize(im,None, fx=self.scale, fy=self.scale)
		height, width, channels = im.shape
		while(True):
			
			frame_draw = np.copy(im)
			if self.sel_rect != None:
				cv2.rectangle(frame_draw, tuple(self.sel_rect[0]), tuple(self.sel_rect[1]), (255, 255, 255))
			if self.x_cursor is not None:
				cv2.line(frame_draw, (self.x_cursor, 0), (self.x_cursor, height), (0,0,255))
				cv2.line(frame_draw, (0, self.y_cursor), (width, self.y_cursor), (0,0,255))
			cv2.imshow(self.disp_name, frame_draw)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		# return [334.02,128.36,438.19,188.78,396.39,260.83,292.23,200.41]

	def on_mouse(self, event, x, y, flags, param):
		self.x_cursor = x
		self.y_cursor = y

		if event == cv2.EVENT_LBUTTONDOWN:
			self.in_click = True
			self.down_point = [x, y]            

		if event == cv2.EVENT_LBUTTONUP:
			self.in_click = False
			self.up_point = [x, y]
			if self.up_point == self.down_point:
				self.sel_rect = None
			else:
				for i in range(0,2):
					if self.down_point[i] > self.up_point[i]:
						self.down_point[i], self.up_point[i] = self.up_point[i], self.down_point[i]
			if self.down_point != None:
				self.sel_rect = (self.down_point, self.up_point)
		

		if self.in_click:

			self.sel_rect = (self.down_point, (x, y))


def main():
	vid_file_1 = os.path.expanduser("~/Videos/VID_20190327_195111.mp4")
	vid_file_2 = os.path.expanduser("~/Videos/VID_20190327_194904.mp4")
	# vid_file_3 = os.path.expanduser("~/Videos/VID_20190327_195218.mp4")
	
	labeler = Video_Labeler(vid_file_2, start_frame_num=0)
	
	labeler.start_labeling()


	# labels = pickle.load( open( "VID_20190327_195111_labels.p", "rb" ) )
	# cap =  cv2.VideoCapture(vid_file_1)

	# frame_num = 0
	
	

	# while True:
	# 	cap.set(1,frame_num)
	# 	ret, im = cap.read()
	# 	res = labels[frame_num].bounding_rect
	# 	cv2.rectangle(im, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 3)

	# 	cv2.imshow("Recorded Annotations", im)
	# 	frame_num = frame_num + 1
	# 	cv2.waitKey(33)

if __name__ == "__main__":
	main()

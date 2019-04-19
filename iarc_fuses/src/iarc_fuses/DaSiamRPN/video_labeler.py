# --------------------------------------------------------
# DaSiamRPN
# Licensed under The MIT License
# Written by Qiang Wang (wangqiang2015 at ia.ac.cn)
# --------------------------------------------------------
#!/usr/bin/python
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

class Annotation():
	def __init__(self, file_reference="", frame_num=None, bounding_rect=[]):
		self.file_reference = file_reference
		self.frame_num = frame_num
		self.bounding_rect = bounding_rect
	
	def set_label(self, file_reference="", frame_num=None, bounding_rect=[]):
		self.file_reference = file_reference
		self.frame_num = frame_num
		self.bounding_rect = bounding_rect



class Video_Labeler():

	def __init__(self, vid_file=False, scale=.6):
		self.labels = []
		self.scale = scale
		self.frame_num = 0

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

	def color_cut(self, img, rect):
		frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		hsv_cropped = frame_HSV[rect[1]:rect[1]+rect[3], 
								rect[0]:rect[0]+rect[2]]

		# low_b =  np.percentile(hsv_cropped, 100 - self.threshold_content_limit, axis=(0, 1))
		# high_b = np.percentile(hsv_cropped, self.threshold_content_limit, axis=(0, 1))

		high_b =  np.asarray((180, 255, 85))
		low_b =    np.asarray((0, 0, 0))
		frame_threshold = cv2.inRange(hsv_cropped, low_b, high_b)
		
		rect_2 = self.get_bounding_box(frame_threshold)
		# cv2.imshow("Color Cut", frame_threshold)
		# cv2.waitKey(0)
		return [rect[0]+rect_2[0], rect[1] + rect_2[1], rect_2[2], rect_2[3]]



	def grab_cut(self, img, rect):
		mask = np.zeros(img.shape[:2],np.uint8)
		bgdModel = np.zeros((1,65),np.float64)
		fgdModel = np.zeros((1,65),np.float64)
		# rect = (self.sel_rect[0][0],self.sel_rect[0][1],self.sel_rect[1][0]- self.sel_rect[0][0],self.sel_rect[1][1] - self.sel_rect[0][1])

		cv2.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
		mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
		img = img*mask2[:,:,np.newaxis]
		# img += 255*(1 - mask2[:,:,np.newaxis])
		return self.get_bounding_box(mask2)


	def get_bounding_box(self, mask):
		im,contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) != 0:

			#find the biggest area
			c = max(contours, key = cv2.contourArea)
			# draw in blue the biggest contour
			# cv2.drawContours(img, [c], -1, 255, 3)
			x,y,w,h = cv2.boundingRect(c)

			return [x,y,w,h]

	def check_rect(self, rect, im_size):
		if rect[0] < 0:
			rect[0] = 0
		if rect[1] < 0:
			rect[1] = 0
		if rect[0] + rect[2] > im_size[0]:
			rect[2] = im_size[0] - rect[0]
		if rect[1] + rect[3] > im_size[1]:
			rect[3] = im_size[1] - rect[1]

		return rect

	def start_labeling(self):
		self.cap.set(1,self.frame_num)
		# self.cap.set(1,900)
		ret, im = self.cap.read()
		im = cv2.resize(im,None, fx=self.scale, fy=self.scale)
		height, width, channels = im.shape
		# # image and init box
		self.draw_bbox()
		sel_rect_wh = (self.sel_rect[0][0],self.sel_rect[0][1],
			self.sel_rect[1][0]- self.sel_rect[0][0], self.sel_rect[1][1] - self.sel_rect[0][1])
		rect_gc = self.color_cut(im, sel_rect_wh)
		# tracker init
		target_pos, target_sz = np.array(rect_gc[0:2]) + np.array(rect_gc[2:4])/2, np.array(rect_gc[2:4])

		state = SiamRPN_init(im, target_pos, target_sz, self.net, use_gpu=True)

		state_hist = [state]
		toc = 0
		old_frame_num = -1
		self.frame_num = 0

		while(True):
			print("State Hist Len: ", len(state_hist))
			print("Frame Num: ", self.frame_num)
			# qqprint(old_frame_num, self.frame_num)
			if old_frame_num != self.frame_num:
				old_frame_num = self.frame_num
				self.cap.set(1,self.frame_num)
				# Capture frame-by-frame
				ret, im = self.cap.read()
				im = cv2.resize(im,None, fx=self.scale, fy=self.scale)


				if len(self.labels) <= self.frame_num:
					print("Adding state!")
					state_hist.append(SiamRPN_track(state_hist[-1], im, use_gpu=True))  # track)
					state = state_hist[-1]
					res = cxy_wh_2_rect(state['target_pos'], state['target_sz'])
				else:
					print("")
					state = state_hist[self.frame_num] # Use previous frame state
					res = self.labels[self.frame_num].bounding_rect
				res = [res[0]- width*self.rect_pad/2, res[1]- height*self.rect_pad/2, res[2] + width*self.rect_pad, res[3] + height*self.rect_pad]
				res = self.check_rect(res, (width, height))
				res = [int(l) for l in res]
				# res = self.grab_cut(im, tuple(res))
				# res = self.color_cut(im, tuple(res))
				cv2.rectangle(im, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 3)




			k =  cv2.waitKey(33)

			if k & 0xFF == ord('d'):
				self.draw_bbox() 
				self.labels[self.frame_num].bounding_rect = (self.sel_rect[0][0],self.sel_rect[0][1],self.sel_rect[1][0]- self.sel_rect[0][0],self.sel_rect[1][1] - self.sel_rect[0][1])
				b_bx = rect_2_cxy_wh([self.sel_rect[0][0],self.sel_rect[0][1], self.sel_rect[1][0], self.sel_rect[1][1]] )
				state['target_pos'] = b_bx[0]
				state['target_sz']  = b_bx[1]
				old_frame_num = self.frame_num-1


			# If a new frame is being annotated, add it
			if len(self.labels) <= self.frame_num:
				self.labels.append(Annotation(self.vid_file, self.frame_num, res))
			else: # Check if the cache should be used
				if k & 0xFF == ord('d'):
					self.labels[self.frame_num] = Annotation(self.vid_file, self.frame_num, res)

			cv2.imshow(self.disp_name, im)

			if k & 0xFF == ord('j'):
				if self.frame_num > 0:
					self.frame_num = self.frame_num - 1

			if k & 0xFF == ord('k'):
				self.frame_num = self.frame_num + 1

			if k & 0xFF == ord('s'):
				pickle.dump(self.labels, open("{}_labels.p".format(os.path.splitext(ntpath.basename(self.vid_file))[0]), "wb" ) )

			if k & 0xFF == ord('q'):
				break

		# When everything done, release the capture
		self.cap.release()
		cv2.destroyAllWindows()


	def draw_bbox(self):
		self.sel_rect = None
		self.cap.set(1,self.frame_num)
		ret, im = self.cap.read()
		im = cv2.resize(im,None, fx=self.scale, fy=self.scale)
		while(True):
			
			frame_draw = np.copy(im)
			if self.sel_rect != None:
				cv2.rectangle(frame_draw, tuple(self.sel_rect[0]), tuple(self.sel_rect[1]), (255, 255, 255))
			
			cv2.imshow(self.disp_name, frame_draw)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		# return [334.02,128.36,438.19,188.78,396.39,260.83,292.23,200.41]

	def on_mouse(self, event, x, y, flags, param):

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
	# vid_file_2 = os.path.expanduser("~/Videos/VID_20190327_194904.mp4")
	# vid_file_3 = os.path.expanduser("~/Videos/VID_20190327_195218.mp4")
	
	labeler = Video_Labeler(vid_file_1)
	
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

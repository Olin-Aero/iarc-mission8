import pickle
import cv2
from tkinter import * 
from tkinter.filedialog import askopenfilename
from video_labeler import Annotation
import os
import io
from PIL import Image
from PIL import ImageTk
#import pandas as pd
import numpy as np
import tensorflow as tf
import sys
import time

sys.path.append("/home/maximilian/Documents/Github/models/research")
from PIL import Image
from object_detection.utils import dataset_util
from collections import namedtuple, OrderedDict
# a hack until schommer's ROS works
sys.path.append('../')
from box_utils import BoxUtils

class LabelChecker():
	def __init__(self, root, p_file=False, width=600):
		self.root = root

		if not p_file:
			self.p_file = self.select_p_file()
		else:
			self.p_file = p_file

		file = open(self.p_file,"rb")
		self.annotation_array = pickle.load(file)
		self.annotation_num = 0
		self.cap = cv2.VideoCapture(self.annotation_array[0].file_reference)

		# print()
		# print(Image.fromarray(np.zeros((width,height,3), np.uint8)))
		self.im_width = width
		self.im_height = int(width/self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) * self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

		self.orig_im_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
		self.orig_im_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
		self.canvas = Canvas(self.root, width = self.im_width, height = self.im_height) 
		self.canvas.grid(row=0, column=0)
		array = np.zeros((self.im_height,self.im_width, 3))
		image =  Image.fromarray(array.astype('uint8'))
		self.image = ImageTk.PhotoImage(image=image)
		self.current_frame  = self.annotation_array[0].frame_num
		print(self.current_frame)
		self.cap.set(1, self.current_frame)
		ret, img = self.cap.read()
		img = cv2.resize(img, (self.im_width, self.im_height))
		self.image_on_canvas = self.canvas.create_image(0, 0, anchor = NW, image =self.image)
		self.disp_opencv_img(img)
		# self.canvas_image = self.canvas.create_image(0,0, anchor="nw", image=img)
		# self.canvas.grid(row=0, column=0)
		# self.label = Label(self.root, image=ImageTk.PhotoImage(master=self.canvas, width=width, height=height))
		# self.label.grid(row=0, column=0)
		self.view_btn = Button(self.root, text="View Labels", command=self.view_callback)
		self.view_btn.grid(row=1, column=0)   

		self.convert_btn = Button(self.root, text="Generate Haar Labels", command=self.convert_callback)
		self.convert_btn.grid(row=1, column=1)


		
		# 
		# self.canvas.pack()
		# self.view_btn.pack()
		# self.convert_btn.pack()

	def disp_opencv_img(self, image):
		# print(image)
		image = cv2.resize(image, (self.im_width, self.im_height))
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		image = Image.fromarray(image.astype('uint8'))
		self.image = ImageTk.PhotoImage(image=image)
		# self.image = ImageTk.PhotoImage(file = "ball1.gif")
		# cv2.imshow("lol", img)
		# cv2.waitKey(0)
		self.canvas.itemconfig(self.image_on_canvas, image = self.image)
		# self.label.image = image
	def select_p_file(self):
		Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
		filename = askopenfilename(title = "Select file",filetypes = (("pickle files","*.p"),("all files","*.*"))) # show an "Open" dialog box and return the path to the selected file
		return filename

	def view_callback(self):
		if self.current_frame != self.annotation_array[self.annotation_num].frame_num:
			self.current_frame = self.annotation_array[self.annotation_num].frame_num
			self.cap.set(1, self.current_frame)
		
		rect_xyxy = self.annotation_array[self.annotation_num].bounding_rect
		# print("rect_xyxy normalized: ", rect_xyxy)
		# print(self.orig_im_width, self.orig_im_height)
		rect_xyxy = np.array(BoxUtils.unnormalize(rect_xyxy, BoxUtils.FMT_XYXY, [self.orig_im_height, self.orig_im_width]), np.uint32)
		# print("rect_xyxy: ", rect_xyxy)
		# print(rect_xyxy[0:2], rect_xyxy[2:4])
		self.annotation_num = self.annotation_num + 1
		ret, img = self.cap.read()
		cv2.rectangle(img, tuple(rect_xyxy[0:2]), tuple(rect_xyxy[2:4]), (0, 255, 255), 3)
		self.disp_opencv_img(img)
		time.sleep(.05)

		self.root.after(15, self.view_callback)
		self.current_frame = self.current_frame + 1
		# for label in self.annotation_array:

	def convert_callback(self):
		f_dat = open("haar_info.dat", "w+")
		current_frame = self.annotation_array[0].frame_num
		for annotation in self.annotation_array:
			fname = "pos_images/im_{}.jpg".format(annotation.frame_num)
			# print(annotation.bounding_rect)
			if self.current_frame != self.annotation_array[self.annotation_num].frame_num:
				current_frame = annotation.frame_num
				self.cap.set(1, current_frame)
			ret, im = self.cap.read()
			im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
			
			rect_xyxy = annotation.bounding_rect

			rect_xyxy = np.array(BoxUtils.unnormalize(rect_xyxy, BoxUtils.FMT_XYXY, [self.orig_im_height, self.orig_im_width]), np.uint32)
			rect_ccwh = rect_xywh = BoxUtils.convert(rect_xyxy,
				BoxUtils.FMT_XYXY,
				BoxUtils.FMT_CCWH
				)
			rect_ccwh = [rect_ccwh[0], rect_ccwh[1], max(rect_ccwh[2:4]),max(rect_ccwh[2:4])]
			rect_xywh = BoxUtils.convert(rect_ccwh,
				BoxUtils.FMT_CCWH,
				BoxUtils.FMT_XYWH
				)
			rect_xywh = map(int, rect_xywh)
			try:
				
				# self.disp_opencv_img(im_gray)
				line_out = fname + " 1 {} {} {} {}".format(*rect_xywh)
				print(line_out)
				# Write outputs
				ret = cv2.imwrite(fname, im_gray)
				f_dat.write(line_out + "\n")

				cv2.rectangle(im_gray, tuple(rect_xyxy[0:2]), tuple(rect_xyxy[2:4]), (255, 255, 255), 3)
			except:
				print("Skipping Frame")
			current_frame = current_frame + 1

			# print(annotation.)
		f_dat.close()
		print("Convert Callback!")

	def to_tf(img_file, boxs):
		with tf.gfile.GFile(img_file, 'rb') as fid:
			encoded_jpg = fid.read()

		# parse metadata
		encoded_jpg_io = io.BytesIO(encoded_jpg)
		image = Image.open(encoded_jpg_io)
		width, height = image.size

		filename = img_file.encode('utf8')
		image_format = b'jpg'

		# check if the image format is matching with your images.
		xmins = []
		xmaxs = []
		ymins = []
		ymaxs = []
		classes_text = []
		classes = []

		for box in boxs:
			cx, cy, w, h = box[:4]
			xmins.append(cx - (w/2.0))
			xmaxs.append(cx + (w/2.0))
			ymins.append(cy - (h/2.0))
			ymaxs.append(cy + (h/2.0))
			classes_text.append('drone')
			classes.append( 1 )
			#classes.append(class_text_to_int(row['class']))

		tf_example = tf.train.Example(features=tf.train.Features(feature={
			'image/height': dataset_util.int64_feature(height),
			'image/width': dataset_util.int64_feature(width),
			'image/filename': dataset_util.bytes_feature(filename),
			'image/source_id': dataset_util.bytes_feature(filename),
			'image/encoded': dataset_util.bytes_feature(encoded_jpg),
			'image/format': dataset_util.bytes_feature(image_format),
			'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
			'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
			'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
			'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
			'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
			'image/object/class/label': dataset_util.int64_list_feature(classes),
		}))
		return tf_example

def main():
	root = Tk()
	LabelChecker(root)
	root.mainloop()
	# checker = LabelChecker()

if __name__ == "__main__":
	main()


# from tkinter import *

# master = Tk()

# def callback():
#     print("click!")

# b = Button(master, text="OK", command=callback)
# b.pack()

# mainloop()
#!/usr/bin/env python2
import rospy
from mode import Mode
from sensor_msgs.msg import Image
from iarc_forebrain.msg import ImageBin
from iarc_arbiter.drone import Drone


class Photo(Mode):
	DEFAULT_PITCH_ANGLE = 30 # degrees
	PHOTO_PITCH_ANGLE = 60 # degrees
	WAIT_TIME = rospy.Duration.from_sec(1.0)
	def __init__(self, drone):
		self.drone = drone
		rospy.Subscriber(drone.namespace+"image_raw", Image, self.image_raw_callback)
		self.qrPub = rospy.Publisher("/qr_image", ImageBin, queue_size=10)
		self.image = None
		self.start_time = rospy.Time.now()
		self.bin_number = -1
		self.has_triggered = False

	def image_raw_callback(self,data):
		self.image = data

	def enable(self, binNumber=0):
		# makes a imagebin message and sends it to the qr node
		self.active=True
		self.start_time=rospy.Time.now()
		self.bin_number = binNumber
		self.has_triggered = False
		self.drone.move_camera(self.PITCH_ANGLE, 0)

	def update(self, look_direction=0, obstacles=[]):
		self.drone.hover()
		if (not self.has_triggered and (rospy.Time.now() - self.start_time)>self.WAIT_TIME):
			if(self.image == None):
				rospy.logwarn("Image publishing delayed.")
				return
			imageBinMsg = ImageBin()
			imageBinMsg.image = self.image
			imageBinMsg.bin = int(self.bin_number)
			self.qrPub.publish(imageBinMsg)
			self.has_triggered = True
			rospy.logwarn("Image captured from drone ")
			self.drone.move_camera(self.DEFAULT_PITCH_ANGLE, 0)


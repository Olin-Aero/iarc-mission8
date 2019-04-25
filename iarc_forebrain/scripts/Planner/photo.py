#!/usr/bin/env python2
import rospy
from mode import Mode
from sensor_msgs.msg import Image
from iarc_forebrain.msg import ImageBin
from iarc_arbiter.drone import Drone


class Photo(Mode):
	def __init__(self, drone):
		self.drone = drone
		rospy.Subscriber(drone.namespace+"image_raw", Image, self.image_raw_callback)
		self.qrPub = rospy.Publisher("/qr_image", ImageBin, queue_size=10)
		self.image = None
	def image_raw_callback(self,data):
		self.image = data

	def enable(self, binNumber=0):
		# makes a imagebin message and sends it to the qr node
		self.active=True
		imageBinMsg = ImageBin()
		imageBinMsg.image = self.image
		imageBinMsg.bin = int(binNumber)
		if(self.image == None):
			rospy.logwarn("No images are being published.")
			return
		self.qrPub.publish(imageBinMsg)
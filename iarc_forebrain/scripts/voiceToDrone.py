#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from util.Drone import Drone

class voiceToDrone():
	def __init__(self):
		rospy.init_node('keyboardInput', anonymous=True)
		self.voiceSub = rospy.Subscriber('/voice',String,self.voiceCB)
		self.alpha = Drone()
		self.updateRate = rospy.Rate(5)
		self.message = None

	def voiceCB(self,msg):
		self.message = msg.data
		alphaPos = self.alpha.get_pos().pose.position
		print(alphaPos)
		words = self.message.split(" ")
		if(words[0] == "alpha"):
			if(words[1] == "north"):
				self.alpha.move_to(alphaPos.x,alphaPos.y+float(words[2]))
			if(words[1] == "west"):
				self.alpha.move_to(alphaPos.x-float(words[2]),alphaPos.y)
			if(words[1] == "south"):
				self.alpha.move_to(alphaPos.x,alphaPos.y-float(words[2]))
			if(words[1] == "east"):
				self.alpha.move_to(alphaPos.x+float(words[2]),alphaPos.y)


	def run(self):
		while not rospy.is_shutdown() and self.message == None:
			rospy.loginfo("No messages")

		while not rospy.is_shutdown():
			rospy.loginfo("got it")
			self.updateRate.sleep()

if __name__ == '__main__':
	vo = voiceToDrone()
	vo.run()
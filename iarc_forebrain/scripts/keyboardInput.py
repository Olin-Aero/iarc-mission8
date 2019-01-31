#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

class keyboardInput():
	def __init__(self):
		rospy.init_node('keyboardInput', anonymous=True)
		self.voicePub = rospy.Publisher('/voice',String,queue_size=10)

	def run(self):
		while not rospy.is_shutdown():
			text = raw_input("Tell me what to do: ")
			self.voicePub.publish(text)

if __name__ == '__main__':
	key = keyboardInput()
	key.run()

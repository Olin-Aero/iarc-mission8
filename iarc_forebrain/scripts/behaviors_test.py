#!/usr/bin/env python2

import math
import rospy
from util.Drone import Drone
from geometry_msgs.msg import Pose, Point, Quaternion

class BehaviorTest(object):
	def __init__(self):
		self.drone = Drone()
		self.onboardTest()
		# status, string = self.run()
		# print string
		self.drone.land

	def testTakeOffLand(self): # Tests takeoff and landing behaviors
		self.drone.takeoff()
		self.drone.hover(3)
		if not self.posEqual(0,0,1.45):
			return False

		self.drone.land()
		if not self.posEqual(0,0,0):
			return False

		self.drone.takeoff(2.0)
		self.drone.hover(3)
		if not self.posEqual(0,0,2.0):
			return False
		return True

	def testMoveTo(self): # Tests absolute coordinate motion commands
		self.drone.move_to(1.0, 0.0, 'odom', 2.0, 0.1) # TODO: Publish to /map once sim tf frames are fixed
		self.drone.hover(3)
		if not self.posEqual(1.0,0,2.0):
			return False
		return True

	def testMoveRelative(self): # Tests relative coordinate motion commands
		self.drone.move_relative(1.0, 0.0, 'odom', 2.0, 0.1) # TODO: Publish to /map once sim tf frames are fixed
		self.drone.hover(3)
		if not self.posEqual(2.0,0,4.0,0.5):
			return False
		return True

	def posEqual(self, x=0, y=0, z=0, tol=0.2):
		# Returns True if the drone is at the given coordinates and false if not.
		currPos = self.drone.get_pos().pose.position

		if(math.sqrt((x-currPos.x)**2 + (y-currPos.y)**2 + (z-currPos.z)**2) <= tol):
			return True
		else:
			print "Expected " + str(Point(x,y,z)) + '\n' + " Actual was " + str(currPos)
			return False

	def simTest(self):
		if not self.testTakeOffLand():
			return False, "TakeoffLand failed"
		if not self.testMoveTo():
			return False, "MoveTo failed"
		if not self.testMoveRelative():
			return False, "MoveRelative failed"
		return True, "All tests completed successfully."

	def onboardTest(self):
		print "Taking off..."
		self.drone.takeoff(1)
		self.drone.hover(3)
		print "Moved!"
		self.drone.move_relative(1.0, 0.0, 'odom', 0.0, 0.1)
		self.drone.hover(3)
		print "Moved!"
		self.drone.move_relative(1.0, 0.0, 'odom', 0.0, 0.1)
		self.drone.hover(3)
		print "Landing..."


if __name__ == '__main__':
	BehaviorTest()

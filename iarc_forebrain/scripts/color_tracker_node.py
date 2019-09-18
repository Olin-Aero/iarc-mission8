#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sys


class ColorTrackerNode:

    def __init__(self, tracker):
        # Create a new ROS node for this program
        rospy.init_node('color_tracker')

        # CvBridge is a library that converts ROS images to OpenCV images
        self.bridge = CvBridge()
        self.tracker = tracker

        # Subscribe to the ROS topic for the drone camera feed
        # To subscribe to a topic, you specify a callback function, in this case image_raw_callback
        # Whenever a new video frame is available, it will be passed to the callback function
        self.pub = rospy.Publisher("/color_target_coordinates", Point, queue_size=10)
        rospy.Subscriber("/ardrone/front/image_raw", Image, self.image_raw_callback)

    def image_raw_callback(self, msg):
        # If an error is thrown, the try-except statement prevents the program from crashing
        try:
            # Convert the ROS message to an OpenCV image
            # "bgr8" specifies the color format of the image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Identify the target coordinates
            target = self.tracker(frame, False)
            # Publish the target coordinates to a ROS topic
            if len(target)>=3:
                self.pub.publish(target[0], target[1], target[2])
            else:
                self.pub.publish(target[0], target[1], 0)

        except CvBridgeError as e:
            print(e)
      
    def run(self):
        # Keep the program running
        rospy.spin()

# Start the node
if __name__ == '__main__':
    if len(sys.argv) >= 2:
        # Load the color tracking function by filename
        exec('import %s' % sys.argv[1])
        tracker = sys.modules[sys.argv[1]].__dict__["detect_helmet_coordinates"]
        # Run the node
        test = ColorTrackerNode(tracker)
        test.run()
    else:
        print("ERROR: Program requires filename (excluding the extension) of detect_helmet_coordinates function as an argument.")
        print("> rosrun iarc_forebrain color_tracker_node.py helmet_tracker_logic")

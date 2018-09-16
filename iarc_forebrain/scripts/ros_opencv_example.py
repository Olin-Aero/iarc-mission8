#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class ROS_CV_Test:

    def __init__(self):
        # Create a new ROS node for this program
        rospy.init_node('ROS_CV_Test')

        # Subscribe to the ROS topic for the drone camera feed
        # To subscribe to a topic, you specify a callback function, in this case image_raw_callback
        # Whenever a new video frame is available, it will be passed to the callback function
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_raw_callback)
        
        # CvBridge is a library that converts ROS images to OpenCV images
	self.bridge = CvBridge()

    def image_raw_callback(self, msg):
        # If an error is thrown, the try-except statement prevents the program from crashing
        try:
            # Convert the ROS message to an OpenCV image
            # "bgr8" specifies the color format of the image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Define the upper and lower color thresholds as arrays
            lower = np.array([150,150,150])
            upper = np.array([255,255,255])

            # A mask specifies a boolean values (0 or 1) for each pixel of the image,
            # indicating whether that pixel is within the color threshold
            mask = cv2.inRange(frame, lower, upper)

            # Erase (set to black) any pixels that are the wrong color
            frame = cv2.bitwise_and(frame, frame, mask=mask)

            # Display the filtered image
            cv2.imshow('frame',frame)
        except CvBridgeError as e:
            print(e)
        cv2.waitKey(3)
      
    def run(self):
        # Keep the program running
        rospy.spin()

        # Clean up any leftover OpenCV windows
        cv2.destroyAllWindows()

# Start the program
if __name__ == '__main__':
    test = ROS_CV_Test()
    test.run()

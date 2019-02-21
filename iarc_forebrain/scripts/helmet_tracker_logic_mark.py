#!/usr/bin/env python2
import sys
import cv2

# In normal usage, this function will be imported by the *_ros file, and called from there.
# For test purposes, this file is also executable and can directly process a saved image.
def detect_helmet_coordinates(image, visualize=True):
    """
    Detects the pixel coordinates of the helmet in the image feed

    Input: OpenCV image
    Output: tuple of x- and y- coordinates of the center of the green helmet in the display
            or (-1, -1) if no helmet is visible.
    """

    # set upper and lower bounds for color I want to track in HSV
    greenLower = (29, 86, 6)
    greenUpper = (70, 250, 250)

    # blur image and convert to HSV
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # filter for green
    mask = cv2.inRange(hsv, greenLower, greenUpper)

    # erode and dilate to get rid of noise in the filtered image
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
 	
 	# find moments of green object
    M = cv2.moments(mask)

    try:
    	# calculate x, y positino of object from moments
    	cX = int(M["m10"] / M["m00"])
    	cY = int(M["m01"] / M["m00"])

    	# if visualize is True, draw crosshairs
    	if visualize:
    		cv2.line(image, (cX - 12, cY), (cX + 12, cY), (255, 0, 0), 4)
    		cv2.line(image, (cX, cY - 12), (cX, cY + 12), (255, 0, 0), 4)
        	cv2.imshow('Raw Image', image)
		
		# (x, y) measured in pixels
		return (cX, cY)
    
    except:
    	# if the object is not in the frame, imshow returns the original image...
    	cv2.imshow('Raw Image', image)

    	# ...and return (0, 0) for position
    	return (-1, -1)



### Test code for processing data directly from the computer webcam or a saved image file
if __name__ == '__main__':
    if len(sys.argv) == 2:
        filename = sys.argv[1]
        if filename == 'webcam':
            # Process the webcam in realtime
            camera = cv2.VideoCapture(0)  # "0" here means "the first webcam on your system"

            while True:
                # Get the latest image from the webcam
                ok, image = camera.read()
                if not ok:
                    print("Unable to open webcam...")
                    break

                # Process it
                coordinates = detect_helmet_coordinates(image)
                print(coordinates)

                # Exit if the "Esc" key is pressed
                key = cv2.waitKey(1)
                if key == 27:
                    break
        else:
            # Read data from an image file
            image = cv2.imread(filename)
            coordinates = detect_helmet_coordinates(image)
            print(coordinates)
            cv2.waitKey(0)

    else:
        print('Usage: When you run this script, provide either the filename of an image or the string "webcam"')

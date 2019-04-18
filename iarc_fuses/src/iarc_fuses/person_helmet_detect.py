#!/usr/bin/env python2

import cv2
import numpy as np
import person_detect
import object_detection_tf 

person_detector = object_detection_tf.ObjectDetectorTF(use_gpu=False, cmap={1:'person'})
cam = cv2.VideoCapture(0)

while True:
  ret, img = cam.read()
  if not ret:
    break

  objs = person_detector(img) # Find all objs 

  # Select objects with high scores and the right class, split out their scores and boxes
  mask = (objs['score'] > 0.5) & (objs['class'] == 'person')
  scores = objs['score'][mask]
  boxes = objs['box'][mask] # upper-left y, x, lower-right y, x, as fractions of the image
  
  # Draw bounding boxes
  for (box, score) in zip(boxes, scores):
    object_detection_tf.draw_bbox(img, box, str(score))

  # Show the thing
  cv2.imshow('win', img)
  k = cv2.waitKey(1)
  if k in [ord('q'), 27]:
      print('quitting...')
      break

  # Search for helmets in the bounding boxes
  for box in boxes:
    # Convert fractions to pixel counts
    bounds = np.round(box * np.array([img.shape[0], img.shape[1], img.shape[0], img.shape[1]])).astype(int)
    subimg = img[bounds[0]:bounds[2], bounds[1]:bounds[3], :]
    helmets = person_detect.detect_helmet(subimg)
    print(helmets)


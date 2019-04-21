#!/usr/bin/env python2

import cv2
import numpy as np
import utils
import helmet_detect
import object_detection_tf 

helmet_detector = object_detection_tf.ObjectDetectorTF(use_gpu=False, cmap={1:'person'})
cam = cv2.VideoCapture(0)

while True:
  ret, img = cam.read()
  if not ret:
    break

  objs = helmet_detector(img) # Find all objs 

  # Select objects with high scores and the right class, split out their scores and boxes
  mask = (objs['score'] > 0.5) & (objs['class'] == 'person')
  scores = objs['score'][mask]
  boxes = objs['box'][mask] # upper-left y, x, lower-right y, x, as fractions of the image
  
  # Draw bounding boxes
  for (box, score) in zip(boxes, scores):
    object_detection_tf.draw_bbox(img, box, str(score))


  # Search for helmets in the bounding boxes
  helmets = []
  for box in boxes:
    # Convert fractions to pixel counts
    bounds = np.round(box * np.array([img.shape[0], img.shape[1], img.shape[0], img.shape[1]])).astype(int)
    subimg = img[bounds[0]:bounds[2], bounds[1]:bounds[3], :]
    helmet = helmet_detect.detect_helmet(subimg, show=False)
    # Move contour from relative to subimg to relative to whole image
    if helmet is not None: 
      helmet = utils.translate_contour(helmet, (bounds[1], bounds[0])) 
    helmets += [helmet]


  # Outline helmets which are big enough to matter
  size_ratio_threshold = 0.01
  size_threshold = 100
  for (box, helmet) in zip(boxes, helmets):
    if helmet is not None:
      # TODO: Don't calculate bounds twice.  Combine loops? Store this instead of boxes?
      bounds = np.round(box * np.array([img.shape[0], img.shape[1], img.shape[0], img.shape[1]])).astype(int)
      box_area = abs((bounds[2]-bounds[0]) * (bounds[3]-bounds[1]))
      helmet_area = cv2.contourArea(helmet)
      # if helmet_area / box_area > size_ratio_threshold and helmet_area > size_threshold:
      if True:
        # Draw the helmet
        img = cv2.drawContours(img, [helmet], 0, (255,255,255), 1)

  # Show the image, with bounding boxes and helmets
  cv2.imshow('win', img)
  k = cv2.waitKey(1)
  if k in [ord('q'), 27]:
      print('quitting...')
      break

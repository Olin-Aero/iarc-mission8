#!/usr/bin/env python2

import cv2
import person_detect
import object_detection_tf 

person_detector = object_detection_tf.ObjectDetectorTF(use_gpu=False, cmap={1:'person'})
cam = cv2.VideoCapture(0)

while True:
  ret, img = cam.read()
  if not ret:
    break

  objs = person_detector(img) # Find all objs 

  # Convert dict of lists to list of dicts
  objs = [{'box': b, 'score': s, 'class': c} for (b, s, c) in zip(objs['box'], objs['score'], objs['class'])]
  objs = [obj for obj in objs if obj['score'] > 0.5 and obj['class'] == 'person'] # limit to high scores
  for obj in objs:
    object_detection_tf.draw_bbox(img, obj['box'], str(obj['class']))

  cv2.imshow('win', img)
  k = cv2.waitKey(1)
  if k in [ord('q'), 27]:
      print('quitting...')
      break

  

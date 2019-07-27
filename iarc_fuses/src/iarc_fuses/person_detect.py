#!/usr/bin/env python2

# USAGE
# python detect.py --images images

# import the necessary packages
from __future__ import print_function
from imutils.object_detection import non_max_suppression
import numpy as np
import imutils
from detect_config import PERSON_HSV_THRESHOLDS
from object_track import Object_Tracker
import cv2
import utils


def detect_helmet(image):
    frame_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    disp_contours = np.zeros(image.shape[:2], dtype="uint8")
    kernel = np.ones((5,5),np.uint8)
    # frame_HSV = cv2.dilate(frame_HSV,kernel,iterations = 1)
    # frame_HSV = cv2.dilate(frame_HSV)
    frame_threshold = cv2.inRange(frame_HSV, PERSON_HSV_THRESHOLDS['lowHSV'], PERSON_HSV_THRESHOLDS['highHSV'])
    cv2.imshow("threshold", frame_threshold)

    im2, contours, hierarchy = cv2.findContours(frame_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return
    # print("_____________________")
    # print(contours)
    # contours = imutils.grab_contours(contours)
    contour = max(contours, key=cv2.contourArea)
    disp_contours = cv2.drawContours(disp_contours, [contour], 0, (255,255,255), 1)
    cv2.imshow("contours", disp_contours)
    cv2.waitKey(0)
    return contour


def main2():
    vidPath = './schommer.mp4'
    cap = cv2.VideoCapture(vidPath)

    # Start tracker by initializing state
    ret, image = cap.read()
    # tracker.start_tracking(image, [334, 128, 438, 188])

    # startedTracking = False
    while cap.isOpened():
        ret, image = cap.read()
        if not ret: # If we have reached the end of the video
          break
        image = cv2.resize(image, None, fx=.5, fy=.5)
        detect_helmet(image)
        # res = utils.convert_to_pixels(np.shape(image), tracker.track(image))
        # print(tracker.state)
        # cv2.rectangle(image, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 3)
        # Show image
        cv2.imshow("object detection", image)

        # Press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def main():
    # initialize the HOG descriptor/pels
    # rson detector

    vidPath = '/home/maximilian/Pictures/VID_20190220_192021.mp4'
    classPath = 'yolov3.txt'
    weights = 'yolov3.weights'
    config = 'yolov3.cfg'
    tracker = Object_Tracker()
    cap = cv2.VideoCapture(vidPath)

    startedTracking = False
    while cap.isOpened():
        ret, image = cap.read()
        image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)
        Width = image.shape[1]
        Height = image.shape[0]
        scale = 0.00392

        # read class names from text file
        classes = None
        with open(classPath, 'r') as f:
            classes = [line.strip() for line in f.readlines()]

        # generate different colors for different classes
        COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

        # read pre-trained model and config file
        net = cv2.dnn.readNet(weights, config)

        # create input blob
        blob = cv2.dnn.blobFromImage(image, scale, (416, 416), (0, 0, 0), True, crop=False)

        # set input blob for the network
        net.setInput(blob)

        if not startedTracking:
            # run inference through the network
            # and gather predictions from output layers
            outs = net.forward(get_output_layers(net))

            # initialization
            class_ids = []
            confidences = []
            boxes = []
            conf_threshold = 0.5
            nms_threshold = 0.4

            # for each detetion from each output layer
            # get the confidence, class id, bounding box params
            # and ignore weak detections (confidence < 0.5)
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        center_x = int(detection[0] * Width)
                        center_y = int(detection[1] * Height)
                        w = int(detection[2] * Width)
                        h = int(detection[3] * Height)
                        x = center_x - w / 2
                        y = center_y - h / 2
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])


            # apply non-max suppression
            indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

            # go through the detections remaining
            # after nms and draw bounding box
            for i in indices:
                if (classes[int(i)] == 'person'):
                    i = i[0]
                    box = boxes[i]
                    x = box[0]
                    y = box[1]
                    w = box[2]
                    h = box[3]
                    bbox = [x, y, w, h]
                    tracker.start_tracking(image, bbox)
                    draw_bounding_box(image, class_ids[i], confidences[i], round(x), round(y), round(x + w), round(y + h), classes, COLORS)
            print("Done")


        # display output image
        cv2.imshow("object detection", image)

        # # wait until any key is pressed
        # cv2.waitKey()

        # save output image to disk
        cv2.imwrite("object-detection.jpg", image)

        # # release resources
        # cv2.destroyAllWindows()

        ########### HOG STARTS HERE ##############
        # hog = cv2.HOGDescriptor()
        # hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        #
        # image = imutils.resize(image, width=min(400, image.shape[1]))
        # orig = image.copy()
        #
        # # detect people in the image
        # (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        #                                         padding=(8, 8), scale=.5)
        #
        # # draw the original bounding boxes
        # for (x, y, w, h) in rects:
        #     cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
        #
        # # apply non-maxima suppression to the bounding boxes using a
        # # fairly large overlap threshold to try to maintain overlapping
        # # boxes that are still people
        # rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        # rects = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        #
        # # draw the final bounding boxes
        # for (xA, yA, xB, yB) in rects:
        #     cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
        #
        # # show some information on the number of bounding boxes
        # filename = vidPath[vidPath.rfind("/") + 1:]
        # print("[INFO] {}: {} original boxes, {} after suppression".format(
        #     filename, len(rects), len(rects)))
        #
        # # show the output images
        # cv2.imshow("Before NMS", orig)
        # cv2.imshow("After NMS", image)
        # # cv2.waitKey(0)
        #
        # # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # #
        # # cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        ################## HOG ENDS HERE ########################

    cap.release()
    cv2.destroyAllWindows()


# function to get the output layer names
# in the architecture
def get_output_layers(net):
    layer_names = net.getLayerNames()

    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers


# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h, classes, COLORS):
    label = str(classes[class_id])

    color = COLORS[class_id]
    # print(img, (x, y), (x_plus_w, y_plus_h), int(color[0]))
    # cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), int(color[0]), 2)

    pt1 = (int(x), int(y))
    pt2 = (int(x_plus_w), int(y_plus_h))

    cv2.rectangle(img, pt1, pt2, (0, 255, 0), 3)
    font = cv2.FONT_HERSHEY_SIMPLEX
    # cv2.putText(img, 'OpenCV', (10, 500), font, 4, (255, 255, 255), 2, cv2.LINE_AA)
    # cv2.rectangle(img, pt1, pt2, int(color[0]))
    cv2.putText(img, label, (int(x - 10), int(y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


if __name__ == '__main__':
    # main()
    main2()

#! /usr/bin/env python3
"""
	Project:
			Mitch detector
	Author: Mitchell D Scott
	Description:
		Detects and displays images
"""
import os
import cv2
import yaml
import rospy
import numpy as np
import buffvision as bv
from sensor_msgs.msg import Image

class MDS_Detector:
	def __init__(self, low, high, color=(0, 255, 0), thickness=2, in_topic='image_raw', out_topic='detected_boundary', with_core=False):
		"""
			Define all the parameters of the model here.
			No need to save images or results
		"""
		self.low = low #  Lower boundary of threshold
		self.high = high #  Upper boundary of threshold

		self.steps = None

		self.ANNOTATION_COLOR = color 
		self.ANNOTATION_THICKNESS = thickness

		# Only spin up image sub if core is running
		if with_core:
			self.in_topic = in_topic
			self.out_topic = out_topic

			rospy.init_node('mds_detector', anonymous=True)

			self.bound_pub = rospy.Publisher(out_topic, Float64MultiArray, queue_size=1)
			self.im_subscriber = rospy.Subscriber(in_topic, Image, self.imageCallBack, queue_size=1)

	def drawLines(self, image, contour):
		line = cv2.fitLine(contour, cv2.DIST_L2, 0, 1, 1)
		# find two points on the line
		x = int(line[0])
		y = int(line[1])
		dx = int(line[2])
		dy = int(line[3])	

		return cv2.line(image, (x, y), (x + dx, y + dy), self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)

	def drawEllipse(self, image, contour):
		ellipse = cv2.fitEllipse(contour)
		return cv2.ellipse(image, ellipse, self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)

	def detect(self, image, steps=False):
		"""
			Define all image processing operations here
			@PARAMS:
				image: an RGB image 
				steps: T/F record steps for debuging
			@RETURNS:
				bounds: bounding box of the detected object [(x1,y1), (x2,y2)]
		"""
		bounds = None
		source = image.copy()
		blurred = cv2.bilateralFilter(image, 15, 150, 45)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
		thresh = cv2.inRange(hsv, self.low, self.high)
		thresh = cv2.bitwise_and(image, image, mask=thresh)
		mask = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)
		# if you don't know this already,
		# you need to go read the docs
		contours, hierarchy = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)	

		if steps:
			annotated = image
			for cnt in contours:
				if len(cnt) > 20:
					#annotated = cv2.drawContours(annotated, [cnt], -1, self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)
					annotated = self.drawEllipse(annotated, cnt)
					
			mostly_raw = np.concatenate([source, hsv], axis=1)
			not_so_raw = np.concatenate([thresh, annotated], axis=1)
			self.steps = np.concatenate([mostly_raw, not_so_raw])

		return bounds

	def detect_and_annotate(self, image):
		"""
			Displays the steps in the processing line
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				None
		"""
		results = self.detect(image, steps=True)
		bv.buffshow('Steps', self.steps)


	def detect_and_publish(self, image):
		"""
			Define all image processing operations here.
			Publishes the bounds to topic instead of returning.
			Maybe in debug also publishes an annotated image.
			@PARAMS:
				image: an RGB image (should be 640x480)
			@RETURNS:
				None
		"""
		results = self.detect(image)
		mesg = Float64MultiArray()
		mesg.data = results
		self.bound_pub.publish(mesg)

	def imageCallBack(self, img_msg):
		"""
			Callback for image_raw
			@PARAMS:
				img_msg: the incoming message
			@RETURNS:
				None
		"""
		detect_and_publish(img_msg.data) # Not tested
		

def main():
	idx = [20, 21, 22, 23] # for looking at specific images
	data_path = os.path.join(os.getenv('PROJECT_ROOT'), 'data')
	data = bv.load_data(path=data_path)
	red_hsv_low = (120,108,180)  # hsv for red, lower boundary
	red_hsv_high = (128,145,240) # hsv for red, upper boundary
	blue_hsv_low = (0,45,165) # hsv for blue, lower boundary tuned on cold data
	blue_hsv_high = (175,100,250) # hsv for blue, upper boundary tuned on cold data

	# Replace with Yaml.read ^^

	blue_det = MDS_Detector(blue_hsv_low, blue_hsv_high, with_core=True)

	red_det = MDS_Detector(red_hsv_low, red_hsv_high)

	for i in idx:
		image, label = data[i]

	# for image, label in data:

		red_det.detect_and_annotate(image.copy())
		blue_det.detect_and_annotate(image)

if __name__=='__main__':
	main()
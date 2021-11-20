#! /usr/bin/env python
"""
	Project:
			Mitch detector
	Author: Mitchell D Scott
	Description:
		Detects and displays images
"""
import os
import sys
import cv2
import yaml
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

class MDS_Detector:
	def __init__(self, config=None, low=None, high=None, 
					color=(0, 255, 0), thickness=2, 
					in_topic='image_raw', out_topic='detected_boundary', 
					with_core=False, debug=False):
		"""
			Define all the parameters of the model here.
			No need to save images or results
		"""
		if config is None:
			if low is None or high is None:
				# exit if no bounds or config
				return None
			self.low = low #  Lower boundary of threshold
			self.high = high #  Upper boundary of threshold
		else:
			# read config params
			file = os.path.join(os.getenv('PROJECT_ROOT'), 'config', 'lib', '{}.yaml'.format(config))
			with open(file, 'r') as f:
				bounds = yaml.safe_load(f)
				self.low = np.array(bounds['LOW'])
				self.high = np.array(bounds['HIGH'])
				if 'INTOPIC' in bounds:
					in_topic = bounds['INTOPIC']
				if 'OUTOPIC' in bounds:
					out_topic = bounds['OUTTOPIC']
				if 'CORE' in bounds:
					with_core = bounds['CORE']
				if 'DEBUG' in bounds:
					debug = bounds['DEBUG']

		self.steps = None
		self.ANNOTATION_COLOR = color
		self.ANNOTATION_THICKNESS = thickness

		# Only spin up image sub if core is running
		if with_core:

			self.debug = debug
			self.in_topic = in_topic
			self.out_topic = out_topic

			self.bridge = CvBridge()

			rospy.init_node('mds_detector', anonymous=True)

			self.bound_pub = rospy.Publisher(out_topic, Float64MultiArray, queue_size=1)
			self.annotate_pub = rospy.Publisher('annotated_image', Image, queue_size=1)
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
		_, contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)	

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


	def detect_and_publish(self, image, steps=False):
		"""
			Define all image processing operations here.
			Publishes the bounds to topic instead of returning.
			Maybe in debug also publishes an annotated image.
			@PARAMS:
				image: an RGB image (should be 640x480)
			@RETURNS:
				None
		"""
		results = self.detect(image, steps=steps)
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
		
		if self.debug:
			self.detect_and_publish(self.bridge.imgmsg_to_cv2(img_msg), True) # Not tested
			self.annotate_pub.publish(self.bridge.cv2_to_imgmsg(self.steps, encoding='bgr8'))

		else:
			self.detect_and_publish(self.bridge.imgmsg_to_cv2(img_msg))

def main():
	detector = MDS_Detector(config=sys.argv[1])

	rospy.spin()

if __name__=='__main__':
	main()
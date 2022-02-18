#! /usr/bin/env python3
"""
	Project:
			buffnet detector
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
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

class MDS_Detector:
	def __init__(self, configData=None):
		"""
			Define all the parameters of the model here.
			Can be initialized with a config file, a system launch
			or manually from a terminal. will exit if not enough params
			exist.
		"""
		self.debug = rospy.get_param('/buffbot/DEBUG')
		topics = rospy.get_param('/buffbot/TOPICS')

		self.topics = [topics[t] for t in configData['TOPICS']]

		if configData is None:
			# there is no config for the model to load from
			return None

		self.steps = None
		if 'LOW' in configData:
			self.low = np.array(configData['LOW'])
		if 'HIGH' in configData:
			self.high = np.array(configData['HIGH'])
		if 'ANNOTATION_COLOR' in configData:
			self.ANNOTATION_COLOR = configData['ANNOTATION_COLOR']
		if 'ANNOTATION_THICKNESS' in configData:
			self.ANNOTATION_THICKNESS = configData['ANNOTATION_THICKNESS']


		# Only spin up image sub if core is running
		if len(self.topics) > 0:

			self.bridge = CvBridge()

			rospy.init_node('mds_detector', anonymous=True)

			self.debug_pubs = []
			self.bound_pub = rospy.Publisher(self.topics[1], Float64MultiArray, queue_size=1)

			if self.debug and len(self.topics) == 6:
				for topic in self.topics[2:]:
						self.debug_pubs.append(rospy.Publisher(topic, Image, queue_size=1))

			self.im_subscriber = rospy.Subscriber(self.topics[0], Image, self.imageCallBack, queue_size=1)

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

	def detect(self, image):
		"""
			Define all image processing operations here
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				bounds: bounding box of the detected object [(x1,y1), (x2,y2)]
		"""
		bounds = []
		source = image.copy()
		blurred = cv2.bilateralFilter(image, 15, 150, 45)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
		thresh = cv2.inRange(hsv, self.low, self.high)
		thresh = cv2.bitwise_and(image, image, mask=thresh)
		mask = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)
		# if you don't know this already,
		# you need to go read the docs
		_, contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)	

		annotated = image
		for i, cnt in enumerate(contours):
			a1 = cv2.contourArea(cnt)
			if a1 > 20:
				#annotated = cv2.drawContours(annotated, [cnt], -1, self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)
				annotated = self.drawEllipse(annotated, cnt)
			

				rect = cv2.minAreaRect(cnt)
				box = np.int0(cv2.boxPoints(rect))
				# m1 = np.abs(box[1][0] - box[0][0]) / np.abs(box[1][1] - box[0][1]) 
				cv2.drawContours(annotated, [box], 0, self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)
				#cv2.line(annotated, (box[0][0], box[0][1]), (box[3][0], box[3][1]), self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)
				bounds = np.array([rect[0][0], rect[0][1], rect[1][0], rect[1][1], rect[2]])
		
		self.steps = [blurred, hsv, thresh, annotated]

		return bounds

	def detect_and_annotate(self, image):
		"""
			Displays the steps in the processing line
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				None
		"""
		results = self.detect(image)
		mostly_raw = np.concatenate(steps[:2], axis=1)
		not_so_raw = np.concatenate(steps[2:], axis=1)
		collage = np.concatenate([mostly_raw, not_so_raw])
		bv.buffshow('Steps', collage)

	def publish_steps(self):
		"""
			Images processing steps are saved, this will publish them
		"""
		for i, pub in enumerate(self.debug_pubs):
				pub.publish(self.bridge.cv2_to_imgmsg(self.steps[i], encoding='bgr8'))

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
		
		if not results is None:
			self.bound_pub.publish(mesg)

		if self.debug:
			self.publish_steps()

	def imageCallBack(self, img_msg):
		"""
			Callback for in_topic
			@PARAMS:
				img_msg: the incoming message
			@RETURNS:
				None
		"""
		self.detect_and_publish(self.bridge.imgmsg_to_cv2(img_msg))


def main(configData):

	if configData is None:
		return

	detector = BuffNet(configData=configData)

	if 'TOPICS' in configData:
		rospy.spin()

	if 'DATA' in configData:	
		# run independantly
		data = bv.load_data(path=os.path.join(os.get_env('PROJECT_ROOT'), 'data', configData['DATA']))

		for image, labels in data[0:5]:
			detector.detect_and_annotate(image)


if __name__=='__main__':
	if len(sys.argv) < 2:
		main({})
	if sys.argv[1][-5:] == '.yaml':
		path = os.path.join(os.getenv('PROJECT_ROOT'), 'config', 'lib', sys.argv[1])
		with open(path, 'r') as f:
			data = yaml.safe_load(f)
		main(data)
	elif '/buffbot' in sys.argv[1]:
			main(rospy.get_param(sys.argv[1]))
	else:
		rospy.logerr('Unsupported call: call this with a rosparam component name or a yaml config')








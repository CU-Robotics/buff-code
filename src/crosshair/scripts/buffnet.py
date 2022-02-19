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
import torch
import numpy as np
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from gdrive_handler import GD_Handler
from std_msgs.msg import Float64MultiArray

class BuffNet:
	def __init__(self, configData=None):
		"""
			Define all the parameters of the model here.
			Can be initialized with a config file, a system launch
			or manually from a terminal. will exit if not enough params
			exist.
		"""
		if 'DEBUG' in configData:
			self.debug = True

		model_dir = os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'models')
		model_path = os.path.join(model_dir, 'best.pt')

		if not os.path.exists(model_path):
			gdrive = GD_Handler()
			gdrive.downloadFile(file='BuffNetv2-exp6', path=model_dir)

		self.model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)

		if not rospy.is_shutdown():
			self.debug = rospy.get_param('/buffbot/DEBUG')
			topics = rospy.get_param('/buffbot/TOPICS')
			self.topics = [topics[t] for t in configData['TOPICS']]
			# Only spin up image sub if core is running
			if len(self.topics) > 0:

				self.bridge = CvBridge()
				rospy.init_node('buffnet', anonymous=True)
				self.target_pub = rospy.Publisher(self.topics[1], Float64MultiArray, queue_size=1)

				if self.debug and len(self.topics) > 2:
					self.debug_pub = rospy.Publisher(self.topics[2], Image, queue_size=1)

				self.im_subscriber = rospy.Subscriber(self.topics[0], Image, self.imageCallBack, queue_size=1)

		elif 'TOPICS' in configData:
			self.topics = configData['TOPICS']
		
		else:
			self.topics = []

		if configData is None:
			# there is no config for the model to load from
			return None

		if 'ANNOTATION_COLOR' in configData:
			self.ANNOTATION_COLOR = configData['ANNOTATION_COLOR']
		if 'ANNOTATION_THICKNESS' in configData:
			self.ANNOTATION_THICKNESS = configData['ANNOTATION_THICKNESS']


	def detect(self, image):
		"""
			Define all image processing operations here
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				annotations: bounding box of the detected object with color and class [class, name, (x1,y1), (w,h)]
		"""
		image = cv2.resize(image, (416, 416))
		pred = self.model(image).pandas().xywh

		annotation = []
		detection = []
		for i in range(len(row)):
			annotation.append([row['class'][i], row['name'][i], round(row['xcenter'][i]), round(row['ycenter'][i]), round(row['width'][i]), round(row['height'][i])])
			detection.apend([round(row['xcenter'][i]), round(row['ycenter'][i]), round(row['width'][i]), round(row['height'][i])])

		return annotation, detection

	def annotate_image(self, images=None, labels=None):

		if image is None or labels is None:
			return None

		annotated_image = image.copy()

		for label in labels:
			w = int(label[4])
			h = int(label[5])
			p1 = (int(label[2]) - int(w / 2), int(label[3]) - int(h / 2))
			p2 = (int(label[2]) + int(w / 2), int(label[3]) + int(h / 2))
			annotated_image = cv2.rectangle(annotated_image, p1, p2, ANNO_COL[label[0] % 3], 2)
			annotated_image = cv2.putText(annotated_image, label[1], p1, cv2.FONT_HERSHEY_SIMPLEX, 0.4, ANNO_COL[label[0] % 3], 1, cv2.LINE_AA)
		
		return annotated_image

	def detect_and_annotate(self, image):
		"""
			Displays the steps in the processing line
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				None
		"""
		results = self.detect(image)
		annotated = self.annotate_images([image])
		bv.buffshow('Annotation', annotated)

	def publish_annotated(self, image, label):
		"""
			Images processing steps are saved, this will publish them
		"""
		pub.publish(self.bridge.cv2_to_imgmsg(self.annotate_images([image], [label])[0], encoding='bgr8'))

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
		annotations, predictions = self.detect(image)
		mesg = Float64MultiArray()

		if len(preds) < 1:
			mesg.data = [-1.0, -1.0, -1.0, -1.0, -1.0]
		else:
			mesg.data = np.array(predictions, dtype=np.float64)
			self.target_pub.publish(mesg)

		# if self.debug:
		# 	self.publish_annotated(image.copy, labels)

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
		rospy.logerr('Unsupported call: use this with a rosparam component name or a yaml config')








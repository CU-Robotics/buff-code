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
from std_msgs.msg import Float64MultiArray

class BuffNet:
	def __init__(self):
		"""
			Define all the parameters of the model here.
			Can be initialized with a config file, a system launch
			or manually from a terminal. will exit if not enough params
			exist.
		"""

		model_dir = os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'models')
		model_path = os.path.join(model_dir, rospy.get_param('/buffbot/MODEL/PT_FILE'))

		print(model_path)
		
		if not os.path.exists(model_path):
			print('No model')
			return

		self.model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)

		rospy.init_node('buffnet', anonymous=True)

		self.bridge = CvBridge()

		self.debug = rospy.get_param('/buffbot/DEBUG')

		topics = rospy.get_param('/buffbot/TOPICS')

		self.target_pub = rospy.Publisher(topics['DETECTION_PIXEL'], Float64MultiArray, queue_size=1)

		if self.debug:
			self.debug_pub = rospy.Publisher(topics['IMAGE_DEBUG'], Image, queue_size=1)

		self.im_subscriber = rospy.Subscriber(topics['IMAGE'], Image, self.imageCallBack, queue_size=1)

		r = rospy.get_param('/buffbot/CAMERA/RESOLUTION')
		self.image_size = (r, r)



	def detect(self, image):
		"""
			Define all image processing operations here
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				annotations: bounding box of the detected object with color and class [class, name, (x1,y1), (w,h)]
		"""
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		prediction = np.array(self.model(image).pandas().xywh)[0]

		annotation = []
		detection = []

		if len(prediction) < 1:
			return annotation, detection

		for x,y,w,h,cf,cl,n in prediction:
			annotation.append([round(x), round(y), round(w), round(h), cf, cl, n])
			detection.append([round(x), round(y), round(w), round(h), cf, cl])

		return annotation, detection

	def generate_color(self, cl):
		return (np.cos(cl) * 255, np.sin(cl) * 255, np.tan(cl) * 255)

	def annotate_image(self, frame, labels):

		if frame is None or labels is None:
			return None

		image = frame.copy()

		colors = [(255,0,0), (0,0,255), (0,255,0)]
		if len(labels) < 1:
			return image

		for [x, y, w, h, cl, c, name] in labels:
			x1 = int(x - (w / 2))
			x2 = int(x + (w / 2))
			y1 = int(y - (h / 2))
			y2 = int(y + (h / 2))
			image = cv2.rectangle(image, (x1, y1), (x2, y2), colors[int(c)], 2)
			image = cv2.putText(image, name + f'-{np.round(cl, 4)}', (x1, y1 - 15), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
		
		return image

	def detect_and_annotate(self, image):
		"""
			Displays the steps in the processing line
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				None
		"""
		results = self.detect(image)
		annotated = self.annotate_image(image, results[0])
		bv.buffshow('Annotation', annotated)

	def publish_annotated(self, image):
		"""
			Images processing steps are saved, this will publish them
		"""
		self.debug_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))

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
		image = cv2.resize(image, self.image_size)
		annotations, predictions = self.detect(image)
		mesg = Float64MultiArray()

		if len(predictions) < 1:
			mesg.data = [-1.0, -1.0, -1.0, -1.0, -1.0]
		else:
			for pred in predictions:
				mesg.data = np.array(pred, dtype=np.float64)
				self.target_pub.publish(mesg)

		if self.debug:
			self.publish_annotated(self.annotate_image(image, annotations))

	def imageCallBack(self, img_msg):
		"""
			Callback for in_topic
			@PARAMS:
				img_msg: the incoming message
			@RETURNS:
				None
		"""
		self.detect_and_publish(self.bridge.imgmsg_to_cv2(img_msg))


def main():
	detector = BuffNet()
	rospy.spin()


if __name__=='__main__':
	main()


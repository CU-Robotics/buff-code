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
	def __init__(self, config_data=None):
		"""
			Define all the parameters of the model here.
			Can be initialized with a config file, a system launch
			or manually from a terminal. will exit if not enough params
			exist.
		"""
		if 'DEBUG' in config_data:
			self.debug = True

		model_dir = os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'models')
		model_path = os.path.join(model_dir, rospy.get_param('/buffbot/MODEL/MODEL_FILE'))

		if not os.path.exists(model_path):
			gdrive = GD_Handler()
			gdrive.downloadFile(file='BuffNetv3-exp5', path=model_dir, title='BuffNetv3-exp5.pt')
			model_path = os.path.join(model_dir, 'BuffNetv3-exp5.pt')

		self.model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)

		self.bridge = CvBridge()
		rospy.init_node('buffnet', anonymous=True)
		self.debug = rospy.get_param('/buffbot/DEBUG')

		topics = rospy.get_param('/buffbot/TOPICS')

		self.target_pub = rospy.Publisher(topics['DETECTION_PIXEL'], Float64MultiArray, queue_size=1)
		if self.debug and len(pubs) > 1:
			self.debug_pub = rospy.Publisher(topics['IMAGE_DEBUG'], Image, queue_size=1)

		if len(subs) > 0:
			self.im_subscriber = rospy.Subscriber(['IMAGE'], Image, self.imageCallBack, queue_size=1)

		if 'CAMERA' in config_data:
			w, h, d = rospy.get_param('/buffbot/CAMERA/RESOLUTION')
			self.image_size = (w, d)
		else:
			self.image_size = (320, 320)


	def detect(self, image):
		"""
			Define all image processing operations here
			@PARAMS:
				image: an RGB image 
			@RETURNS:
				annotations: bounding box of the detected object with color and class [class, name, (x1,y1), (w,h)]
		"""
		image = cv2.resize(image, self.image_size)
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

	def annotate_image(self, image=None, labels=None):

		if image is None or labels is None:
			return None

		annotated_image = image.copy()

		scale = (image.shape[1] / self.image_size[0], image.shape[0] / self.image_size[1])
		for label in labels:
			w = int(label[2]) * scale[0]
			h = int(label[3]) * scale[1]
			p1 = (int((label[0] - (w / 2)) * scale[0]), int((label[1] - (h / 2)) * scale[1]))
			p2 = (int((label[0] + (w / 2)) * scale[0]), int((label[1] + (h / 2)) * scale[1]))
			color = self.generate_color(label[5])
			annotated_image = cv2.rectangle(annotated_image, p1, p2, color, 2)
			annotated_image = cv2.putText(annotated_image, f'{label[6]}-{round(label[4])}%', p1, cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
		
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


def main(config_data):

	if config_data is None:
		return

	detector = BuffNet(config_data=config_data)

	rospy.spin()


if __name__=='__main__':
	if len(sys.argv) < 2:
		print(f'No Data: BuffNet exiting ...')
	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))
	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)
		main(data)

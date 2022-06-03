#! /usr/bin/env python3
import os
import cv2
import sys
import time
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray

"""
	Project pixel coordinates to the world
	
"""

class Projector:
	def __init__(self, data):
		# camera heading
		self.phi = 0.0
		self.psi = 0.0
		self.a = data['A']
		self.m = data['M']

		self.FOV = rospy.get_param('/buffbot/CAMERA/FOV')
		image_res = rospy.get_param('/buffbot/CAMERA/RESOLUTION')
		self.image_size = np.array([image_res, image_res, 3])

		self.init_ros(data)

	def init_ros(self, data):
		rospy.init_node('projector', anonymous=True)
		self.rate = rospy.Rate(data['RATE'])
		
		self.debug = rospy.get_param('/buffbot/DEBUG')
		topics = rospy.get_param('/buffbot/TOPICS')

		self.detect_sub = rospy.Subscriber(
			topics['DETECTION_PIXEL'], Float64MultiArray, self.detection_callback, queue_size=5)

		self.gimbal_sub = rospy.Subscriber(
		 	topics['GIMBAL_STATE'], Float64MultiArray, self.gimbal_callback, queue_size=1)

		self.project_pub = rospy.Publisher(
			topics['DETECTION_WORLD'], Float64MultiArray, queue_size=1)


	def detection_callback(self, msg):
		"""
		Parse a detection msg
		PARAMS:
			msg: Float64MultiArray, detection msg, data=[x,y,w,h,cf,cl]
		"""
		# for now. need to figure out how to get accurate time between messages?
		# Build a custom message that has a timestamp
		t = time.time()
		# do projector stuff
		r = self.project(np.array(msg.data))
		msg = Float64MultiArray(data=r)
		self.project_pub.publish(msg)

	def gimbal_callback(self, msg):
		state = msg.data
		self.psi = state[0]
		self.phi = state[1]

	def height_2_distance(self, h):
		return (self.a * h) + (self.m / h)

	def project(self, detection):
		"""
		Projects a detection into the world frame
		PARAMS:
			pose: Float64MultiArray.data, [x1,y1,x2,y2,cf,cl] (detection msg)
		RETURNS:
			vector (x,y): body frame position of the detection
		"""

		x1, y1, x2, y2, cf, cl = detection
		xc = (x1 + x2) / 2
		w = abs(x2 - x1)
		h = abs(y2 - y1)
		d = self.height_2_distance(h)
		alpha = np.radians((1 - (xc / self.image_size[0])) * self.FOV)
		print('=====')
		print(h, d)
		print(xc, alpha)
		return d * np.cos(self.phi) * np.array([np.cos(self.psi + alpha), np.sin(self.psi + alpha)])


def main(data):
	projector = Projector(data)

	try:
		while not rospy.is_shutdown():
			# for sim
			projector.rate.sleep()

	except KeyboardInterrupt as e:
		print('Projector killed')

	except Exception as e:
		print(e)


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print(f'No Data: Projector exiting ...')

	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))

	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)

		main(data)




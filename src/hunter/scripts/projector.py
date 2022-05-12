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
	def __init__(self):
		# camera heading
		self.phi = 0.0
		self.psi = 0.0
		self.a = config_data['A']
		self.m = config_data['M']
		self.P = config_data['P']

	def height_2_distance(h):
		return self.a * np.exp(self.m * (h + self.p))

	def project(self, pose):
		"""
		Projects a detection into the world frame
		PARAMS:
			pose: Float64MultiArray.data, [x,y,h,w,cf,cl] (detection msg)
		RETURNS:
			vector (x,y): body frame position of the detection
		"""
		d = height_2_distance(pose[2])
		alpha = np.radians((1 - (pose[0] / self.image_size[0])) * self.FOV)
		return d * np.cos(self.phi) np.array([np.cos(self.psi + alpha), np.sin(self.psi + alpha)])
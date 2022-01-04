#! /usr/bin/env python3
import os
import sys
import cv2
import rospy
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class cv2_Camera:
	def __init__(self, device, topic, fps=30, debug=False):
		# init camera
		self.camera = cv2.VideoCapture(device)
		# set fps
		self.camera.set(cv2.CAP_PROP_FPS, fps)

		# set image resolution
		self.resolution = (int(self.camera.get(3)), int(self.camera.get(4)))

		# Create the image publisher
		self.pub = rospy.Publisher(topic, Image, queue_size=1)
		# Init this program as a ROS node
		# anonymous sets a unique node ID
		rospy.init_node('omega_streamer', anonymous=True)

		# Initialize the bridge object
		# CvBridge is used to conver cv images to sensor_msgs/Image
		self.bridge = CvBridge()

		# set the debug mode
		self.debug = debug
		
		if debug:
			rospy.loginfo('Camera and publisher Initialized: {} {} {}'.format(device, topic, fps))

	def stream(self):
		# If the stream is open and ROS is running
		if cap.isOpened():
			if self.debug:
				rospy.loginfo('Camera is open and the stream is starting: ...')
			while not rospy.is_shutdown():

				# Capture frame
				ret, frame = cap.read()

				if ret:
					# Convert the cv frame to a ROS message
					imgMsg = bridge.cv2_to_imgmsg(frame, "bgr8")
					# Publish the message
					pub.publish(imgMsg)
			# use this return code so we can know if it should respawn
			return 0

		else:
			rospy.logerr('Could\'nt open camera: Exiting...')
			return 1

#! /usr/bin/env python
import os
import sys
import cv2
import rospy
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main(config):

	debug, topics, config = bv.load_config(config, 'CAMERA')

	# Create the image stream and set its capture rate to 30 FPS
	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FPS, 30)

	# Set a size variable (resolution)
	size=(int(cap.get(3)), int(cap.get(4)))

	# Create the image publisher
	pub = rospy.Publisher(topics['RAW_IMG'], Image, queue_size=1)
	# Init this program as a ROS node
	# anonymous sets a unique node ID
	rospy.init_node('image_streamer', anonymous=True)

	# Initialize the bridge object
	# CvBridge is used to conver cv images to sensor_msgs/Image
	bridge = CvBridge()

	rospy.loginfo('Streaming camera')
	# If the stream is open and ROS is running
	if cap.isOpened():
		while not rospy.is_shutdown():

			# Capture frame
			ret, frame = cap.read()

			# Convert the cv frame to a ROS message
			imgMsg = bridge.cv2_to_imgmsg(frame, "bgr8")

			# Publish the message
			pub.publish(imgMsg)


if __name__=='__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])
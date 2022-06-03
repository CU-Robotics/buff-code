#! /usr/bin/env python3
import os
import sys
import cv2
import yaml
import glob
import rospy
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Data_Streamer:
	def __init__(self):
	

		all_topics = rospy.get_param('/buffbot/TOPICS')

		# Create the image publisher
		self.pub = rospy.Publisher(all_topics['IMAGE'], Image, queue_size=1)
		# Init this program as a ROS node
		# anonymous sets a unique node ID
		rospy.init_node('omega_streamer', anonymous=True)

		# Initialize the bridge object
		# CvBridge is used to conver cv images to sensor_msgs/Image
		self.bridge = CvBridge()

		self.rate = rospy.Rate(rospy.get_param('/buffbot/CAMERA/FPS'))

		# set the debug mode
		self.debug = rospy.get_param('/buffbot/DEBUG')

		self.lives = 9


	def stream(self, data_path):
		# If the stream is open and ROS is running
		r = rospy.get_param('/buffbot/CAMERA/RESOLUTION')

		for root, dirs, files in os.walk(data_path):
			for d in dirs:
				data_yaml = os.path.join(root, d, 'data.yaml')
				if os.path.exists(data_yaml):
					image_path = os.path.join(root, d, 'test', 'images')
					m = len(image_path) + 1
					image_files = glob.glob(os.path.join(image_path, '*.jpg'))

					for i, imfile in enumerate(image_files):
						image = cv2.imread(imfile)
						msg = self.bridge.cv2_to_imgmsg(cv2.resize(image, (r, r)))
						self.pub.publish(msg)
						self.rate.sleep()
							

def main():
	project_root = os.getenv('PROJECT_ROOT')
	data_path = os.path.join(project_root, 'data')

	ds = Data_Streamer()
	ds.stream(data_path)


if __name__=='__main__':
	main()

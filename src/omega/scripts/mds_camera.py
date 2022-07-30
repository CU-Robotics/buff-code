#! /usr/bin/env python3
import os
import sys
import cv2
import yaml
import rospy
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class cv2_Camera:
	def __init__(self):
		# Save config for reset
		self.init_camera()

		all_topics = rospy.get_param('/buffbot/TOPICS')

		# Create the image publisher
		self.pub = rospy.Publisher(all_topics['IMAGE'], Image, queue_size=1)
		# Init this program as a ROS node
		# anonymous sets a unique node ID
		rospy.init_node('omega_streamer', anonymous=True)

		# Initialize the bridge object
		# CvBridge is used to conver cv images to sensor_msgs/Image
		self.bridge = CvBridge()

		# set the debug mode
		self.debug = rospy.get_param('/buffbot/DEBUG')

		self.lives = 9
		
		if self.debug:
			rospy.loginfo('Camera and publisher Initialized: {} {} {}'.format(rospy.get_param('/buffbot/CAMERA/DEVICE'), all_topics['IMAGE'], self.fps))

	def init_camera(self):
		# init camera
		self.camera = cv2.VideoCapture(rospy.get_param('/buffbot/CAMERA/DEVICE'))
		# set fps
		self.fps = rospy.get_param('/buffbot/CAMERA/FPS')
		self.camera.set(cv2.CAP_PROP_FPS, self.fps)

		# set image resolution
		r = rospy.get_param('/buffbot/CAMERA/RESOLUTION')
		self.resolution = (r, r, 3)
		self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
		self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

	def stream(self):
		# If the stream is open and ROS is running
		if self.camera.isOpened():
			if self.debug:
				rospy.loginfo('Camera is open and the stream is starting: ...')
			
			rate = rospy.Rate(self.fps)
			while not rospy.is_shutdown() and self.lives > 0:

				# Capture frame
				ret, frame = self.camera.read()


				if ret:
					# Convert the cv frame to a ROS message
					imgMsg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
					# Publish the message
					self.pub.publish(imgMsg)

					rate.sleep()

				else:
					self.lives -= 1
					if self.debug:
						rospy.logerr('Camera reseting after return: {}'.format(ret))

					self.camera.release()

					rospy.sleep(1)

					self.camera = cv2.VideoCapture(rospy.get_param('/buffbot/CAMERA/DEVICE'))

			self.camera.release()
			return

		else:
			return 1 



def main(config_data):

	# create the video stream
	camera = cv2_Camera()
		
	# Stream the video
	ret = camera.stream()

	if ret == 1 and 'DATA_DEFAULT' in config_data:
		rospy.logerr('Couldn\'t open camera: Trying video...')
		rospy.set_param('/buffbot/CAMERA/DEVICE', os.path.join(os.getenv('PROJECT_ROOT'), 'data', config_data['DATA_DEFAULT']))
		camera.camera.release()
		camera.init_camera()
		camera.stream()


if __name__=='__main__':
	if len(sys.argv) < 2:
		print(f'No Data: MDS camera exiting')
	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))
	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)
		main(data)

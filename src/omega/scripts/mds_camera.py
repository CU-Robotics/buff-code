#! /usr/bin/env python3
import os
import sys
import cv2
import yaml
import rospy
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from gdrive_handler import GD_Handler


class cv2_Camera:
	def __init__(self, config_data):
		# Save config for reset
		self.config_data = config_data
		self.init_camera()

		out_topic = config_data['TOPICS']['PUBLISH'][0]
		all_topics = rospy.get_param('/buffbot/TOPICS')


		# Create the image publisher
		self.pub = rospy.Publisher(all_topics[out_topic], Image, queue_size=1)
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
			rospy.loginfo('Camera and publisher Initialized: {} {} {}'.format(self.config_data['DEVICE'], out_topic, self.fps))

	def init_camera(self):
		# init camera
		self.camera = cv2.VideoCapture(self.config_data['DEVICE'])
		# set fps
		self.fps = self.config_data['FPS']
		self.camera.set(cv2.CAP_PROP_FPS, self.fps)

		# set image resolution
		w, h, d = self.config_data['RESOLUTION']
		self.resolution = (w, h, d)
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

					self.camera = cv2.VideoCapture(self.config_data['DEVICE'])

			self.camera.release()
			return

		elif 'DATA_DEFAULT' in self.config_data:
			self.lives -= 1
			return 1


def main(config_data):

	# create the video stream
	camera = cv2_Camera(config_data)
		
	# Stream the video
	ret = camera.stream()

	if ret == 1 and 'DATA_DEFAULT' in config_data:
		rospy.logerr('Couldn\'t open camera: Trying video...')
		camera.config_data['DEVICE'] = os.path.join(os.getenv('PROJECT_ROOT'), 'data', config_data['DATA_DEFAULT'])
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

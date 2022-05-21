#! /usr/bin/env python3
"""
	Project:
			BuffVision
	Author: Mitchell D Scott
	Description:
		This file contains tools for doing cv. there are functions
		that will be used in many programs, they can be imported to 
		any scripts from here. (as long as buffpy/lib is on PYTHONPATH)

"""
import os
import cv2
import sys
import time
import glob
import yaml
#import rospy
import datetime
import traceback
import numpy as np
#from cv_bridge import CvBridge
#from sensor_msgs.msg import Image
import xml.etree.ElementTree as ET

def buffshow(title, image, wait=0):
	"""
		Uses the openCV imshow and waitKey to display an image
		@PARAMS
			title: Title of the image
			image: openCV image or numpy nd-array
			wait: preference for the wait key
		@RETURNS
			None
		@DISPLAYS
			openCV window, awaits user input
	"""
	cv2.imshow(title, image)
	cv2.waitKey(wait)
	cv2.destroyAllWindows()

def display_annotated_raw(data_point):
	"""
		Uses buffshow to display an image with its annotation
		@PARAMS
			data_point: a touple of The label object (a dict like) and the image
		@RETURNS
			None
	"""
	image, label = data_point
	bounds = get_bounding_from_label(label)
	for bound in bounds:
		image = cv2.rectangle(image, bound[0], bound[1], (0,255,0), 2)
		
	buffshow('annotated', image)

def load_label(path):
	"""
		Load all labels from a directory.
		@PARAMS
			path: Path to the directory with labels
		@RETURNS
			labels: array of the label objects (see ElementTree docs)
	"""
	labels = []

	with open(path, 'r') as f:
		lines = f.readlines()
		for line in lines:
			labels.append(np.array(line.split(' '), dtype=np.float64))

	return np.array(labels)
	
def load_data(path='../data'): # default path only works in jupyter notebook or from project root
	"""
		Loads all images and labels in the given directory.
		Assumes paired file structure (XXX.xml, XXX.jpg)
		@PARAMS
			path: file to the directory with data
		@RETURNS
			None
	"""
	m = len(path) + len('labels') + 2
	label_paths = glob.glob(os.path.join(path, 'labels', '*.txt'))
	
	data = []
	for labelf in label_paths:

		imfile = os.path.join(path, 'images', labelf[m:-4] + '.jpg')
		if os.path.exists(imfile):
			image = cv2.imread(imfile)
			label = load_label(labelf)
			data.append([image, label])
				
	return data

def display_annotated(image, labels):
	"""
		Displays an image with its annotation using buffshow.
		@PARAMS
			data: a touple (image, bounds)
		@RETURNS
			None
	"""
	for c,x,y,w,h in labels:
		# Draw a rectangle on the image, green 2px thick
		image = cv2.rectangle(image, (int(x - (w/2)), int(y - (h/2))), (int(x + (w/2)), int(y + (h/2))), (0,255,0), 2)
		
	buffshow('annotated', image)

def dateFilledPath(fileExt='.png'):
	"""
		create a date and time stamped filepath. Root is always PROJECT_ROOT/data
		@PARAMS
			ext: the extension of the file, must match the file type (not checked)
		@RETURNS
			date filled filepath
	"""
	return os.path.join(os.getenv('PROJECT_ROOT'), 'data', datetime.datetime.now() + ext)

def save_image(image, filepath=None):
	"""
		Use openCV to save an image.
		@PARAMS
			image: the image to save (np array, cv2.mat... etc)
			filepath: place to save the image, if None will become a date
				timestamp and be saved in data (if it exists).
		@RETURNS
			None
	"""
	if not filepath is None:
		cv2.imwrite(filepath, image)
	else:
		cv2.imwrite(dateFilledPath(), image)

def single_image_capture_cv():
	"""
		Use openCV to create a stream to a camera and capture a single image.
		@PARAMS
			None
		@RETURNS
			None
	"""
	camera = cv2.VideoCapture(0)

	if camera.isOpened():
		read, frame = camera.read()
		save_image(frame)

	camera.release()

def capture_video(duration=10):
	"""
		Use openCV to create a stream to a camera and record a video.
		@PARAMS
			duration: length in seconds of the video (not exact)
		@RETURNS
			None
	"""
	ext = 'avi'
	cap = cv2.VideoCapture(0)
	size=(int(cap.get(3)), int(cap.get(4)))
	out = cv2.VideoWriter(datefilledPath('video', fileExt='.{}'.format(ext)), cv2.VideoWriter_fourcc('M','J','P','G'), 30, size)

	if cap.isOpened():
		start = time.time()
		while True:
			ret, frame = cap.read()
			out.write(frame)

			if time.time() - start > duration:
				break

	cap.release()
	out.release()

def cv2_stream_test(device=0):
	"""
		Use openCV to create a stream to a camera and display fps
		Crucial to use while not rospy.is_shutdown
		@PARAMS
			None
		@RETURNS
			None
	"""

	# Create the image stream
	cap = cv2.VideoCapture(device)
	# Set a size variable (resolution)
	size=(int(cap.get(3)), int(cap.get(4)))

	# used to count frames and print FPS
	n = 0
	start = time.time()
	# If the stream is open and ROS is running
	if cap.isOpened():
		while not rospy.is_shutdown():

			# Capture frame
			ret, frame = cap.read()

			# Run stats
			n += 1
			elapsed = time.time() - start 
			print('elapsed time: {}, n frames: {}, FPS: {}'.format(elapsed, n, n / elapsed))

def ROS_cv2_publisher(topic='image_raw'):
	"""
		Use openCV to create a stream to a camera and publish the images
		to a ROS topic.
		@PARAMS
			topic: the name of the topic the image will publish to
		@RETURNS
			None
	"""

	# Create the image stream and set its capture rate to 30 FPS
	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FPS, 30)

	# Set a size variable (resolution)
	size=(int(cap.get(3)), int(cap.get(4)))

	# Create the image publisher
	pub = rospy.Publisher(topic, Image, queue_size=1)
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


def load_config_from_system_launch(args):
	"""
		DEPRECATED
		Used to handle input from a system launch
		PARAMS:
			args: list of args from system launch ([Program, Debug, Config, Topics...])
		RETURNS:
			program: filename of program
			debug: T/F debug or naaa
			data: dict of config data
			topics: dict of ros topics (name : type)
	"""
	program, _, debug, config = args[:4]

	if debug == 'True':
		debug = True

	filepath = os.path.join(os.getenv('PROJECT_ROOT'), 'config', 'lib', config)

	if os.path.exists(filepath):
		with open(filepath, 'r') as f:
			data = yaml.safe_load(f)

	else:
		data = None

	# exclude node names and logs for now
	return program, debug, data, args[4:-2]

if __name__=='__main__':
	"""
		Spawn vision nodes here
	# """
	# # single_image_capture_cv()
	# # capture_video()

	# ROS_cv2_publisher()

	# #cv2_stream_test()

	
		

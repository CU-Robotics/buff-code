#! /usr/bin/env python3
"""
	Project:
			BuffVision
	Author: Mitchell D Scott
	Description:
		This file contains tools for doing cv. This is where we
	will call our predicition model from. Think of this as 
	the crows nest the model sits in while looking for land (or whales).
	This design will allow us to spontaneously switch models
	or even spin up two instances on competing machines.

"""
import os
import cv2
import sys
import time
import glob
import rospy
import datetime
import traceback
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from gdrive_handler import GD_Handler


def buffshow(title, image, wait=0):
	cv2.imshow(title, image)
	cv2.waitKey(wait)
	cv2.destroyAllWindows()

def get_bounding_from_label(label):
	bounds = []
	boundingboxes = label.findall('object')
	for boundary in boundingboxes:
		bound = boundary.find('bndbox')
		xmin = int(bound.find('xmin').text)
		ymin = int(bound.find('ymin').text)
		xmax = int(bound.find('xmax').text)
		ymax = int(bound.find('ymax').text)
		bounds.append([(xmin, ymin), (xmax, ymax)])
	return bounds

def display_annotated_raw(data_point):
	image, label = data_point
	bounds = get_bounding_from_label(label)
	for bound in bounds:
		image = cv2.rectangle(image, bound[0], bound[1], (0,255,0), 2)
		
	buffshow('annotated', image)

def load_images(path):
	#	Deprecated
	# this is now handled in the data
	# loader function
	filenames = glob.glob(path)
	return [(file.split('/')[-1], cv2.imread(file)) for file in filenames] # by saving the image with its filename we can properly match it to the data

def load_labels(path):
	filenames = glob.glob(path)
	return [ET.parse(file) for file in filenames]

def get_image_file_from_label(label):
	return label.find('filename').text
	
def load_data(path='../data'): # default path only works in jupyter notebook
	# images = load_images(os.path.join(path, '*.jpg'))
	labels = load_labels(os.path.join(path, '*.xml'))
	# if len(images) - len(labels):
	# 	print(f'mismatched: images {len(images)} != labels {len(labels)}')
		
	data = []
	for label in labels:
		imfile = get_image_file_from_label(label)
		impath = os.path.join(path, imfile)
		if os.path.exists(impath):
			image = cv2.imread(impath)
			data.append((image, get_bounding_from_label(label)))
				
	return data

def display_annotated(data):
	image, bounds = data
	for bound in bounds:
		image = cv2.rectangle(image, bound[0], bound[1], (0,255,0), 2)
		
	buffshow('annotated', image)

def datefilledPath(type, ext='.png'):
	return f'/home/cu-robotics/buffvision/{type}/{datetime.datetime.now()}{ext}'

def save_image(image, filepath=None):

	if not filepath is None:
		cv2.imwrite(filepath, image)
	else:
		cv2.imwrite(datefilledPath('images'), image)

def single_image_capture_cv():
	camera = cv2.VideoCapture(0)

	if camera.isOpened():
		read, frame = camera.read()
		save_image(frame)

	camera.release()

def capture_video(duration=10):
	
	ext = 'avi'
	cap = cv2.VideoCapture(0)
	size=(int(cap.get(3)), int(cap.get(4)))
	out = cv2.VideoWriter(datefilledPath('video', ext=f'.{ext}'), cv2.VideoWriter_fourcc('M','J','P','G'), 30, size)

	if cap.isOpened():
		start = time.time()
		while True:
			ret, frame = cap.read()
			out.write(frame)

			if time.time() - start > duration:
				break

	cap.release()
	out.release()


if __name__=='__main__':
	#single_image_capture_cv()
	capture_video()
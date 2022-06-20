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
import string
import random
import shutil
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

def xywh2xyxy(x, y, w, h):
	h2 = h / 2
	w2 = w / 2
	x1 = (x - w2)
	x2 = (x + w2)
	y1 = (y - h2)
	y2 = (y + h2)
	return x1, y1, x2, y2

def crop_image(image, x, y, w, h):
	x1,y1,x2,y2 = xywh2xyxy(x, y, w, h)
	x1 *= image.shape[1]
	x2 *= image.shape[1]
	y1 *= image.shape[0]
	y2 *= image.shape[0]

	# print(f'image shape {image.shape}')
	# print(f'cropping {x1} {x2} {y1} {y2}')
	# print(f'shape: {round(abs(x1 - x2))} {round(abs(y1 - y2))}')
	return image[round(y1):round(y2), round(x1):round(x2)]

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

def get_annotated(image, labels):
	"""
		returns an image with its annotation using buffshow.
		@PARAMS
			data: a touple (image, bounds)
		@RETURNS
			None
	"""
	colors = [(255,0,0), (0,0,255), (255,0,255), (0,255,0)]

	for c,x,y,w,h in labels:
		# Draw a rectangle on the image, green 2px thick
		[x, w] = np.array([x, w]) * image.shape[1]
		[y, h] = np.array([y, h]) * image.shape[0]

		image = cv2.rectangle(image, (int(x - (w/2)), int(y - (h/2))), (int(x + (w/2)), int(y + (h/2))), colors[int(c)], 2)
		image = cv2.putText(image, f'{c}', (int(x - (w/2)), int(y - (h/2))-15), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255,255,255), 2, cv2.LINE_AA)

	return image

def display_annotated(image, labels):
	"""
		Displays an image with its annotation using buffshow.
		@PARAMS
			data: a touple (image, bounds)
		@RETURNS
			None
	"""
	colors = [(255,0,0), (0,0,255), (255,0,255), (0,255,0)]

	for c,x,y,w,h in labels:
		# Draw a rectangle on the image, green 2px thick
		[x, w] = np.array([x, w]) * image.shape[1]
		[y, h] = np.array([y, h]) * image.shape[0]

		image = cv2.rectangle(image, (int(x - (w/2)), int(y - (h/2))), (int(x + (w/2)), int(y + (h/2))), colors[int(c)], 2)
		image = cv2.putText(image, f'{c}', (int(x - (w/2)), int(y - (h/2))-15), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255,255,255), 2, cv2.LINE_AA)

	buffshow('annotated', image)

def dateFilledPath(fileName='00'):
	"""
		create a date and time stamped filepath. Root is always PROJECT_ROOT/data
		@PARAMS
			ext: the extension of the file, must match the file type (not checked)
		@RETURNS
			date filled filepath
	"""
	return f'{fileName}{datetime.datetime.now()}'.replace(' ', '').replace('.', '-')

def write_sample(save_path, key, image, label):
	date_string = dateFilledPath(fileName=f'{key}',)

	image_path = os.path.join(save_path, 'images')
	image_date_path = os.path.join(image_path, date_string + '.jpg')
	cv2.imwrite(image_date_path, image)

	label_path = os.path.join(save_path, 'labels')
	label_date_path = os.path.join(label_path, date_string + '.txt')
	with open(label_date_path, 'w+') as f:
		if len(label) < 1:
			f.write('')
		else:
			for c, x, y, w, h in label:
				f.write(f'{c} {x} {y} {w} {h}\n')


def save_txt_label_data(images, labels, gen_path):

	for i, (image,label) in enumerate(zip(images,labels)):

		key = ''.join(random.choice(string.ascii_uppercase) for i in range(5))
		write_sample(os.path.join(gen_path, 'train'), key, image, label)

def clear_generated(gen_path):
	if os.path.exists(gen_path):
		shutil.rmtree(gen_path)

def make_generated(gen_path):

	title = gen_path.split('/')[-1]

	train_path = os.path.join(gen_path, 'train')
	valid_path = os.path.join(gen_path, 'valid')
	test_path = os.path.join(gen_path, 'test')

	if not os.path.exists(gen_path):
		os.mkdir(gen_path)

	os.mkdir(train_path)
	os.mkdir(valid_path)
	os.mkdir(test_path)

	data_file = os.path.join(gen_path, 'data.yaml')

	with open(data_file, 'w+') as f:
		f.write(f'train: /content/{title}/train/\nval: /content/{title}/valid\n\nnc: 3\nnames: [\'blue-armor\', \'red-armor\', \'robot\']')

	os.mkdir(os.path.join(train_path, 'images'))
	os.mkdir(os.path.join(train_path, 'labels'))

	os.mkdir(os.path.join(valid_path, 'images'))
	os.mkdir(os.path.join(valid_path, 'labels'))

	os.mkdir(os.path.join(test_path, 'images'))
	os.mkdir(os.path.join(test_path, 'labels'))



	
		

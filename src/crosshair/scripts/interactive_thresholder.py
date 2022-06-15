#! /usr/bin/env python3
import os
import cv2
import sys
import glob
import numpy as np
import buffvision as bv

# Someday will become its own project

pixels = []


def click_event(event, x, y, flags, param):
	global pixels
	# on press mark position

	if event == cv2.EVENT_LBUTTONUP:
		print('Click detected at ({},{})'.format(x,y))
		pixels.append((x,y))

def run_stats(pixels):
	std = [0,0,0]
	n = len(pixels)
	if n == 0:
		return [0,0], [0,0]

	average = np.mean(pixels, axis=0)
	for pixel in pixels:
		for i in range(len(pixel)):
			std[i] += (pixel[i] - average[i])**2 / n

	return average, std

def main(data_dir):
	global pixels

	all_values = []
	value_high = [0,0,0]
	value_low = [180,255,255]

	cv2.namedWindow("image")
	cv2.setMouseCallback("image", click_event)

	project_root = os.getenv('PROJECT_ROOT')
	data_path = os.path.join(project_root, 'data', data_dir)

	m = len(data_path) + len('labels') + 2
	label_paths = glob.glob(os.path.join(data_path, 'labels', '*.txt'))
	
	for i,labelf in enumerate(label_paths):
		imfile = os.path.join(data_path, 'images', labelf[m:-4] + '.jpg')
		if os.path.exists(imfile):
			image = cv2.imread(imfile)
			label = bv.load_label(labelf)

			print(f'{i}: image shape: {image.shape} ')
			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

			#bv.buffshow('image', image)
			bv.display_annotated(image, label)

			for x, y in pixels:
				h, s, v = hsv[y,x]

				all_values.append([h,s,v])

				value_low[0] = min(value_low[0], h)
				value_low[1] = min(value_low[1], s)
				value_low[2] = min(value_low[2], v)

				value_high[0] = max(value_high[0], h)
				value_high[1] = max(value_high[1], s)
				value_high[2] = max(value_high[2], v)

			pixels = []

	avg, std = run_stats(all_values)

	print('Low: {}'.format(value_low))
	print('High: {}'.format(value_high))
	print('Average: {}'.format(avg))
	print('STD: {}'.format(std))



if __name__ == '__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])








	
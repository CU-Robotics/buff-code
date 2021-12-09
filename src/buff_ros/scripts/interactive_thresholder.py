#! /usr/bin/env python
import os
import cv2
import sys
import numpy as np
import buffvision as bv

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
	average = np.mean(pixels, axis=0)
	for pixel in pixels:
		for i in range(len(pixel)):
			std[i] += (pixel[i] - average[i])**2

	for i in range(len(std)):
		std[i] = std[i] / n

	return average, std

def main():
	global pixels

	all_values = []
	value_high = [0,0,0]
	value_low = [180,255,255]

	cv2.namedWindow("image")
	cv2.setMouseCallback("image", click_event)

	data = bv.load_data(os.path.join(os.getenv('PROJECT_ROOT'), 'data'))

	for i, (image, labels, filepath) in enumerate(data[0:5]):
		print('{}: image {} shape: {} '.format(i, filepath,image.shape))
		hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		cv2.imshow('image', image)
		cv2.waitKey()
	
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



if __name__=='__main__':
	main()
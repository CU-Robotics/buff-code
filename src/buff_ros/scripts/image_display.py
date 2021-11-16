#! /usr/bin/env python3
"""
	Project:
			ImageViewer
	Author: Mitchell D Scott
	Description:
		Displays images with openCV
"""
import os
import cv2
import buffvision as bv



def main():
	data = bv.load_data(path='/home/mdyse/buff-code/data')
	idx = [5, 31]
	for i in idx:
		image, label = data[i]
		bv.buffshow(f'Raw {i}', image)

	for i in idx:
		image, label = data[i]
		hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		bv.buffshow(f'HSV {i}', hsv)

if __name__=='__main__':
	main()
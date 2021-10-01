#! /usr/bin/env python3
import cv2
import sys
import time
import numpy
import datetime
import traceback


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
	# Will record videos but they are empty?
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
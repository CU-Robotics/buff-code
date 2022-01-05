#! /usr/bin/env python3
import os
import sys
import cv2
import rospy
import buffvision as bv
from camera import cv2_Camera
from gdrive_handler import GD_Handler

def scan_for_video():
	data_path = os.path.join(os.getenv('PROJECT_ROOT'), 'data')
	for root, dirs, files in os.walk(data_path):
		for f in files:
			if f[-4:] == '.mp4':
				return os.path.join(root, f)

	# when no video download one
	# needs testing
	gdrive = GD_Handler()
	gdrive.downloadBatch('Bad_Reputaion')

	return os.path.join('data', 'bad_reputation.mp4')


def main(debug, config, topic):
	# default fps and device
	fps = 30
	device = 0

	if 'DEVICE' in config:
		device = config['DEVICE']

	if 'FPS' in config:
		fps = config['FPS']

	# create the video stream
	camera = cv2_Camera(device, topic, fps=fps, debug=debug)
	# Stream the video
	ret = camera.stream()

	if ret == 1:
		video_file = scan_for_video()
		if video_file:
			camera = cv2_Camera(video_file, topic, fps=fps, debug=debug)
			# Stream the video
			ret = camera.stream()



if __name__=='__main__':
	if sys.argv[1] == 'sys-launch':
		program, debug, config, topics = bv.load_config_from_system_launch(sys.argv)
		main(debug, config, topics[0])

	else:
		rospy.logerr('Unsupported: Call visual spawner from a system launch')

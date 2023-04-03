#!/usr/bin/python3

import os
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage


def main():
	rospy.init_node('image_stream', anonymous=True)

	project_root = os.getenv("PROJECT_ROOT")
	data_file = os.path.join(project_root, 'data', 'sample_competition.avi')

	if not os.path.isfile(data_file):
		print(f"Sample video not found, download it from drive and put it under {data_file}")

	cap = cv2.VideoCapture(data_file)

	image_pub = rospy.Publisher("/image_raw/compressed", CompressedImage, queue_size=1)

	rate = rospy.Rate(30)

	while not rospy.is_shutdown() and cap.isOpened():

		ret, frame = cap.read()

		# if frame is read correctly ret is True
		if not ret:
			print("Can't receive frame (stream end?). Exiting ...")
			break

		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
		# Publish new image
		image_pub.publish(msg)

		rate.sleep()

	cap.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
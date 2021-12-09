#! /usr/bin/env python
import os
import sys
import rospy
import buffvision as bv
from sensor_msgs.msg import Image
from mds_detector import MDS_Detector

def displayCallback(image_msg):
	rospy.loginfo('image_recieved')
	detector = MDS_Detector(config="nn_hsv_red")
	detector.detect_and_annotate(CVbridge().imgmsg_to_cv2(image_msg))

def main(config):

	#rospy.init_node('debugger_view', anonymous=True)

	#sub = rospy.Subscriber('image_raw', Image, displayCallback, queue_size=1)

	#rospy.spin()

	data = bv.load_data(os.path.join(os.getenv('PROJECT_ROOT'), 'data'))

	detector = MDS_Detector(config=config)

	for image, labels in data[0:5]:
		detector.detect_and_annotate(image)

if __name__=='__main__':
	if len(sys.argv) > 1:
		main(sys.argv[1])
#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from mds_detector import MDS_Detector


def displayCallback(image_msg):
	rospy.loginfo('image_recieved')
	detector = MDS_Detector(config="nn_hsv_red")
	detector.detect_and_annotate(CVbridge().imgmsg_to_cv2(image_msg))

def main():

	rospy.init_node('debugger_view', anonymous=True)

	sub = rospy.Subscriber('image_raw', Image, displayCallback, queue_size=1)

	

	rospy.spin()
	

if __name__=='__main__':
	main()
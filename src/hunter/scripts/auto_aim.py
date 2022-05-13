#! /usr/bin/env python3
import os
import cv2
import sys
import time
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray


def setup(data):
	debug_pubs = {}        
	topics = rospy.get_param('/buffbot/TOPICS')
	debug = rospy.get_param('/buffbot/DEBUG')

	rospy.init_node('dr_tracer', anonymous=True)
	rate = rospy.Rate(data['RATE'])

	# detect_sub = rospy.Subscriber(
	# 	topics['DETECTION'], Float64MultiArray, detection_callback, queue_size=5)

	gimbal_sub = rospy.Subscriber(
		topics['GIMBAL_STATE'], Float64MultiArray)

	prediction_pub = rospy.Publisher(
		topics['SERIAL_OUT'], String, queue_size=1)

	if debug:
		debug_pubs['target_map'] = rospy.Publisher(topics['TARGET_MAP'], Image, queue_size=1)
		debug_pubs['tracking_err'] = rospy.Publisher(topics['TRACKING_ERROR'], Float64MultiArray, queue_size=1)
		
	return debug, rate, gimbal_sub, prediction_pub, debug_pubs

def publish_prediction(dr_tracker, serial_pub):
	"""
	Publish the yaw and pitch to aim at the target
	"""
	dr_tracker.predict()
	psi = np.arctan(dr_tracker.pose[1] / dr_tracker.pose[0]) # arctan of x,y is yaw
	phi = dr_tracker.shooter_drop * np.linalg.norm(dr_tracker.pose) # phi is this needs to be tuned function of distance
	psi_msg = String(f'BYR {psi}')
	phi_msg = String(f'BPR {phi}')
	serial_pub.publish(phi_msg)
	serial_pub.publish(psi_msg)

def detection_callback(self, msg):
	"""
	Parse a detection msg
	PARAMS:
		msg: Float64MultiArray, detection msg, data=[x,y,w,h,cf,cl]
	"""
	# for now. need to figure out how to get accurate time between messages?
	# Build a custom message that has a timestamp
	t = time.time()
	# do tracker stuff
	pose = np.array(msg.data)
	self.measure = self.project(pose)

def spin():
	dr_tracker = Dead_Reckon_Tracer()


def main(data):

	if config_data is None:
		return



if __name__ == '__main__':
	if len(sys.argv) < 2:
		print(f'No Data: BuffNet exiting ...')
	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))
	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)
		main(data)
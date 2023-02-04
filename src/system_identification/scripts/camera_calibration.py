#!/usr/bin/python3
import time
import numpy as np
from scipy import linalg as la
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float64MultiArray

gain = 0
set_point = 0
control_active = False
control_out = np.zeros((7))

def controller(msg):
	if control_active:
		output = msg.data[0]
		position_ref = msg.data[1]
		vel_ref = msg.data[2]
		pos_control = (gain * (set_point - msg.data[1]))
		control_out[4] = pos_control - (0.25 * (pos_control - control_out[4]))

def hold_pose(hz, duration):
	global gain
	global set_point
	global control_out
	global control_active

	gain =  5000
	set_point = np.pi / 3;
	rate = rospy.Rate(hz) # 10hz

	pub = rospy.Publisher('control_input', Float64MultiArray, queue_size=10)

	rospy.Subscriber(f"controller_5_report", Float64MultiArray, controller)

	print(f"Calibrator taking robot control: ...")
	ctr = 0
	control_active = True
	while ctr / hz < duration:

		msg = Float64MultiArray()
		msg.data = control_out
		pub.publish(msg)
		rate.sleep()

		if rospy.is_shutdown():
			exit(0)

		ctr += 1

	msg = Float64MultiArray()
	msg.data = np.zeros(7)
	pub.publish(msg)
	rate.sleep()
	
	control_active = False

	print(f"Calibrator Finished")

if __name__ == '__main__':
	try:
		rospy.init_node('camera_calibrator', anonymous=True)

		time.sleep(2)
		hz = 100
		duration = 5
		hold_pose(hz, duration)

	except rospy.ROSInterruptException:
		pass
#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray


def callback(mesg):
	rospy.loginfo('target_recieved')
	pass 

def main():

	rospy.init_node('serial_pipe', anonymous=True)

	sub = rospy.Subscriber('detected_targets', Float64MultiArray, callback, queue_size=5)

	rospy.spin()
	

if __name__=='__main__':
	main()
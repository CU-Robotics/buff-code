#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray


def callback(mesg, publisher):
	rospy.loginfo('boundary_recieved')

	publisher.publish(mesg)

def main():

	pub = rospy.Publisher('detected_targets', Float64MultiArray, queue_size=1)

	rospy.init_node('target_tracker', anonymous=True)

	sub = rospy.Subscriber('detected_boundary', Float64MultiArray, callback, pub, queue_size=5)

	rospy.spin()
	

if __name__=='__main__':
	main()
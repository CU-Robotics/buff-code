#! /usr/bin/env python3
import sys
import rospy
import buffvision as bv
from std_msgs.msg import Float64MultiArray


def callback(mesg):
	# write message
	# Serial.py
	pass
	

def main(configData):

	rospy.init_node('target_tracker', anonymous=True)

	all_topics = rospy.get_param('/buffbot/TOPICS')

	topics = [all_topics[t] for t in configData['TOPICS']]

	sub = rospy.Subscriber(topics[0], Float64MultiArray, callback, queue_size=5)

	rospy.spin()
	

if __name__=='__main__':
	if '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))
	else:
		main()
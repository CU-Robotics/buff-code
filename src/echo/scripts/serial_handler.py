#! /usr/bin/env python3
import sys
import rospy
import buffvision as bv
from std_msgs.msg import Float64MultiArray


def callback(mesg, publisher):
	# write message
	# Serial.py
	pass
	

def main(debug=False, config=None, topic='serial_out'):

	rospy.init_node('target_tracker', anonymous=True)

	sub = rospy.Subscriber(topic, Float64MultiArray, callback, queue_size=5)

	rospy.spin()
	

if __name__=='__main__':
	if sys.argv[1] == 'sys-launch':
		program, debug, config, topics = bv.load_config_from_system_launch(sys.argv)
		main(debug=debug, config=config, topic=topics[0])
	else:
		main()
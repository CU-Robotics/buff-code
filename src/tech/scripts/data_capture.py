#! /usr/bin/env python3
import rospy
import buffvision as bv
from std_msgs.msg import Float64MultiArray


def callback(mesg, publisher):
	# handle tracking here

	# then publish result
	publisher.publish(mesg)

def main(debug=False, config=None, topics=['detected_boundary']):

	pub = rospy.Publisher(topics[1], Float64MultiArray, queue_size=1)

	rospy.init_node('target_tracker', anonymous=True)

	sub = rospy.Subscriber(topics[0], , callback, pub, queue_size=5)

	rospy.spin()
	

if __name__=='__main__':
	if sys.argv[1] == 'sys-launch':
		program, debug, config, topics = bv.load_config_from_system_launch(sys.argv)
		main(debug=debug, config=config, topics=topics)
	else:
		main()
#! /usr/bin/env python3
import sys
import time
import rospy
import numpy as np
import buffvision as bv
from std_msgs.msg import Float64MultiArray


class MDS_Tracker():
	def __init__(self, configData):

		self.debug = rospy.get_param('/buffbot/DEBUG')
		topics = rospy.get_param('/buffbot/TOPICS')

		self.topics = [topics[t] for t in configData['TOPICS']]

		self.pose = None
		self.last_measure = None
		self.vStack = np.zeros(2)

		self.target_pub = rospy.Publisher(self.topics[1], Float64MultiArray, queue_size=1)
		
		if self.debug:
			self.err_pub = rospy.Publisher(self.topics[2], Float64MultiArray, queue_size=1)

		rospy.init_node('target_tracker', anonymous=True)

		self.detect_sub = rospy.Subscriber(self.topics[0], Float64MultiArray, self.callback, queue_size=5)

	def callback(self, msg):

		t = time.time()
		data = msg.data
		new_measure = np.array([data[0], data[1]])

		if self.pose is None or self.last_measure is None:
			self.vel = np.zeros(2)
		else:
			self.vel = (new_measure - self.pose) / (t - self.last_measure) # most recent change
			if self.debug:
				m = Float64MultiArray()
				m.data = (new_measure - self.pose)
				self.err_pub.publish(m)

		self.last_measure = t
		self.pose = new_measure

	def update_pos(self, t0):
		t1 = time.time()

		if self.last_measure is None or self.pose is None:
			return t1

		if time.time() - self.last_measure > 5:
			self.pose = None
			return t1
	
		self.pose += self.vel * (t1 - t0) # Velocity * Time = Distance

		return t1

	def publish_pos(self):
		if self.pose is None:
			return

		m = Float64MultiArray()
		m.data = self.pose
		self.target_pub.publish(m)

	def spin(self):
		rate = rospy.Rate(30)
		prev = time.time()
		start = prev

		while not rospy.is_shutdown():
			prev = self.update_pos(prev)
			# publish the estimate
			self.publish_pos()
			rate.sleep()


def main(configData):

	# Use this for debugging and script runs
	tracker = MDS_Tracker(configData=configData)
	tracker.spin()
	

if __name__=='__main__':
	if len(sys.argv) < 2:
		print(f'No Data: MDS tracker exiting')
	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))
	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)
		main(data)




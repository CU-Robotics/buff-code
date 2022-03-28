#! /usr/bin/env python3
import os
import sys
import time
import rospy
import numpy as np
import buffvision as bv
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray

"""
	Tracker class
	args:
	- config_data: dictionary containing the configuration data
		- data: file to save telemetry plots
		- topics: rostopic to subscribe and publish or debug output
	- Indicate that you want to save the predict
"""


class Dead_Reckon_Tracer:
	def __init__(self, config_data):

		self.t = 0
		self.error = 0
		self.t_offset = 0.02

		# Initialize the estimated postion at current time + offset
		self.pose = np.zeros(2, dtype=np.float64)
		# Initialize the history of measurements
		self.history = np.zeros((4,2), dtype=np.float64)
		# Initialize the trajectory of the target
		self.trajectory = np.zeros((3,2), dtype=np.float64)

		self.debug_pubs = {}        
		topics = rospy.get_param('/buffbot/TOPICS')
		self.debug = rospy.get_param('/buffbot/DEBUG')

		pubs = [topics[t] for t in config_data['TOPICS']['PUBLISH']]
		subs = [topics[t] for t in config_data['TOPICS']['SUBSCRIBE']]

		rospy.init_node('target_tracker', anonymous=True)
		self.rate = rospy.Rate(30)

		self.detect_sub = rospy.Subscriber(
			subs[0], Float64MultiArray, self.detected_callback, queue_size=1)

		self.prediction_pub = rospy.Publisher(
			pubs[0], Float64MultiArray, queue_size=1)

		if len(pubs) > 1:
			for pub in pubs[1:]:
				self.debug_pubs[pub] = rospy.Publisher(pub, Float64MultiArray, queue_size=1)

	def publish_debug(self):
		for topic in self.debug_pubs:
			if topic == 'error':
				msg = Float64MultiArray(self.error)
				self.debug_pubs[topic].publish(msg)
			if topic == 'target_velocity':
				msg = Float64MultiArray(self.trajectory[0])
				self.debug_pubs[topic].publish(msg)
			if topic == 'target_acceleration':
				msg = Float64MultiArray(self.trajectory[1])
				self.debug_pubs[topic].publish(msg)

	def detected_callback(self, msg):
		# for now. need to figure out how to get accurate time between messages?
		# Build a custom message that has a timestamp
		t = time.time()

		# do tracker stuff
		measure = np.array(msg.data)
		self.update_prev(measure)
		self.update_deltas(t - self.t)
		self.t = t

		if self.debug:
			self.update_error()


	def update_error(self):
		self.error = np.linalg.norm(self.pose - self.history[0])

	def update_deltas(self, dt):

		# Get deltas and pose at t
		velocity, acceleration, jerk = self.trajectory
		v1 = velocity
		a1 = acceleration

		velocity = (self.history[0] - self.history[1]) / dt
		acceleration = (velocity - v1) / dt
		jerk = (acceleration - a1) / dt

		# save updates
		self.trajectory = np.array([velocity, acceleration, jerk])


	def predict(self):
		dt = (time.time() - self.t) + self.t_offset
		t_vect = np.array([[dt, dt**2 / 2, dt**3 / 6],
							[dt, dt**2 / 2, dt**3 / 6]])

		return self.pose + np.dot(self.trajectory, t_vect)

	def publish_prediction(self):
		prediction = self.predict()
		msg = Float64MultiArray(prediction)
		self.prediction_pub.publish(msg)


	def spin(self):

		while not rospy.is_shutdown():
			self.publish_prediction()
			if self.debug:
				self.publish_debug()
			rospy.sleep(self.rate)


def main(config_data):

	if config_data is None:
		return

	tracker = Tracker(config_data=config_data)

	while not rospy.is_shutdown():
		tracker.spin()


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print(f'No Data: BuffNet exiting ...')

	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))

	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)

		main(data)

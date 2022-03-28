#! /usr/bin/env python3
import os
import cv2
import sys
import time
import rospy
import numpy as np
import buffvision as bv
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
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

		# camera heading
		self.phi = 0.0
		self.psi = 0.0
		self.FOV = np.radians(120)

		# armour photogrammetry
		self.m = 125.0
		self.b = 50.0

		# input bounds
		self.image_size = (300, 300, 3)

		self.bridge = CvBridge()

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
			subs[0], Float64MultiArray, self.detection_callback, queue_size=1)

		self.prediction_pub = rospy.Publisher(
			pubs[0], Float64MultiArray, queue_size=1)

		self.debug_pubs[pubs[1]] = rospy.Publisher(pubs[1], Float64MultiArray, queue_size=1)
		self.debug_pubs[pubs[2]] = rospy.Publisher(pubs[2], Image, queue_size=1)


	def publish_debug(self):
		"""
		Publish the measured error of predictions and a map
		of the detected targets
		"""
		for topic in self.debug_pubs:
			if topic == 'tracking_error':
				msg = Float64MultiArray(data=self.error)
				self.debug_pubs[topic].publish(msg)

			if topic == 'target_map':
				d = (self.image_size[0] * 0.075, self.image_size[1] * 0.075)
				image = np.ones(self.image_size, dtype=np.uint8) * 255
				origin = (int(self.image_size[0] / 2), int(self.image_size[1] / 2))
				bot_1 = (int(origin[0] - d[0]), int(origin[1] - d[1]))
				bot_2 = (int(origin[0] + d[0]), int(origin[1] + d[1]))
				fovr = (int(origin[0] + (4 * d[0] * np.cos(self.psi + (self.FOV / 2)))), int(origin[1] + (4 * d[1] * np.sin(self.psi + (self.FOV / 2)))))
				fovl = (int(origin[0] + (4 * d[0] * np.cos(self.psi - (self.FOV / 2)))), int(origin[1] + (4 * d[1] * np.sin(self.psi - (self.FOV / 2)))))

				image = cv2.rectangle(image, bot_1, bot_2, (0,0,0), 2)
				image = cv2.line(image, origin, fovl, (255,0,0))
				image = cv2.line(image, origin, fovr, (255,0,0))
				for (x,y) in self.history:
					target = (int(origin[0] + x), int(origin[1] + y))
					image = cv2.circle(image, target, 10, (0,255,0), 2)

				x,y = self.pose
				pose = (int(origin[0] + x), int(origin[1] + y))
				image = cv2.circle(image, pose, 10, (0,0,255))

				msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
				self.debug_pubs[topic].publish(msg)



	def publish_prediction(self):
		"""
		Publish the yaw and pitch to aim at the target
		"""
		self.predict()
		psi = np.arctan(self.pose[1] / self.pose[0]) # arctan of x,y is yaw
		phi = 0.05 * np.linalg.norm(self.pose) # phi is function of distance
		msg = Float64MultiArray(data=[phi, psi])
		self.prediction_pub.publish(msg)

	def detection_callback(self, msg):
		# for now. need to figure out how to get accurate time between messages?
		# Build a custom message that has a timestamp
		t = time.time()
		# do tracker stuff
		pose = np.array(msg.data)
		measure = self.project(pose)
		self.history = [measure, self.history[0], self.history[1], self.history[2]]
		self.update_trajectory(t - self.t)
		self.psi += np.pi / 200
		self.psi %= 2 * np.pi
		self.t = t

		if self.debug:
			self.update_error()

	def project(self, pose):
		"""
		Projects a detection into the body frame
		"""
		d = (self.m / pose[3]) + self.b
		alpha = ((pose[0] / self.image_size[0]) - 0.5) * self.FOV
		return d * np.array([np.cos(self.psi + alpha), np.sin(self.psi + alpha)])

	def predict(self):
		dt = (time.time() - self.t) + self.t_offset
		t_vect = np.array([dt, dt**2 / 2, dt**3 / 6])

		self.pose = np.array(self.history[0]) + np.dot(t_vect, self.trajectory)[0]

	def update_error(self):
		self.error = np.linalg.norm(self.pose - self.history[0])

	def update_trajectory(self, dt):

		# Get deltas and pose at t
		velocity, acceleration, jerk = self.trajectory
		v1 = velocity
		a1 = acceleration

		velocity = (self.history[0] - self.history[1]) / dt
		acceleration = (velocity - v1) / dt
		jerk = (acceleration - a1) / dt

		# save updates
		self.trajectory = np.array([velocity, acceleration, jerk])

	def spin(self):

		while not rospy.is_shutdown():
			self.publish_prediction()
			if self.debug:
				self.publish_debug()
			self.rate.sleep()


def main(config_data):

	if config_data is None:
		return

	tracker = Dead_Reckon_Tracer(config_data=config_data)

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

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

"""
	Dead Reckoning Tracker class
	Uses dead-reckoning to track detected objects
"""


class Dead_Reckon_Tracer:
	def __init__(self, config_data):

		self.t = 0
		self.error = 0
		self.measure = None
		self.m = config_data['M']
		self.b = config_data['B']
		self.a = config_data['A']
		self.d_scale = config_data['DSCALE']
		self.t_offset = config_data['LEAD_TIME']

		# camera heading
		self.phi = 0.0
		self.psi = 0.0

		params = rospy.get_param('/buffbot')
		if 'LUXONIS' in params:
			self.FOV = params['LUXONIS']['CAMERA']['FOV']
			# input bounds
			image_res = params['LUXONIS']['CAMERA']['RESOLUTION']
			self.image_size = (image_res, image_res, 3)

		self.bridge = CvBridge()

		# Initialize the estimated postion at current time + offset
		self.pose = np.zeros(2, dtype=np.float64)
		# Initialize the history of measurements
		self.history = np.ones((4,2), dtype=np.float64) * -1
		# Initialize the trajectory of the target
		self.trajectory = np.zeros((3,2), dtype=np.float64)

		self.debug_pubs = {}        
		topics = rospy.get_param('/buffbot/TOPICS')
		self.debug = rospy.get_param('/buffbot/DEBUG')

		rospy.init_node('dr_tracer', anonymous=True)
		self.rate = rospy.Rate(30)

		# self.detect_sub = rospy.Subscriber(
		# 	topics['DETECTION'], Float64MultiArray, self.detection_callback, queue_size=5)

		self.aim_heading_sub = rospy.Subscriber(
			topics['AIM_HEADING'], Float64MultiArray)

		self.prediction_pub = rospy.Publisher(
			topics['TARGET'], String, queue_size=1)

		if self.debug:
			self.map_pub = rospy.Publisher(topics['TARGET_MAP'], Image, queue_size=1)
			self.tracking_err_pub = rospy.Publisher(topics['TRACKING_ERROR'], Float64MultiArray, queue_size=1)


	def publish_debug(self):
		"""
		Publish the measured error of predictions and a map
		of the detected targets
		"""
			
		msg = Float64MultiArray(data=self.error)
		self.tracking_err_pub.publish(msg)

		d = (self.image_size[0] * 0.02, self.image_size[1] * 0.02)
		image = np.ones(self.image_size, dtype=np.uint8) * 255
		origin = (int(self.image_size[0] / 2), int(self.image_size[1] / 2))
		bot_1 = (int(origin[0] - d[0]), int(origin[1] - d[1]))
		bot_2 = (int(origin[0] + d[0]), int(origin[1] + d[1]))
		fovr = (int(origin[0] + (5 * d[0] * np.cos(self.psi + np.radians(self.FOV / 2)))), int(origin[1] + (5 * d[1] * np.sin(self.psi + np.radians(self.FOV / 2)))))
		fovl = (int(origin[0] + (5 * d[0] * np.cos(self.psi - np.radians(self.FOV / 2)))), int(origin[1] + (5 * d[1] * np.sin(self.psi - np.radians(self.FOV / 2)))))

		image = cv2.rectangle(image, bot_1, bot_2, (0,0,0), 2)
		image = cv2.line(image, origin, fovl, (255,0,0))
		image = cv2.line(image, origin, fovr, (255,0,0))
		for (x,y) in self.history:
			if x >= 0 and y >=0:
				target = (int(origin[0] + x), int(origin[1] + y))
				image = cv2.circle(image, target, 10, (0,255,0), 2)

		x,y = self.pose
		pose = (int(origin[0] + x), int(origin[1] + y))
		image = cv2.circle(image, pose, 10, (0,0,255))

		msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
		self.map_pub.publish(msg)



	def publish_prediction(self):
		"""
		Publish the yaw and pitch to aim at the target
		"""
		self.predict()
		psi = np.arctan(self.pose[1] / self.pose[0]) # arctan of x,y is yaw
		phi = self.d_scale * np.linalg.norm(self.pose) # phi is this needs to be tuned function of distance
		msg = String(f'aim_control:{phi},{psi}')
		self.prediction_pub.publish(msg)

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
		

	def project(self, pose):
		"""
		Projects a detection into the body frame
		PARAMS:
			pose: Float64MultiArray.data, [x,y,h,w,cf,cl] (detection msg)
		RETURNS:
			vector (x,y): body frame position of the detection
		"""
		d = self.a * np.exp(self.m * (pose[2] + self.b))
		alpha = np.radians((pose[0] / self.image_size[0]) * self.FOV) / 2
		return d * np.array([np.cos(self.psi + alpha), np.sin(self.psi + alpha)])

	def predict(self):
		"""
		Predict the target postion t_offset in the future. Uses dead reckoning.
		"""
		dt = (time.time() - self.t) + self.t_offset
		t_vect = np.array([dt, dt**2 / 2, dt**3 / 6])

		self.pose = np.array(self.history[0]) + np.dot(t_vect, self.trajectory)[0]

	def update_error(self):
		"""
		Calculate the error of an estimate
		"""
		self.error = np.linalg.norm(self.pose - self.history[0])

	def update_trajectory(self, dt):
		"""
		Given a new measurement, update the trajectory
		PARAMS:
			dt: float, time since last measurment
		"""
		# Get deltas and pose at t
		velocity, acceleration, jerk = self.trajectory
		pose = self.history[0]
		p_last = self.history[1]

		v_last = velocity
		a_last = acceleration

		# update trajectory
		if p_last[0] < 0:
			pass
		elif self.history[2][0] < 0:
			self.trajectory = np.array([(pose - p_last), acceleration * dt, jerk * dt]) / dt
		elif self.history[3][0] < 0:
			self.trajectory = np.array([(pose - p_last), (velocity - v_last), jerk * dt]) / dt
		else:
			self.trajectory = np.array([(pose - p_last), (velocity - v_last), (acceleration - a_last)]) / dt
		
		

	def spin(self):

		while not rospy.is_shutdown():

			if time.time() - self.t > 5:
				self.history = np.ones((4,2), dtype=np.float64) * -1
				self.trajectory = np.zeros((3,2), dtype=np.float64)

			self.publish_prediction()

			if not self.measure is None:
				t = time.time()
				self.history = [self.measure, self.history[0], self.history[1], self.history[2]]
				self.update_trajectory(t - self.t)
				# update last measurement time
				self.t = t
				self.measure = None

				# If debug mode update the calculated error
				if self.debug:
					self.update_error()

			if self.debug:
				self.publish_debug()
			self.rate.sleep()


def main(config_data):
	exit(0)
	tracker = Dead_Reckon_Tracer(config_data)
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
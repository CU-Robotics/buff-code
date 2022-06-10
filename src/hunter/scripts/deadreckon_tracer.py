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
	"""
	Dead Reckon Tracer
	dead reckoning means filtering a signal using
	the most basic equations of motion. In this case
	we use Newtons Equations:
	Xt = X0 + Vdt + 1/2Adt^2
	This class is basically just an integrator.
	The tracer uses previous measurements to calculate
	the trajectory of the function. Then using this trajectory 
	the tracer can predict where the measured objects will be.
	This class assumes the detections it recieves lay in a 2D
	grid world. To the tracer the world is unbounded, this may 
	cause problems in the future.
	"""
	
	def __init__(self, name):
		self.pose = None
		self.history = None
		self.t = time.time() 
		self.old_t = time.time()
		self.trajectory = np.zeros((3,2), dtype=np.float64)

		self.init_ros(name)

	def init_ros(self, name):

		rospy.init_node('tracker', anonymous=True)

		self.r_threshold = rospy.get_param('/buffbot/TRACKER/MATCH_THRESHOLD')
		self.d_offset = rospy.get_param('/buffbot/TRACKER/SHOOTER_DROP')
		self.t_offset = rospy.get_param('/buffbot/TRACKER/LEAD_TIME')
		self.FOV = rospy.get_param('/buffbot/CAMERA/FOV')
		
		hz = rospy.get_param('/buffbot/CAMERA/FPS')
		self.rate = rospy.Rate(hz / 2)
		
		self.debug = rospy.get_param('/buffbot/DEBUG')
		topics = rospy.get_param('/buffbot/TOPICS')

		detection_topic = rospy.get_param(f'{name}/DETECTION_TOPIC')

		self.detect_sub = rospy.Subscriber(
			topics[detection_topic], Float64MultiArray, self.detection_callback, queue_size=5)

		self.prediction_pub = rospy.Publisher(
			topics['SERIAL_OUT'], String, queue_size=1)
		
		if self.debug:
			self.error_pub = rospy.Publisher(
				topics['TRACKING_ERROR'], Float64MultiArray, queue_size=1)

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
		detections = np.array(msg.data)
		closest_det = np.ones(4) / 2
		closest_det[3] = 0

		for detection in detections.reshape(round(len(detections)/4), 4):
			if detection[3] > closest_det[3]:
				closest_det = detection

		self.update_trajectory(t, np.array(closest_det[:2]))

	def reset(self):
		self.history = None
		self.trajectory = np.zeros((3,2), dtype=np.float64)

	def predict(self, t, pose=None):
		"""
			make a prediction of where the target will be.
			t: float, simulation time to find position at
			
		"""
		dt = t - self.t + self.t_offset
		t_vect = np.array([dt, dt**2 / 2, dt**3 / 6])

		# vdt + .5adt^2 + .1667jdt^3
		# [xt,yt] = [x0,y0] + [dt, .5dt**2, .1667dt*3]*[v;a;j]    
		if not pose is None:
			return pose + (t_vect @ self.trajectory)
		else:
			# print(self.trajectory)
			self.pose = self.history[0] + (t_vect @ self.trajectory)

	def publish_error(self):
		error = self.history[0] - self.predict((2 * self.t) - self.old_t - self.t_offset, pose=self.history[1])
		msg = Float64MultiArray(data=error)
		self.error_pub.publish(msg)

	def update_trajectory(self, t, measure, debug=False):
		"""
			update the trajectory of a target, tracks up to a third derivative (jerk)
			t: float, time in simulation
		"""
		dt = t - self.t
		if dt == 0:
			return

		if self.history is None:
			self.history = np.zeros((4,2), dtype=np.float64)
			self.history[0] = measure
			self.history[1] = measure

		else:
			if np.linalg.norm(measure - self.history[0]) > self.r_threshold:
				self.trajectory = np.zeros((3,2), dtype=np.float64)
				self.history[0] = measure

			self.history[1:] =  self.history[:3]
			self.history[0] = measure

		velocity, acceleration, jerk = self.trajectory

		v1 = velocity
		a1 = acceleration

		velocity = (self.history[0] - self.history[1]) / dt
		self.old_t = self.t
		self.t = t

		# acceleration = (velocity - v1) / dt
		# jerk = (acceleration - a1) / dt

		# save updates
		self.trajectory = np.array([velocity, acceleration, jerk])

	def spin(self):
		while not rospy.is_shutdown():

			if time.time() - self.t > 5:
				self.reset()

			if not self.history is None:

				self.predict(time.time())

				phi_err = (self.pose[1] - self.d_offset) * self.FOV
				psi_err = (self.pose[0] - 0.5) * self.FOV
				msg = String(f'GW {round(psi_err * 100)}') # GH {round(phi_err * 100)} 
				self.prediction_pub.publish(msg)

				if self.debug:
					self.publish_error()

			self.rate.sleep()

def main(name):
	tracker = Dead_Reckon_Tracer(name)

	tracker.spin()


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print(f'No Data: Tracker exiting ...')

	elif '/buffbot' in sys.argv[1]:
		main(sys.argv[1])


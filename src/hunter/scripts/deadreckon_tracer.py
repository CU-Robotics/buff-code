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
	
	def __init__(self, data, debug=False):
		self.pose = None
		self.history = None
		self.t = time.time() 
		self.old_t = time.time()
		self.trajectory = np.zeros((3,2), dtype=np.float64)

		self.r_threshold = data['MATCH_THRESHOLD']
		self.t_offset = data['LEAD_TIME']
		self.d_offset = data['SHOOTER_DROP']
		self.rate_hz = data['RATE']

		self.init_ros(data)

	def init_ros(self, data):

		rospy.init_node('tracker', anonymous=True)
		self.rate = rospy.Rate(data['RATE'])
		
		self.debug = rospy.get_param('/buffbot/DEBUG')
		topics = rospy.get_param('/buffbot/TOPICS')

		self.detect_sub = rospy.Subscriber(
			topics['DETECTION_WORLD'], Float64MultiArray, self.detection_callback, queue_size=5)

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
		measure = np.array(msg.data)
		self.update_trajectory(t, measure)

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
			self.history[0] = np.array(measure).reshape(2)

		self.history[1:] =  self.history[:3]
		self.history[0] = np.array(measure).reshape(2)

		if np.linalg.norm(self.history[0] - self.history[1]) > self.r_threshold:
			self.reset()
			return

		velocity, acceleration, jerk = self.trajectory

		if debug:
			self.error.append()

		v1 = velocity
		a1 = acceleration

		velocity = (self.history[0] - self.history[1]) / dt
		self.old_t = self.t
		self.t = t

		acceleration = (velocity - v1) / dt
		jerk = (acceleration - a1) / dt

		# save updates
		self.trajectory = np.array([velocity, acceleration, jerk])

	def spin(self):

		while not rospy.is_shutdown():

			if time.time() - self.t > 5:
				self.reset()

			if not self.history is None:

				self.predict(time.time())
				psi = np.arctan(self.pose[1] / self.pose[0]) # arctan of x,y is yaw
				phi = self.d_offset * np.linalg.norm(self.pose) # phi is this needs to be tuned function of distance
				msg = String(f'GPR {phi} GYR {psi}')
				self.prediction_pub.publish(msg)

				if self.debug:
					self.publish_error()

			self.rate.sleep()

def main(data):
	tracker = Dead_Reckon_Tracer(data)
	tracker.spin()


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print(f'No Data: Tracker exiting ...')

	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))

	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)

		main(data)
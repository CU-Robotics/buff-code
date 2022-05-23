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
	def __init__(self, debug=False):
		self.t = 0 
		self.debug = False
		self.error = [[0,0]]

		self.pose = None
		self.history = np.zeros((4,2), dtype=np.float64)
		self.trajectory = np.zeros((3,2), dtype=np.float64)

	def predict(self, dt, pose=None):
		"""
			make a prediction of where the target will be.
			t: float, simulation time to find position at
			
		"""
		t_vect = np.array([dt, dt**2 / 2, dt**3 / 6])

		# vdt + .5adt^2 + .1667jdt^3
		# [xt,yt] = [x0,y0] + [dt, .5dt**2, .1667dt*3]*[v;a;j]    
		if not pose is None:
			return pose + (t_vect @ self.trajectory)
		else:
			self.pose = self.history[0] + t_vect @ self.trajectory

	def update_trajectory(self, t, measure, debug=False):
		"""
			update the trajectory of a target, tracks up to a third derivative (jerk)
			t: float, time in simulation
		"""
		dt = t - self.t
		if dt == 0:
			return

		self.history[1:] =  self.history[:3]
		self.history[0] = np.array(measure)

		velocity, acceleration, jerk = self.trajectory

		if debug:
			self.error.append(self.history[0] - self.predict(dt, pose=self.history[1]))

		v1 = velocity
		a1 = acceleration

		velocity = (self.history[0] - self.history[1]) / dt
		self.t = t

		acceleration = (velocity - v1) / dt
		jerk = (acceleration - a1) / dt

		# save updates
		self.trajectory = np.array([velocity, acceleration, jerk])

 #  def spin(self):

	# while not rospy.is_shutdown():

	# 	if time.time() - self.t > 5:
	# 		self.trajectory *= 0

	# 	self.publish_prediction()

	# 	if not self.measure is None:
	# 		t = time.time()
	# 		self.history = [self.measure, self.history[0], self.history[1], self.history[2]]
	# 		self.update_trajectory(t - self.t, self.measure)
	# 		# update last measurement time
	# 		self.t = t
	# 		self.measure = None

	# 		# If debug mode update the calculated error
	# 		if self.debug:
	# 			self.update_error()

	# 	if self.debug:
	# 		self.publish_debug()
	# 	self.rate.sleep()
		

# class Dead_Reckon_Tracer:
# 	def __init__(self, config_data):

# 		self.t = 0
# 		self.error = 0
# 		self.measure = None
		
# 		self.d_scale = config_data['DSCALE']
# 		self.t_offset = config_data['LEAD_TIME']

# 		params = rospy.get_param('/buffbot')
# 		if 'LUXONIS' in params:
# 			self.FOV = params['LUXONIS']['CAMERA']['FOV']
# 			# input bounds
# 			image_res = params['LUXONIS']['CAMERA']['RESOLUTION']
# 			self.image_size = (image_res, image_res, 3)

# 		self.bridge = CvBridge()

# 		# Initialize the estimated postion at current time + offset
# 		self.pose = np.zeros(2, dtype=np.float64)
# 		# Initialize the history of measurements
# 		self.history = np.ones((4,2), dtype=np.float64) * -1
# 		# Initialize the trajectory of the target
# 		self.trajectory = np.zeros((3,2), dtype=np.float64)


# 	def publish_debug(self):
# 		"""
# 		Publish the measured error of predictions and a map
# 		of the detected targets
# 		"""
			
# 		msg = Float64MultiArray(data=self.error)
# 		self.tracking_err_pub.publish(msg)

# 		d = (self.image_size[0] * 0.02, self.image_size[1] * 0.02)
# 		image = np.ones(self.image_size, dtype=np.uint8) * 255
# 		origin = (int(self.image_size[0] / 2), int(self.image_size[1] / 2))
# 		bot_1 = (int(origin[0] - d[0]), int(origin[1] - d[1]))
# 		bot_2 = (int(origin[0] + d[0]), int(origin[1] + d[1]))
# 		fovr = (int(origin[0] + (5 * d[0] * np.cos(self.psi + np.radians(self.FOV / 2)))), int(origin[1] + (5 * d[1] * np.sin(self.psi + np.radians(self.FOV / 2)))))
# 		fovl = (int(origin[0] + (5 * d[0] * np.cos(self.psi - np.radians(self.FOV / 2)))), int(origin[1] + (5 * d[1] * np.sin(self.psi - np.radians(self.FOV / 2)))))

# 		image = cv2.rectangle(image, bot_1, bot_2, (0,0,0), 2)
# 		image = cv2.line(image, origin, fovl, (255,0,0))
# 		image = cv2.line(image, origin, fovr, (255,0,0))
# 		for (x,y) in self.history:
# 			if x >= 0 and y >=0:
# 				target = (int(origin[0] + x), int(origin[1] + y))
# 				image = cv2.circle(image, target, 10, (0,255,0), 2)

# 		x,y = self.pose
# 		pose = (int(origin[0] + x), int(origin[1] + y))
# 		image = cv2.circle(image, pose, 10, (0,0,255))

# 		msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
# 		self.map_pub.publish(msg)

# 	def predict(self):
# 		"""
# 		Predict the target postion t_offset in the future. Uses dead reckoning.
# 		"""
# 		dt = (time.time() - self.t) + self.t_offset
# 		t_vect = np.array([dt, dt**2 / 2, dt**3 / 6])

# 		self.pose = np.array(self.history[0]) + (t_vect @ self.trajectory)

# 	def update_error(self):
# 		"""
# 		Calculate the error of an estimate
# 		"""
# 		self.error = np.linalg.norm(self.pose - self.history[0])

# 	def update_trajectory(self, dt):
# 		"""
# 		Given a new measurement, update the trajectory
# 		PARAMS:
# 			dt: float, time since last measurment
# 		"""
# 		# Get deltas and pose at t
# 		velocity, acceleration, jerk = self.trajectory
# 		pose = self.history[0]
# 		p_last = self.history[1]

# 		v_last = velocity
# 		a_last = acceleration

# 		# update trajectory
# 		if p_last[0] < 0:
# 			pass
# 		elif self.history[2][0] < 0:
# 			self.trajectory = np.array([(pose - p_last), acceleration * dt, jerk * dt]) / dt
# 		elif self.history[3][0] < 0:
# 			self.trajectory = np.array([(pose - p_last), (velocity - v_last), jerk * dt]) / dt
# 		else:
# 			self.trajectory = np.array([(pose - p_last), (velocity - v_last), (acceleration - a_last)]) / dt
		
		

# 	def spin(self):

# 		while not rospy.is_shutdown():

# 			if time.time() - self.t > 5:
# 				self.history = np.ones((4,2), dtype=np.float64) * -1
# 				self.trajectory = np.zeros((3,2), dtype=np.float64)

# 			self.publish_prediction()

# 			if not self.measure is None:
# 				t = time.time()
# 				self.history = [self.measure, self.history[0], self.history[1], self.history[2]]
# 				self.update_trajectory(t - self.t)
# 				# update last measurement time
# 				self.t = t
# 				self.measure = None

# 				# If debug mode update the calculated error
# 				if self.debug:
# 					self.update_error()

# 			if self.debug:
# 				self.publish_debug()
# 			self.rate.sleep()


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
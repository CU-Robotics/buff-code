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
	Project pixel coordinates to the world
	
"""

class Target_Map:
	def __init__(self, data):
		self.phi = 0.0
		self.psi = 0.0
		self.t = time.time()
		self.bridge = CvBridge()
		self.r = np.zeros(2, dtype=np.float64)
		self.history = np.zeros((4,2), dtype=np.float64)

		self.rate = data['RATE']
		self.map_size = data['MAP_SIZE']

		self.FOV = rospy.get_param('/buffbot/CAMERA/FOV')

		self.init_ros(data)

	def init_ros(self, data):
		rospy.init_node('projector', anonymous=True)
		
		self.debug = rospy.get_param('/buffbot/DEBUG')
		topics = rospy.get_param('/buffbot/TOPICS')

		self.target_sub = rospy.Subscriber(
			topics['DETECTION_WORLD'], Float64MultiArray, self.target_callback)

		self.gimbal_sub = rospy.Subscriber(
			topics['GIMBAL_STATE'], Float64MultiArray, self.gimbal_callback, queue_size=1)

		self.map_pub = rospy.Publisher(
			topics['TARGET_MAP'], Image, queue_size=1)

	def target_callback(self, msg):
		"""
		Parse a detection msg
		PARAMS:
			msg: Float64MultiArray, detection msg, data=[x,y,w,h,cf,cl]
		"""
		# do projector stuff
		self.t = time.time()
		self.r = np.array(msg.data)
		self.history[1:] = self.history[:3]
		self.history[0] = self.r

	def gimbal_callback(self, msg):
		state = msg.data
		self.psi = state[2]
		self.phi = state[1]

	def get_class_color(cl):
		colors = [[255, 0, 0], [255, 0, 0], [0, 255, 0]]
		return np.array(colors[cl])

	def publish_map(self):

		d = (self.map_size[0] * 0.02, self.map_size[1] * 0.02)
		image = np.ones(self.map_size, dtype=np.uint8) * 255
		origin = (int(self.map_size[0] / 2), int(self.map_size[1] / 2))
		bot_1 = (int(origin[0] - d[0]), int(origin[1] - d[1]))
		bot_2 = (int(origin[0] + d[0]), int(origin[1] + d[1]))
		fovr = (int(origin[0] + (5 * d[0] * np.cos(np.radians(self.psi + (self.FOV / 2))))), int(origin[1] - (5 * d[1] * np.sin(np.radians(self.psi + (self.FOV / 2))))))
		fovl = (int(origin[0] + (5 * d[0] * np.cos(np.radians(self.psi - (self.FOV / 2))))), int(origin[1] - (5 * d[1] * np.sin(np.radians(self.psi - (self.FOV / 2))))))

		image = cv2.rectangle(image, bot_1, bot_2, (0,0,0), 2)
		image = cv2.line(image, origin, fovl, (255,0,0))
		image = cv2.line(image, origin, fovr, (255,0,0))

		if not self.r is None:
			for i, (c,x,y) in enumerate(self.history):
				color = get_class_color(c)
				target = (int(origin[0] + x), int(origin[1] + y))
				image = cv2.circle(image, target, 10, color * i / 5, 2)

		msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
		self.map_pub.publish(msg)

	def spin(self):
		while not rospy.is_shutdown():
			t = time.time()

			if t - self.t > 5:
				self.r = None

			self.publish_map()


def main(data):
	t_map = Target_Map(data)

	try:
		t_map.spin()

	except KeyboardInterrupt as e:
		print('Target Map killed')

	except Exception as e:
		print(e)


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print(f'No Data: Target Map exiting ...')

	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))

	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)

		main(data)




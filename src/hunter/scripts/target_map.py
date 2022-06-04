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
	def __init__(self, name):
		self.phi = 0.0
		self.psi = 0.0
		self.t = time.time()
		self.bridge = CvBridge()
		self.r = np.zeros(2, dtype=np.float64)
		self.map = None

		self.init_ros(name)
		self.draw_map()

	def init_ros(self, name):
		rospy.init_node('projector', anonymous=True)

		self.rate = rospy.Rate(rospy.get_param('/buffbot/CAMERA/FPS'))
		self.map_size = rospy.get_param(f'{name}/MAP_SIZE')
		self.FOV = rospy.get_param('/buffbot/CAMERA/FOV')
		self.FPS = rospy.get_param('/buffbot/CAMERA/FPS')
		self.debug = rospy.get_param('/buffbot/DEBUG')

		n_classes = rospy.get_param('/buffbot/MODEL/NCLASSES')
		self.history = [None for i in range(n_classes)]

		topics = rospy.get_param('/buffbot/TOPICS')

		self.target_sub = rospy.Subscriber(
			topics['DETECTION_WORLD'], Float64MultiArray, self.target_callback, queue_size=5)

		self.gimbal_sub = rospy.Subscriber(
			topics['GIMBAL_STATE'], Float64MultiArray, self.gimbal_callback, queue_size=1)

		self.map_pub = rospy.Publisher(
			topics['TARGET_MAP'], Image, queue_size=1)

	def draw_map(self):

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

		for i in range(len(self.history)):
			if not self.history[i] is None:
				for j, (x,y) in enumerate(self.history[i]):
					color = self.get_class_color(i)
					target = (int(origin[0] + x), int(origin[1] + y))
					image = cv2.circle(image, target, 10, color * 5 / (j+1), 2)

		self.map = image

	def target_callback(self, msg):
		"""
		Parse a detection msg
		PARAMS:
			msg: Float64MultiArray, detection msg, data=[x,y,w,h,cl]
		"""
		# do projector stuff
		self.t = time.time()
		detections = np.array(msg.data)

		for detection in detections.reshape((round(len(detections) / 3), 3)):
			c = int(detection[0])
			if not self.history[c] is None:
				self.history[c].insert(0,detection[1:])
				while len(self.history[c]) > 4:
					self.history[c].pop()

	def gimbal_callback(self, msg):
		state = msg.data
		self.psi = state[2]
		self.phi = state[1]

	def get_class_color(self, cl):
		colors = [[0, 0, 255], [255, 0, 0], [0, 255, 0]]
		return np.array(colors[cl])

	def publish_map(self):

		msg = self.bridge.cv2_to_imgmsg(self.map, encoding='rgb8')
		self.map_pub.publish(msg)

	def spin(self):
		while not rospy.is_shutdown():
			t = time.time()

			if t - self.t > 5:
				self.r = None

			self.publish_map()


def main(name):
	t_map = Target_Map(name)

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
		main(sys.argv[1])

	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)

		main(data)




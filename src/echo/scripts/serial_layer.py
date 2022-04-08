#! /usr/bin/env python3
import sys
import time
import rospy
import serial
import numpy as np
import traceback as tb
from std_msgs.msg import String, Float64MultiArray


class SerialLayer():
	def __init__(self, config_data):

		self.lives = 4

		self.timeout = 0
		self.debug = False
		self.device = None
		self.baudrate = -1
		self.publishers = {}
		self.subscribers = {}
		self.connected = False
		self.port = '/dev/ttyACM0'

		self.debug = rospy.get_param('/buffbot/DEBUG')

		self.port = config_data['PORT']
		self.timeout = config_data['TIMEOUT']
		self.baudrate = config_data['BAUDRATE']

		self.try_connect()

		topics = rospy.get_param('/buffbot/TOPICS')
		self.aim_sub = rospy.Subscriber(topics['TARGET'], String, self.writer_callback, queue_size=1)

		rospy.init_node('echo-serial', anonymous=True)

		self.publishers['aim'] = rospy.Publisher(topics['AIM_HEADING'], Float64MultiArray, queue_size=1)


	def write_device(self, packet):
		"""
		  Write a packet to the teensy
		"""
		if not self.device is None:
			self.device.write(packet)


	def writer_callback(self, msg):
		"""
		  Callback for writing messages to the teensy
		"""
		s = ''
		for l in msg.data:
			s += str(l)
		self.write_device(bytes(s, 'utf-8'))


	def try_connect(self):
		"""
		  Try to connect with the device.
		"""
		try:
			if self.device:
				self.device.close()

			self.device = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
			self.device.flush()

		except Exception as e:
			rospy.logerr(e)
			rospy.logerr(f"Serial lives left {self.lives}")
			self.lives -= 1
			if self.lives < 1:
				return False

		else:
			rospy.loginfo('Device Connected: Reading...')


	def parse_packet(self):
		"""
		  Create a new msg to publish.
			Params:
				topic: string - destination for the publisher
				msgType: string - the datatype of the msg
				data: ??? - the data to fill msg
			Returns:
			  msg: ??? - the msg to send
		"""
		packet = self.device.readline().decode().rstrip().split(',')
		
		if self.debug:
			rospy.loginfo(packet)

		for item in packet:
			name, val = item.split(':')
			
			if not name in self.publishers:
				self.publishers[name] = rospy.Publisher(name, Float64MultiArray, queue_size=10)

			msg = Float64MultiArray(data=np.array(val.split(','), dtype=np.float64))
			self.publishers[name].publish(msg)


	def spin(self):
		"""
		  Read a line from the serial port and publish it
		to a topic given the mode. The mode is the first 4 bytes
		of the msg.
		"""
		while not rospy.is_shutdown():
			try:
				if not self.device:
					if not self.try_connect():
						return

				elif self.device.in_waiting:
					self.parse_packet()

			except KeyboardInterrupt:
				if self.device:
					self.device.close()
				return

			except Exception as e:
				tb.print_exc()
				print(e)
				if self.device:
					self.device.close()
				print('Possible I/O Disconnect: Reseting...')
				time.sleep(2)
				if not self.try_connect():
					return


def main(config_data):

	layer = SerialLayer(config_data)
	layer.spin()

		


if __name__=='__main__':
	if len(sys.argv) < 2:
		print(f'No Data: Serial Layer exiting')
	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))
	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)
		main(data)


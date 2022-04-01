#! /usr/bin/env python3
import sys
import time
import rospy
import serial
import traceback as tb
from std_msgs.msg import Float64MultiArray


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

		if 'DEBUG' in config_data:
			self.debug = config_data['DEBUG']

		if 'PORT' in config_data:
			self.port = config_data['PORT']

		if 'TIMEOUT' in config_data:
			self.timeout = config_data['TIMEOUT']

		if 'BAUDRATE' in config_data:
			self.baudrate = config_data['BAUDRATE']

		if 'TOPICS' in config_data: # Set up ros publishers
			# If topics probably debug ...?
			debug = rospy.get_param('/buffbot/DEBUG')

			topics_names = config_data['TOPICS']
			all_topics = rospy.get_param('/buffbot/TOPICS')
			# if 'PUBLISH' in config_data['TOPICS']:
			# 	for tn in topics_names['PUBLISH']:
			# 		topic = all_topics[tn]
			# 		self.publishers[topic] = rospy.Publisher(topic, Float64, queue_size=10)

			if 'SUBSCRIBE' in config_data['TOPICS']:
				for tn in topics_names['SUBSCRIBE']:
					topic = all_topics[tn]
					self.subscribers[topic] = rospy.Subscriber(topic, Float64MultiArray, self.writer_callback, queue_size=1)

		# for now require roscore to run (maybe later set up non-ros runtime)
		rospy.init_node('echo-serial', anonymous=True)

		self.try_connect()


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
			print(f'Error Detecting Device at {self.port}')
			print(e)
			
			self.lives -= 1
			if self.lives < 1:
				exit(0)

		else:
			print('Device Connected: Reading...')


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
				self.publishers[name] = rospy.Publisher(name, Float64, queue_size=10)

			# rospy.loginfo(val)
			msg = Float64(float(val))
			self.publishers[name].publish(msg)


	def spin(self):
		"""
		  Read a line from the serial port and publish it
		to a topic given the mode. The mode is the first 4 bytes
		of the msg.
		"""
		try:
			if not self.device:
				self.try_connect()

			elif self.device.in_waiting:
				self.parse_packet()

		except Exception as e:
			tb.print_exc()
			print(e)
			print('Possible I/O Disconnect: Reseting...')
			time.sleep(2)
			self.try_connect()


def main(data):

	try:
		layer = SerialLayer(data)

		while not rospy.is_shutdown():
			layer.spin()

	except KeyboardInterrupt:
		if layer.device:
			layer.device.close()

	except Exception as e:
		tb.print_exc()
		if layer.device:
			layer.device.close()


if __name__=='__main__':
	if len(sys.argv) < 2:
		print(f'No Data: Serial Layer exiting')
	elif '/buffbot' in sys.argv[1]:
		main(rospy.get_param(sys.argv[1]))
	elif '.yaml' in sys.argv[1]:
		with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
			data = yaml.safe_load(f)
		main(data)


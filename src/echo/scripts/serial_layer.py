#! /usr/bin/env python3
import sys
import time
import rospy
import serial
import traceback as tb
from std_msgs.msg import Float64


class SerialLayer():
	def __init__(self, config_data):

		self.debug = False
		self.device = None
		self.baudrate = -1
		self.publishers = {}
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
			topics_names = config_data['TOPICS']
			all_topics = rospy.get_param('/buffbot/TOPICS')
			debug = rospy.get_param('/buffbot/DEBUG')
			for tn in topics_names:
				topic = all_topics[tn]
				self.publishers[topic] = rospy.Publisher(topic, Float64, queue_size=1)

		# for now require roscore to run (maybe later set up non-ros runtime)
		rospy.init_node('echo-serial', anonymous=True)

		self.tryConnect()


	def tryConnect(self):
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

		else:
			print('Device Connected: Reading...')


	def parsePacket(self):
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
				self.publishers[name] = rospy.Publisher(name, Float64, queue_size=1)

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
				self.tryConnect()

			elif self.device.in_waiting:
				self.parsePacket()

		except Exception as e:
			tb.print_exc()
			print(e)
			print('Possible I/O Disconnect: Reseting...')
			time.sleep(2)
			self.tryConnect()


def main(data):

	try:
		layer = SerialLayer(data) # tryConnect(port)

		while not rospy.is_shutdown():
			layer.spin()

	except KeyboardInterrupt:
		if layer.device:
			layer.device.close()

	except Exception as e:
		exc_type, exc_value, exc_traceback = sys.exc_info()
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


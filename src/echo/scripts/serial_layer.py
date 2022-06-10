#! /usr/bin/env python3
import sys
import time
import rospy
import serial
import numpy as np
import traceback as tb
from std_msgs.msg import String, Float64MultiArray


class SerialLayer():
	def __init__(self, name):

		self.name = name
		self.device = None
		self.publishers = {}
		self.connected = False

		self.debug = rospy.get_param('/buffbot/DEBUG')

		self.lives = int(rospy.get_param(f'{name}/LIVES'))
		self.serial_LUT = rospy.get_param(f'{name}/SERIAL_LUT')

		self.try_connect()

		topics = rospy.get_param('/buffbot/TOPICS')
		self.writer_sub = rospy.Subscriber(topics['SERIAL_OUT'], 
			String, self.writer_callback, queue_size=10)

		rospy.init_node('echo-serial', anonymous=True)

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
		byte_string = bytes(msg.data, 'utf-8')
		# print(byte_string)
		self.write_device(byte_string)

	def try_connect(self):
		"""
		  Try to connect with the device.
		"""
		try:
			if self.device:
				self.device.close()

			self.device = serial.Serial(rospy.get_param(f'{self.name}/PORT'), 
				rospy.get_param(f'{self.name}/BAUDRATE'), 
				timeout=rospy.get_param(f'{self.name}/TIMEOUT'))

			self.device.flush()

		except Exception as e:
			lives = int(rospy.get_param(f'{self.name}/LIVES'))
			rospy.logerr(e)
			rospy.logerr(f"Serial lives left {self.lives}")
			rospy.set_param(f'{self.name}/LIVES', lives-1)
			if lives < 2:
				return False

		else:
			rospy.loginfo('Device Connected: Reading...')

		return True

	def module_lookup(self, module, key):
		data = self.serial_LUT[module][key[0]]
		if 'TYPE' in data:
			return '/' + data['NAME'] + self.module_lookup(data['TYPE'], key[1:])

		else:
			return '/' + data
			
	def access_2_string(self, key):
		module = self.serial_LUT['SUBSYSTEM'][key[0]]['TYPE']
		s = '/' + self.serial_LUT['SUBSYSTEM'][key[0]]['NAME']
		return s + self.module_lookup(module, key[1:])

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
		packet = self.device.readline().decode().rstrip().split(':')

		if len(packet) == 2:
			name, val = packet
			if len(val.split('.')) - 1 > len(val.split(',')):
				return

		else:
			rospy.loginfo(packet[0])
			return

		if name[0] == '/':
			topic = self.access_2_string(name[1:])
			topic = '/' + '_'.join(topic.split('/'))[1:]
			if not name in self.publishers:
				topic 
				self.publishers[name] = rospy.Publisher(topic, Float64MultiArray, queue_size=10)

			msg = Float64MultiArray(data=np.array(val.split(','), dtype=np.float64))
			self.publishers[name].publish(msg)

		if name[0] == '@':
			topic = self.access_2_string(name[1:])
			rospy.set_param(topic, val.split(','))

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
					self.device = None
					print('Possible I/O Disconnect: Reseting...')
					time.sleep(2)


def main(data):

	layer = SerialLayer(data)
	layer.spin()

		
if __name__=='__main__':
	if len(sys.argv) < 2:
		print(f'No Data: Serial Layer exiting')
	elif '/buffbot' in sys.argv[1]:
		main(sys.argv[1])


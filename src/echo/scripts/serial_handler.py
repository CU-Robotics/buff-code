#! /usr/bin/env python3
import sys
import time
import serial
import traceback as tb
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

start = time.time()
t = []
error = []
target = []
measure = []
control = []

fig, ax = plt.subplots()

device = None
baudrate = 9600
connected = False
port = '/dev/tty.usbmodem113221301'

def tryConnect(port):
	"""
	  Try to connect with the device.
	Returns:
	  status: bool - True if successful
	"""
	try:

		device = serial.Serial(port, baudrate, timeout=5)
		device.flush()

	except Exception as e:
		print(f'Error Detecting Device at {port}')
		print(e)

	else:
		print('Device Connected: Reading...')

	return device

def parsePacket(packet):
	"""
	  Create a new msg to publish.
		Params:
			topic: string - destination for the publisher
			msgType: string - the datatype of the msg
			data: ??? - the data to fill msg
		Returns:
		  msg: ??? - the msg to send
	"""
	t.append(time.time() - start)
	for item in packet:
		name, val = item.split(':')
		if name == 'E':
			error.append(float(val))
		elif name == 'T':
			target.append(float(val))
		elif name == 'M':
			measure.append(float(val))
		elif name == 'U':
			control.append(float(val))

def spin(i):
	"""
	  Read a line from the serial port and publish it
	to a topic given the mode. The mode is the first 4 bytes
	of the msg.
	"""
	global device
	try:
		if device.in_waiting:
			packet = device.readline().decode().rstrip().split(',')
			parsePacket(packet)

		ax.plot(t, error)
		ax.plot(t, target)
		ax.plot(t, measure)
		ax.plot(t, control)

	except Exception as e:
		tb.print_exc()
		print(e)
		print('Possible I/O Error')
		time.sleep(2)
		tryConnect(device, port)


def main():
	global device

	try:
		device = tryConnect(port)
		#ani = FuncAnimation(fig, spin, frames=20, interval=100, repeat=True)

		plt.show()
		while True:
			cmdStr = input()
			device.write(cmdStr)

	except KeyboardInterrupt:
		if device:
			device.close()

	except Exception as e:
		exc_type, exc_value, exc_traceback = sys.exc_info()
		tb.print_exc()
		if device:
			device.close()


if __name__=='__main__':
	main()









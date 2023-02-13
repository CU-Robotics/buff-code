#!/usr/bin/python3
import os
import yaml
import time
import numpy as np
from scipy import linalg as la
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float64MultiArray

yaml_path = os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "data", "robots", os.getenv('ROBOT_NAME'), "nodes.yaml")

with open(yaml_path, 'r') as f:
	robot_data = yaml.safe_load(f)

n_motors = len(robot_data['motor_index'])
gm6020_voltage = 24
c620_max_current = 20

collect_samples = False
motor_positions = []
motor_speeds = []
motor_dampers = []

controller_outputs = []
controller_positions = []
controller_speeds = []


def motor_callback(msg):
	if collect_samples:
		motor_positions.append(msg.data[0]);
		motor_speeds.append(msg.data[1]);
		motor_dampers.append(msg.data[2]);

def controller_callback(msg):
	if collect_samples:
		controller_outputs.append(msg.data[0]);
		controller_positions.append(msg.data[1]);
		controller_speeds.append(msg.data[2]);

def discrete_impulse(magnitude=0.2, length=100, start=10, stop=60):
	control = []
	for i in range(length):
		if i >= start and (i < stop or 0 > start):
			control.append(magnitude)
		else:
			control.append(0)

	return control

def sinusiod(length=100, start=10, stop=60):
	control = []
	amplitudes = 0.2
	frequencies = 0.2

	for i in range(length):
		if i >= start and i < stop:
			control.append(np.sum(amplitudes * np.sin(frequencies * i)))
		else:
			control.append(0)

	return control

def freq_sweep_sinusiod(length=100, start=10, stop=60):
	control = []
	amplitudes = np.linspace(0, 10, 150) / 500
	frequencies = np.linspace(0, 100, 150)

	for i in range(length):
		if i >= start and i < stop:
			control.append(np.sum(amplitudes * np.sin(frequencies * i)))
		else:
			control.append(0)

	return control

def send_inputs(hz, control, target_motor):
	global collect_samples
	global motor_positions
	global motor_speeds
	global motor_dampers
	global controller_outputs
	global controller_positions
	global controller_speeds

	pub = rospy.Publisher('motor_can_output', Float64MultiArray, queue_size=10)
	rate = rospy.Rate(hz) # 10hz
	msg = Float64MultiArray()

	rospy.Subscriber(f"motor_{target_motor}_feedback", Float64MultiArray, motor_callback)
	rospy.Subscriber(f"controller_{target_motor}_report", Float64MultiArray, controller_callback)

	print("Test Start")
	for u in control:
		collect_samples = True

		msg.data = np.ones(n_motors)
		msg.data[target_motor] *= u
		pub.publish(msg)
		rate.sleep()

		if rospy.is_shutdown():
			exit(0)

	collect_samples = False
	print("Test Finished")


	n = len(motor_positions)
	m = len(controller_outputs)

	controller_outputs = np.array(controller_outputs)
	controller_positions = np.array(controller_positions)
	controller_speeds = np.array(controller_speeds)

	motor_positions = np.array(motor_positions)
	motor_speeds = np.array(motor_speeds)
	motor_dampers = np.array(motor_dampers)

	if n < m:
		controller_outputs = controller_outputs[(m - n):]
		controller_positions = controller_positions[(m - n):]
		controller_speeds = controller_speeds[(m - n):]

	if n > m:
		motor_positions = motor_positions[(n - m):]
		motor_speeds = motor_speeds[(n - m):]
		motor_dampers = motor_dampers[(n - m):]

	l = int(len(motor_positions) / len(control))

	control = np.array([[u] * l for u in control]).flatten()

	if l > n or l > m:
		controller_outputs = controller_outputs[(m - l):]
		controller_positions = controller_positions[(m - l):]
		controller_speeds = controller_speeds[(m - l):]
		motor_positions = motor_positions[(n - l):]
		motor_speeds = motor_speeds[(n - l):]
		motor_dampers = motor_dampers[(n - l):]


	if len(motor_positions) == 0:
		print("No samples exiting")
		exit(0)

	print(f"Elapsed time: {len(control) / hz} secs")
	print(f"Feedback packets: {motor_positions.shape} {motor_speeds.shape} {motor_dampers.shape}")
	print(f"Control packets: {controller_outputs.shape} {controller_positions.shape} {controller_speeds.shape}")
	return control

def least_squares_solver(motor, control):
	samples = len(motor_positions)
	if samples == 0:
		print("No samples exiting")
		exit(0)

	A = np.vstack([motor_positions[:-1], motor_speeds[:-1], motor_dampers[:-1], controller_outputs[:-1]]).T
	B = np.vstack([motor_positions[1:], motor_speeds[1:], motor_dampers[1:]]).T

	print(f"Solving {A.shape} @ {(A.shape[1], B.shape[1])} = {B.shape}")
	X = la.inv(A.T @ A) @ A.T @ B

	print(f"Found {X.shape}")

	sys_A = X.T[:,:-1].T
	sys_B = np.array([X.T[:,-1]]).T

	print(f"Modeled system with delay:\n{sys_A}\n{sys_B}")
	return sys_A, sys_B


def validate_system(A, B, control, hz):
	error = 0
	p = motor_positions[0]
	v = motor_speeds[0]
	d = motor_dampers[0]

	sim_pos = [p]
	sim_speed = [v]
	sim_damper = [d]

	print(f"Simulating {A.shape} @ {p} + {B.shape} * {control[0]}")
	for i, u in enumerate(control[:-1]):
		x_new = (A @ np.array([[p],[v],[d]])) + (B * u)
		p = x_new[0,0]
		v = x_new[1,0]
		d = x_new[2,0]
		sim_pos.append(p)
		sim_speed.append(v)
		sim_damper.append(d)
		error += np.linalg.norm([motor_positions[i+1] - p, motor_speeds[i+1] - v, motor_dampers[i+1] - d])

	# plt.plot(t, sim_pos, label='simulated')
	# plt.plot(t, motor_positions, label='hardware')
	# plt.legend()
	# plt.show()
	return sim_pos, sim_speed


def lqr(Q, R, A, B):
	return -np.linalg.pinv(R + B.T @ Q @ B) @ B.T @ Q @ A

def find_edge(arr, threashold):
	prev = arr[0]

	for (i, x) in enumerate(arr[1:]):
		if abs(x - prev) > threashold:
			return i

	return -1

def find_control_delay(duration, control):
	ctrl_edge = find_edge(control, 0.001)
	pos_edge = find_edge(motor_positions, 0.001)
	rpm_edge = find_edge(motor_speeds, 0.1)

	if ctrl_edge >= 0 and pos_edge >= 0 and rpm_edge >= 0:
		"""
			(ctrl_edge / ctrl_len) + delay = (pos_edge / pos_len)
			delay = (pos_edge / pos_len) - (ctrl_edge / ctrl_len)
		"""
		p_control_shift = (pos_edge / len(motor_positions)) - (ctrl_edge / len(control))
		v_control_shift = (rpm_edge / len(motor_speeds)) - (ctrl_edge / len(control))

		print(f"Position estimated latency: {p_control_shift * duration}")
		print(f"RPM estimated latency: {v_control_shift * duration}")
		print(f"Fused estimated latency: {(p_control_shift + v_control_shift) * duration / 2}")
		
		return (p_control_shift + v_control_shift) / (2 * duration)

	else:
		print("Couldn't find edge")
		return 0


def display_data(motor, duration, control, sim_positions, sim_speeds):

	control_scaled = control

	control_shift = int(find_control_delay(duration, control) * duration)

	ctrl_steps = range(control_shift, len(control) + control_shift)
	fb_steps = np.linspace(0, len(control), num=len(motor_positions))
	sim_steps = np.linspace(0, len(control), num=len(sim_positions))

	fig, axes = plt.subplots(2, 2, figsize=(8,15))
	axes[0][0].set_title("motor position")
	axes[0][0].plot(fb_steps, motor_positions, label="hardware feeback")
	axes[0][0].plot(fb_steps, controller_positions, label="controller reference")
	axes[0][0].plot(sim_steps, sim_positions, label="sim feedback")

	axes[0][1].set_title("motor speed")
	axes[0][1].plot(fb_steps, motor_speeds, label="hardware feeback")
	axes[0][1].plot(fb_steps, controller_speeds, label="controller reference")
	axes[0][1].plot(sim_steps, sim_speeds, label="sim feedback")


	axes[1][0].set_title("motor damper")
	axes[1][0].plot(fb_steps, motor_dampers, label="hardware")

	axes[1][1].set_title("controller output")
	axes[1][1].plot(ctrl_steps, control, label="output")

	axes[0][0].legend()
	plt.show()


if __name__ == '__main__':
	try:
		rospy.init_node('motor_identifier', anonymous=True)

		target_motor = 4
		# control = freq_sweep_sinusiod()
		control = discrete_impulse()

		time.sleep(2)
		print(f"Starting system identification for motor {target_motor}")
		hz = 20
		duration = len(control) / hz
		control = send_inputs(hz, control, target_motor)

		sys_A, sys_B = least_squares_solver(target_motor, control)

		p, v = validate_system(sys_A, sys_B, control, hz)
		# Q = np.array([[1.0,  0.0,  0.0], 
		# 			 [0.0, 0.001,  0.0],
		# 			 [0.0,  0.0, 0.001]])
		# R = np.array([0.005])

		# k = lqr(Q, R, sys_A, sys_B)

		# print(f"LQR solver feedback gains: {k}")

		display_data(target_motor, duration, control, p, v)

	except rospy.ROSInterruptException:
		pass
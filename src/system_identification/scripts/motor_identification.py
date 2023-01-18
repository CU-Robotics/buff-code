#!/usr/bin/python3
import time
import numpy as np
from scipy import linalg as la
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float64MultiArray


n_motors = 7
motor_voltage = 24

collect_samples = False
motor_positions = [[]] * n_motors
motor_speeds = [[]] * n_motors
motor_torques = [[]] * n_motors


def callback(msg, motor_id):
	if collect_samples:
		motor_positions[motor_id].append(msg.data[0]);
		motor_speeds[motor_id].append(msg.data[1]);
		motor_torques[motor_id].append(msg.data[2]);

def discrete_impulse(magnitude=0.2, length=100, start=10, stop=60):
	control = []
	for i in range(length):
		if i >= start and i < stop:
			control.append(magnitude)
		else:
			control.append(0)

	return control

def send_inputs(hz, control):
	global collect_samples

	pub = rospy.Publisher('motor_can_output', Float64MultiArray, queue_size=10)
	rate = rospy.Rate(hz) # 10hz
	msg = Float64MultiArray()

	for i in range(n_motors):
		rospy.Subscriber(f"motor_{i}_feedback", Float64MultiArray, callback, i)

	for u in control:
		collect_samples = True

		msg.data = np.ones(n_motors) * u
		pub.publish(msg)
		rate.sleep()

		if rospy.is_shutdown():
			exit(0)

	rate.sleep()
	collect_samples = False

	if np.sum([len(x) for x in motor_positions]) == 0:
		print("No samples exiting")
		exit(0)

	print(f"Feedback packets received: {[len(x) for x in motor_positions]}")

def least_squares_solver(motor, control_fn):
	samples = len(motor_positions[motor][:-1])
	if samples == 0:
		print("No samples exiting")
		exit(0)

	control = control_fn(length=samples)
	A = np.vstack([motor_positions[motor][:-1], motor_speeds[motor][:-1], motor_torques[motor][:-1], control]).T
	B = np.vstack([motor_positions[motor][1:], motor_speeds[motor][1:], motor_torques[motor][1:]]).T

	print(f"Solving {A.shape} @ {(A.shape[1], B.shape[0])} = {B.shape}")
	X = la.inv(A.T @ A) @ A.T @ B

	sys_A = X.T[:,:-1].T
	sys_B = np.array([X.T[:,-1]]).T

	print(f"Modeled system with delay:\n{sys_A}\n{sys_B}")
	return sys_A, sys_B


def lqr(Q, R, A, B):
	return -np.linalg.pinv(R + B.T @ Q @ B) @ B.T @ Q @ A

def find_edge(arr, threashold):
	prev = arr[0]

	for (i, x) in enumerate(arr[1:]):
		if abs(x - prev) > threashold:
			return i

	return -1

def find_control_delay(motor, hz, control):
	ctrl_edge = find_edge(control, 0.05)
	pos_edge = find_edge(motor_positions[motor], 0.1)
	rpm_edge = find_edge(motor_speeds[motor], 10)

	if ctrl_edge >= 0 and pos_edge >= 0 and rpm_edge >= 0:
		p_control_shift = int(95 * pos_edge / len(motor_positions[motor])) - ctrl_edge
		v_control_shift = int(95 * rpm_edge / len(motor_speeds[motor])) - ctrl_edge

		print(f"Position estimated latency: {p_control_shift / hz}")
		print(f"RPM estimated latency: {v_control_shift / hz}")
		print(f"Fused estimated latency: {(p_control_shift + v_control_shift) / (2 * hz)}")

	else:
		print("Couldn't find edge")
		return 0

	return (p_control_shift + v_control_shift) / (2 * hz)


def display_data(motor, hz, control):
	control_scaled = [24 * u for u in control]

	control_shift = int(find_control_delay(motor, hz, control) * hz)

	ctrl_steps = range(control_shift, len(control) + control_shift)
	fb_steps = np.linspace(0, 101, num=len(motor_positions[motor]))

	fig, axes = plt.subplots(2, 2, figsize=(8,15))
	axes[0][0].set_title("motor_position")
	axes[0][0].plot(fb_steps, motor_positions[motor], label="angle")
	axes[0][0].plot(ctrl_steps, control_scaled, label="Voltage")

	axes[0][1].set_title("motor_speed")
	axes[0][1].plot(fb_steps, motor_speeds[motor], label="rpm")
	axes[0][1].plot(ctrl_steps, control_scaled, label="Voltage")


	axes[1][0].set_title("motor_torque")
	axes[1][0].plot(fb_steps, motor_torques[motor], label="torque")
	axes[1][0].plot(ctrl_steps, control_scaled, label="Voltage")

	axes[0][0].legend()
	plt.show()


if __name__ == '__main__':
	try:
		rospy.init_node('motor_identifier', anonymous=True)

		target_motor = 4
		control = discrete_impulse()

		time.sleep(2)
		print(f"Starting system identification for motor {target_motor}")
		hz = 10
		send_inputs(hz, control)

		sys_A, sys_B = least_squares_solver(target_motor, discrete_impulse)

		Q = np.array([[1.0,  0.0,  0.0], 
					 [0.0, 0.001,  0.0],
					 [0.0,  0.0, 0.001]])
		R = np.array([0.005])

		k = lqr(Q, R, sys_A, sys_B)

		print(f"LQR solver feedback gains: {k}")

		display_data(target_motor, hz, control)

	except rospy.ROSInterruptException:
		pass
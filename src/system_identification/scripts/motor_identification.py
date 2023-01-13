#!/usr/bin/python3

import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt

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

def send_inputs(hz, control):
	global collect_samples

	pub = rospy.Publisher('motor_can_output', Float64MultiArray, queue_size=10)
	rospy.init_node('stimulator', anonymous=True)
	rate = rospy.Rate(hz) # 10hz
	msg = Float64MultiArray()

	for i in range(n_motors):
		rospy.Subscriber(f"motor_{i}_feedback", Float64MultiArray, callback, i)

	for u in control:
		collect_samples = True

		msg.data = np.ones(n_motors) * u
		pub.publish(msg)
		rate.sleep()

	rate.sleep()
	collect_samples = False

	print(f"Feedback packets received: {[len(x) for x in motor_positions]}")

def find_edge(arr, threashold):
	prev = arr[0]

	for (i, x) in enumerate(arr[1:]):
		if abs(x - prev) > threashold:
			return i

	return -1

def display_data(motor, hz, control):
	control_scaled = [24 * u for u in control]

	ctrl_edge = find_edge(control_scaled, 0.5)
	pos_edge = find_edge(motor_positions[motor], 0.1)
	rpm_edge = find_edge(motor_speeds[motor], 10)

	p_control_shift = 0
	if ctrl_edge >= 0 and pos_edge >= 0 and rpm_edge >= 0:
		p_control_shift = int(95 * pos_edge / len(motor_positions[motor])) - ctrl_edge
		v_control_shift = int(95 * rpm_edge / len(motor_speeds[motor])) - ctrl_edge

		print(f"Position estimated latency: {p_control_shift / hz}")
		print(f"RPM estimated latency: {v_control_shift / hz}")

	ctrl_steps = range(p_control_shift, 100 + p_control_shift)
	fb_steps = np.linspace(0, 101, num=len(motor_positions[motor]))

	fig, axes = plt.subplots(2, 2, figsize=(15,8))
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

# def least_squares_solver(input, feedback):

if __name__ == '__main__':
	try:

		control = []
		for i in range(100):
			if i >= 10 and i < 60:
				control.append(0.2)
			else:
				control.append(0)

		hz = 10
		send_inputs(hz, control)
		display_data(4, hz, control)

	except rospy.ROSInterruptException:
		pass
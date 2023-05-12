#!/usr/bin/env python3

import os
import sys
import yaml
import glob
import rospy
import shutil
import roslaunch
import subprocess as sb

from tools import *
from robot_description import Robot_Description

Buffpy_Lib = os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'lib')
Buffpy_Robots = os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'data', 'robots')

class Robot_Spawner:
	def __init__(self):
		self.commands = None
		self.respawn = None

		self.pool = None
		self.launch = None
		self.roscore = None

	def launch_ros_core(self):
		# launch core
		self.roscore = sb.Popen('roscore', stdout=sb.PIPE, stderr=sb.PIPE)
		self.launch = roslaunch.scriptapi.ROSLaunch()
		self.launch.start()

	def set_ros_params(self, name, with_xacro):
		namespace = '/buffbot'
		params = {'robot-name': name}
		if (with_xacro):
			command_string = f"rosrun xacro xacro {os.path.join(Buffpy_Robots, name, 'buffbot.xacro')}"
			robot_description = sb.check_output(command_string, shell=True, stderr=sb.STDOUT)
			params['description'] = robot_description.decode()

		rospy.set_param(f'{namespace}', params)

	def spawn_nodes(self):
		print(f"Spawning {len(self.commands)} nodes")
		for (i,cmd) in enumerate(self.commands):
			print(f"Starting {cmd}")
			if self.pool is None:
				self.pool = {}

			process = sb.Popen(cmd)
			self.pool[''.join(cmd)] = process

	def launch_system(self, name, with_xacro):
		"""
			This wil launch all nodes and roscore. 
			launches core process with subprocess.Popen 
			systems are launched with the usage below
			rosrun package program args=[debug, config, topics...]
			PARAMS:
				target_path: filepath, to yaml
				config: load configuration (dict)
			RETURNS:
				core_process: subprocess obj, the core process
				pool: list of roslaunch processes
		"""
		project_root = os.getenv('PROJECT_ROOT')

		self.launch_ros_core()
		self.set_ros_params(name, with_xacro)
		self.spawn_nodes()

	def spin(self, robot):
		"""
			initialize the run and launch
			loop and wait for user kill or core to finish
			PARAMS:
				config: a python script or system config file (yaml)
				target:
		"""

		# load the description
		rd = Robot_Description()
		rd.load_description(robot)
		self.respawn, self.commands = rd.get_commands()

		# Try to catch user kill and other errors
		try: 
			self.launch_system(rd.name, rd.with_xacro)
			# core_process is the core process we are trying to spawn (python script or roscore)
			# if it fails there is no point in spinning
			if self.roscore is None: 
				print('Failed to launch core :(')
				exit(0)

			# this hanldes user kill (available after spawn & in the run space below)
			# this will loop until an interrupt or the core dies.
			while self.roscore.poll() is None:
				# restart processes with the respawn flag
				if not self.pool is None:
					for (i, cmd) in enumerate(self.commands):
						proc = self.pool[''.join(cmd)]
						if not proc is None and not proc.poll() is None:
							if proc.returncode != 0 and self.respawn[i]:
								print(f'{self.commands[i]} died: {proc.returncode}')
								spawn_nodes(self.commands[i])

		except KeyboardInterrupt as e:
			print(e)
			print('Terminate Recieved')

		except Exception as e:
			print(e)
			print('Killed due to error')

		if not self.pool is None:
			for (i, cmd) in enumerate(self.commands):
				proc = self.pool[''.join(cmd)]
				if not proc is None:
					proc.terminate()

		if not self.roscore is None:
			self.roscore.terminate()


def main():
	with open(os.path.join(Buffpy_Robots, 'self.txt'), 'r') as f:
		robot = f.read()

	rs = Robot_Spawner()
	rs.spin(robot)

if __name__ == '__main__':
	main()
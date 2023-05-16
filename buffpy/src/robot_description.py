#!/usr/bin/env python3

import os
import sys
import yaml
import glob
import shutil
import subprocess

from buffpy_tools import *

class Robot_Description:
	"""
		Build profiles define how to generate nodes and data
		Robot Descriptions describe what nodes and data a robot
		  needs to use during runtime
	"""
	def __init__(self):
		self.type = None
		self.name = None
		self.path = None
		self.with_xacro = False
		self.buff_nodes = None
		self.buff_respawn = None
		self.ros_nodes = None
		self.ros_respawn = None
		self.data = None

	def ros_node_commands(self, nodes):
		"""
			Parse the ros_nodes section of a nodes.yaml
			Construct a rosrun command for each node.

			@params:
				nodes: dictionary of nodes to spawn
		"""
		self.ros_nodes = []
		self.ros_respawn = []

		# load and launch each node in the config
		for name in nodes:
			args = []
			required = False
			node = nodes[name]

			if 'package' in node:
				package = node['package']
			else:
				print(f"{name} does not specify a package")
				continue

			if 'args' in node:
				args = parse_args(node['args'])

			if 'required' in node:
				required = node['required']

			# launch the process and add it to the pool
			if 'files' in node:
				for program in node['files']:
					cmd = ['rosrun', package, program] + args
					self.ros_respawn.append(required)
					self.ros_nodes.append(cmd)

	def buff_node_commands(self, nodes):
		"""
			parse the buff_nodes section of a nodes.yaml
			Construct commands to run each node, does not
			use rosrun

			@params:
				nodes: dictionary of nodes to spawn
		"""
		self.buff_nodes = []
		self.buff_respawn = []

		# load and launch each node in the config
		for name in nodes:
			args = []
			node = nodes[name]

			if 'args' in node:
				args = parse_args(node['args'])

			if 'required' in node:
				required = node['required']
			else:
				required = False

			# launch the process and add it to the pool
			if 'files' in node:
				for program in node['files']:
					file_path = os.path.join(BuffPy_LOC_LUT['lib'], program)
					if '.py' == program[-3:]:
						cmd = ['python3', file_path]
					else:
						cmd = [file_path]
		
					self.buff_respawn.append(required)
					self.buff_nodes.append(cmd)

	def load_description(self, name):
		"""
			Parse a yaml file into a build profile
			requires a valid yaml file and project
			setup. Both paths must be absolute and 
			valid
		"""
		self.data = {}
		self.name = name
		self.path = os.path.join(BuffPy_LOC_LUT['robots'], self.name, 'nodes.yaml')

		if not os.path.exists(self.path):
			print(f"Can't find description: {self.path}")
			return

		with open(self.path, 'r') as description:
			contents = yaml.safe_load(description)


		if 'robot_type' in contents:
			self.type = contents['robot_type']
			self.data['robot_type'] = contents['robot_type']

		else:
			print("No robot-type identifier: add a robot_type to the description and try again")
			self.name = None
			return

		if contents is None:
			print(f'Missing NODES in config file {target_path}')
			return

		else:
			self.with_xacro = 'with_xacro' in contents

			if 'ros_nodes' in contents:
				self.ros_node_commands(contents['ros_nodes'])
				self.data['ros_nodes'] = contents['ros_nodes']

			if 'buff_nodes' in contents:	
				self.buff_node_commands(contents['buff_nodes'])
				self.data['buff_nodes'] = contents['buff_nodes']

		self.data['robot_name'] = name
		os.environ['ROBOT_NAME'] = name					# for BYU
		self.data['sensor_index'] = contents['sensor_index']
		self.data['motor_index'] = contents['motor_index']

		print(f'System Description:\n{yaml.dump(self.data, allow_unicode=True)}\n')

	def get_commands(self):
		"""
			Combine all commands and respawn modes to
			two lists. (convenience for the launcher)
		"""
		respawn = []
		commands = []

		if not self.buff_respawn is None:
			respawn = self.buff_respawn
			commands = self.buff_nodes

		if not self.ros_respawn is None:
			respawn += self.ros_respawn
			commands += self.ros_nodes

		return respawn, commands



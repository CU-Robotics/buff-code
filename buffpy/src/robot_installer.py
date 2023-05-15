#!/usr/bin/env python3

import subprocess as sb
from buffpy_tools import *


def deploy_directory(robot_address, source, target):
	""" 
		Send buffpy directory to a robot with scp.
		PARAMS:
			robot_address: string, USERNAME@<IP/HOSTNAME>
			source: string or path like object, the target directory to copy
			target: string or path like object, the target directory to copy into
		RETURNS:
			None
	"""
	clean_dir_result = sb.run(['ssh', robot_address, f'if [[ ! -d {target} ]]; then mkdir -p {target}; fi;'], timeout=60)
	if clean_dir_result.returncode != 0:
		return clean_dir_result.returncode

	try:	
		install_result = sb.run(['scp', '-r', source, f'{robot_address}:{target}'])
		print(install_result)
		return install_result.returncode
				
	except sb.TimeoutExpired:
		print(f'{robot_address} Timed out')

	return 13

def deploy_all_devices():
	"""
		Install built code to all robots
		regestered in robots.yaml
		requires password
			1 - without installed keys
			0 - with installed keys
	"""

	# define the installation
	source, target, ID, devices = load_install_params()

	print(f'Deploying with\n\tSource: {source}\n\tTarget: {target}\n\tID: {ID}\n\tDevices: {devices}')

	for robot in devices:

		# get robot type
		robot_type = devices[robot]
		# set target destination
		robot_address = f'cu-robotics@{robot}'
		# set default robot
		with open(ID, 'w+') as f:
			f.write(f"{robot_type}")
	
		# if root does exist remove it
		try:	
			clean_dir_result = sb.run(['ssh', robot_address, f'if [[ -d {target}/buffpy ]]; then rm -rf {target}/buffpy; fi; mkdir -p {target}'], timeout=30)
			if clean_dir_result.returncode == 0:
				deploy_directory(robot_address, source, target)

		except sb.TimeoutExpired:
			print(f'{robot_address} Timed out')
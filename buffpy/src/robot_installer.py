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

def install_ssh_keys():
	"""
		Generate and push ed25519 keys to
		registered devices. This may be a huge 
		security risk. But it makes installing
		and sshing really nice.
		requires password (per device)
			2 - without installed keys
			0 - with installed keys
	"""

	devices = get_devices()
	home = os.getenv('HOME')

	# only generate keys if they do not exist already
	if not os.path.exists(f'{home}/.ssh/id_ed25519.pub'):
		result = subprocess.run(['ssh-keygen', '-t', 'ed25519'])

	for robot in devices:
		robot_address = f'cu-robotics@{robot}'
		try:
			result = subprocess.run(['ssh', robot_address, f'if [[ ! -f /home/cu-robotics/.ssh/id_ed25519.pub ]]; then ssh-keygen -t ed25519; fi'], timeout=30)
			if result.returncode == 0:
				result = subprocess.run(['ssh-copy-id', '-i', f'{home}/.ssh/id_ed25519.pub', robot_address])
				if result.returncode == 0:
					print(f'{robot} Success')

		except subprocess.TimeoutExpired:
			print(f'{robot} Timed out')

def initialize_devices():
	"""
		Deploy buffpy and run installation on
		each registered device.
	"""

	# define the installation
	source, target, ID, devices = load_install_params()

	data_target = os.path.join(target, 'buffpy', 'data')
	src_target = os.path.join(target, 'buffpy')
	data_source = os.path.join(source, 'data', 'install')
	src_source = os.path.join(source, 'scripts')

	print(f'Running install as\n\tSource: {source}\n\tTarget: {target}\n\tID: {ID}\n\tDevices: {devices}')

	for robot in devices:
		# set target destination
		robot_address = f'cu-robotics@{robot}'

		with open(ID, 'w+') as f:
			f.write(f"{robot}")

		# deploy buffpy and source the install scripts
		clean_dir_result = subprocess.run(['ssh', robot_address, f'if [[ -d {target} ]]; then rm -rf {target}/; fi; mkdir -p {target}'], timeout=30)

		if clean_dir_result.returncode != 0:
			print(f'Failed to clean {robot}')
			continue

		if deploy_directory(robot_address, source, target) != 0:
			print(f'Failed to deploy to {robot}')
			continue

		try:
			setup_result = subprocess.run(['ssh', 
					'-t',
					robot_address, 
					f'cd /home/cu-robotics/buff-code && source buffpy/scripts/install.bash'])

		except subprocess.TimeoutExpired:
			print(f'{robot_address} Timed out')
#!/usr/bin/env python3

import os
import yaml
import shutil

# logger statuses (please add more, or make the logger a whole project)
LOGGER_MESSAGE_TYPES = {0: "INFO", 1: "WARNING", 2: "ERROR"}

# A look up table that connects shorthand terms to absolute paths in buff-code
# this file is on the python path and can be used from any other python script
# TODO: find a way to share this with rust (maybe it needs to be saved as a file)
BuffPy_Project_Root = os.getenv('PROJECT_ROOT')
Buffpy_Profile_Path = os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "data", "build")

BuffPy_LOC_LUT = {  'root': BuffPy_Project_Root,
					'src': os.path.join(BuffPy_Project_Root, 'src'),
					'data': os.path.join(BuffPy_Project_Root, 'data'),
					'buffpy': os.path.join(BuffPy_Project_Root, 'buffpy'),
					'lib': os.path.join(BuffPy_Project_Root, 'buffpy', 'lib'), 
					'bin': os.path.join(BuffPy_Project_Root, 'buffpy', 'bin'),
					'docker': os.path.join(BuffPy_Project_Root, 'containers'),
					'docs': os.path.join(BuffPy_Project_Root, 'documentation'),
					'models': os.path.join(BuffPy_Project_Root, 'buffpy', 'data', 'models'),
					'robots': os.path.join(BuffPy_Project_Root, 'buffpy', 'data', 'robots'),
					'profiles': os.path.join(BuffPy_Project_Root, 'buffpy', 'data', 'build'),
					'self': os.path.join(BuffPy_Project_Root, 'buffpy', 'data', 'robots', 'self.txt')}

def buff_log(msg, status):
	"""
		Fancy print function with the buffpy signiture
		and a status, see above for status information
	"""
	if status is None:
		status = 2

	print(f"[Buffpy] {LOGGER_MESSAGE_TYPES[status]}: {msg}")

def reset_directory(directory):
	"""
		Another file system utility
		deletes a directory and recreates it

		Use this to remove the contents of a directory,
		Not to remove the directory
	"""
	if os.path.exists(directory):
		shutil.rmtree(directory)

	os.mkdir(directory)

def assert_directory(directory):
	"""
		Another file system utility,
		checks that a directory exists
		if it doesn't create the directory
	"""
	if not os.path.exists(directory):
		buff_log(f"{directory} does not exist, creating", 1)
		os.mkdir(directory)

def copy_file_from_profile(src, dst):
	"""
		Helper for copy packages
		@params
			src: path to the file (includes filename)
			target_path: path to install (includes filename)

		1) copy the file
	"""
	shutil.copyfile(src, dst)

def copy_dir_from_profile(src, dst):
	"""
		Helper for copy packages
		@params
			src: path to the file (includes filename)
			target_path: path to install (includes filename)

		1) reset dst
		2) copy the directory to dst
	"""
	reset_directory(dst)
	shutil.copytree(src, dst)
	

def copy_packages(src_path, item_name, dst_path):
	"""
		Copy a file or directory from src_path/target_src to dst_path
		The copied package will have the same name (target_src) and will
		exist under dst_path
		Uses buff_log as a debug tool, status will reflect if the
		file exists before copying and is in the right location after

		Heavy logging for now, can turn it down when we feel stable.

		Will create install locations if they don't exist

		@params
			src_path: project directory
			item_name: list of files or directories in project (item_name can be a path leading out of the project)
			dst_path: list of directories to install packages to

	"""
	os.chdir(src_path)
	for (item, loc) in zip(item_name, dst_path):
		assert_directory(loc)
		src = os.path.join(src_path, item)
		dst = os.path.join(loc, item.split('/')[-1])
		buff_log(f"Installing {src} -> {dst}", not os.path.exists(src))

		if os.path.isdir(item):
			copy_dir_from_profile(src, dst)

		elif os.path.isfile(item):
			copy_file_from_profile(src, dst)

		buff_log(f"Installed {src} -> {dst}", not os.path.exists(dst))

def parse_args(args):
	"""
		Used to replace env variables in a string of arguments.
		This allows env variables in the args section of a node 
		 > see nodes.yaml:buff-nodes or nodes.yaml:ros-nodes

		@params
			args: list of strings

		@returns
			list of strings

		ros-nodes:
		  rqt_plot:
		  files: [rqt_plot]
		  package: rqt_plot
		  args: ['-c', '${PROJECT_ROOT}/buffpy/data/robots/penguin/default.perspective']
	"""
	clean_args = []
	for arg in args:

		# find any env vars
		split1 = arg.split('{')

		n = len(split1) // 2

		if n == 0:
			clean_args.append(arg)
			continue

		split = ''

		for i in range(n):
			split2 = split1[(2 * i) + 1].split('}')
			split += split1[2 * i][:-1] + os.getenv(split2[0]) + split2[1]

		clean_args.append(split)

	return clean_args

def get_devices():
	"""
		Load all registered devices from buffpy/data/robots/robots.yaml
	"""
	robots = os.path.join(BuffPy_LOC_LUT['robots'], 'robots.yaml')
	if os.path.exists(robots):
		with open(robots, 'r') as f:
			return yaml.safe_load(f)

def load_install_params():
	"""
		Get installation definition
		RETURNS:
			source: string or path like object, the target directory to copy
			target: string or path like object, the destination directory to copy into
			ID: string or path like object, the identify file of the install (self.yaml)
	"""

	# Setup Parameters for install
	source = BuffPy_LOC_LUT['buffpy']
	target = os.path.join('/home', 'cu-robotics', 'buff-code')
	ID = BuffPy_LOC_LUT['self']

	if len(os.listdir(os.path.join(source, 'lib'))) == 0:
		print("Workspace not built, run:\n\t \'buffpy --build <profile>\'")

	# Don't install from edge devices (robot)
	if 'edge' in os.getenv('HOSTNAME'):
		print(f'Can\'t install from device: {os.getenv("HOSTNAME")}')
		return source, target, ID, []

	devices = get_devices()

	return source, target, ID, devices
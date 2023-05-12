#!/usr/bin/env python3

import os
import shutil

MESSAGE_TYPES = {0: "INFO", 1: "WARNING", 2: "ERROR"}

def buff_log(msg, status):
	if status is None:
		status = 2

	print(f"[Buffpy] {MESSAGE_TYPES[status]}: {msg}")

def reset_directory(directory):
	if os.path.exists(directory):
		shutil.rmtree(directory)

	os.mkdir(directory)

def assert_directory(directory):
	if not os.path.exists(directory):
		os.mkdir(directory)

def copy_file_from_profile(src, target_path):
	target_file = src.split('/')[-1]
	shutil.copyfile(src, os.path.join(target_path, target_file))

def copy_dir_from_profile(src, target_path):
	target_file = src.split('/')[-1]
	target = os.path.join(target_path, target_file)
	reset_directory(target)
	shutil.copytree(src, target)
	

def copy_packages(src_path, target_src, dst_path):
	os.chdir(src_path)
	for (item, dst) in zip(target_src, dst_path):
		src = os.path.join(src_path, item)
		buff_log(f"Installing {src} -> {dst}", not os.path.exists(src))
		if os.path.isdir(item):
			copy_dir_from_profile(src, dst)
		elif os.path.isfile(item):
			copy_file_from_profile(src, dst)

def parse_args(args):
	# can use args tag to send args to nodes
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
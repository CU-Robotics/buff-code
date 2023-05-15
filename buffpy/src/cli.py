#!/usr/bin/env python3

import os
import sys
import yaml
import glob
import shutil
import argparse
import subprocess
from buffpy_tools import *
from robot_installer import *
from build_profile import Build_Profile


# This is BuffPy

def clean_profile(profile):
	bp = Build_Profile();
	bp.load_profile(profile)
	bp.run_clean()

def build_profile(profile):
	bp = Build_Profile();
	bp.load_profile(profile)
	bp.run_build()

def main():
	parser = argparse.ArgumentParser(prog=sys.argv[0], description='CURO CLI Toolset')
	parser.add_argument('-b', '--build', 
		nargs='?',
		metavar='PROFILE',
		const='',
		help='Builds the workspace locally')
	parser.add_argument('--train', 
		metavar='MODEL',
		default='',
		help='Trains a Yolov5 model locally')
	parser.add_argument('-d', '--deploy', 
		action='store_true',
		help='Deploys build to the registered robots')
	parser.add_argument('-i', '--initialize', 
		action='store_true',
		help='Initializes registered devices')
	parser.add_argument('-c', '--clean',
		nargs='?',
		metavar='PROFILE',
		const='',
		help='Clean the entire workspace or a project. use profile=(lib,data) to clean workspace')

	ap = parser.parse_args(sys.argv[1:])

	if ap.clean:
		if ap.clean in BuffPy_LOC_LUT: # clean the workspace
			reset_directory(BuffPy_LOC_LUT[ap.clean])
			buff_log(f"Reset directory {BuffPy_LOC_LUT[ap.clean]}", 0)
		else:	# clean a profile
			clean_profile(ap.clean)

	if ap.build:
		build_profile(ap.build)

	if ap.deploy:
		deploy_all_devices()


if __name__ == '__main__':
	main()
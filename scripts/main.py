#!/usr/bin/env python3
####################
#		BuffVision 
#  	this is the main script for all CV 
#	operations. This is an outer control
#	loop that will enable all vision processing.
#	Any classes and methods are defined in 
#	buffpy and imported below.
###################

######## Imports ########
import os 
import sys
import subprocess as sb

######## Main ########

def main():
	file = sys.argv[0]
	path = os.path.dirname(file)
	
	print('Launching BuffVision standby...')
	sb.run([f'{path}/../buffpy/lib/buffvision.py'], shell=True)


if __name__ == '__main__':
	main()
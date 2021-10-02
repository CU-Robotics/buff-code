# BuffBotics
CU Robotics' development repo

This project is desinged to run in an Ubuntu20 environment or on a Jetson device. This repo provides a python3 package buffpy that is meant to aid in the development of robot software. The package contains a command line tool 'buff'. This tool helps streamline managing data, building locally or to devices, flashing firmware and hopefully soon running regressions on the developed subsystems.

First always load the environment variables, if you're not sure if you did just do it again. Also this script loads the path variables relative to where it was run. Make sure to run it from the root of this repo.

	source buff.bash
	
The majority of functionality is based in the buffpy package. This package is setup as a command line tool in buff.bash. All you need to utilize this package is call 'buff' from the command line.

	usage: buff [-h] [--sshBot] [--setBot ROBOT_IP] [--GDrive ACTION FOLDER_ID] [--installKeys] [--launch LOCATION] [--botPull] [--build LOCATION] [--clean] [--flash FQBN FW]

	CU-Robotics Digital House-Keeper

	optional arguments:
	  -h, --help            show this help message and exit
	  --sshBot              SSH into the bot at the ROBOT_IP env variable
	  --setBot ROBOT_IP     Set the IP of the robot in development
	  --GDrive ACTION FOLDER_ID
				Pull or push a batch from google drive, requires batch folder id
	  --installKeys         Push local sshkeys to the robot at ROBOT_IP
	  --launch LOCATION     Launch the robots software locally (True, 1) or on the bot (bot)
	  --botPull             Pull data from the robot at ROBOT_IP
	  --build LOCATION      Builds the workspace locally (True, 1) or to the robot_ip (install)
	  --clean               Clean the current bin and data, NOT recoverable; only run this if you are sure you want to
	  --flash FQBN FW       Flashes the given board with the target FW (expected port is ACM0)

## Install

Clone the repo to machine

	git clone git@github.com:/CU-Robotics/buff-code.git

or if you don't have ssh keys setup

	git clone https://github.com/CU-Robotics/buff-code.git

Now download the arduino_cli tar and extract it to buffpy. Make sure to rename the file arduino-cli_0.19.2 (this is the version we will use). Your directory should look like (excluding other files to condense the manual)

	buff-code
	 - buffpy
	 	- arduino-cli_0.19.2	 

The arduino executable should be in the arduino-cli_0.19.2 folder. If you use a different version of arduino-cli the name in buff.bash will need to be updated. It is not essential to use the arduino-cli, if you like the IDE better feel free to use that (this means you don't need to instal arduino-cli and you can ignore errrs related to that).

Load environment variables

	source buff.bash

Now run the install from the root of the project

	source scripts/install.bash 

After this installs the dependencies you will have full functionality.

To test the install run

	buff -h

you should see the output from above.

## Dev Notes

When working on this project
	- Document all install and setup procedures
	- Test changes and document the tests and changes
	- Make sure to push when you finish working, otherwise no one else sees your code
	- Do not push broken changes, this will break things for everyone
	- CV is done in the buffpy package
	- Controls are developed in src
	- The more documentation the better
	- 

## CHANGES
 - Version 0.0
   - Date: October 1, 2021
   - Editor: Mitchell D Scott
   - Status: Untested
   - Notes: brought the project into existence


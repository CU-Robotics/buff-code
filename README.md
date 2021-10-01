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

## CHANGES
 - Version 0.0
	Date: October 1, 2021
	Editor: Mitchell D Scott
	Status: Untested
	Notes: brought the project into existence


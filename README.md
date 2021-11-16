# Buff-Code
CU Robotics' development repo

This project is desinged to run in an Ubuntu20 environment or on a Jetson device (Windows support coming eventually). This repo provides a python3 package buffpy that is meant to aid in the development of robot software. The package contains a command line tool 'buff'. This tool helps streamline managing data, building locally or to devices, flashing firmware and hopefully soon running regressions on the developed subsystems.

First always load the environment variables, if you're not sure if you did just do it again. Also this script loads the path variables relative to where it was run. Make sure to run it from the root of this repo.

	source buff.bash
	
The majority of functionality is based in the buffpy package. This package is setup as a command line tool in buff.bash. All you need to utilize this package is call 'buff' from the command line.

	usage: buff [-h] [--sshBot] [--setBot ROBOT_IP] [--GDrive ACTION FOLDER_ID] [--installKeys] [--launch LOCATION]
						[--botPull] [--build PROFILE] [--clean] [--flash FQBN FW]

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
	--build PROFILE       Builds the workspace locally (debug) or to the robot_ip (install)
	--clean               Clean the current bin and data, NOT recoverable; only run this if you are sure you want to
	--flash FQBN FW       Flashes the given board with the target FW (expected port is ACM0)

## Install

Clone the repo to your machine

	git clone git@github.com:/CU-Robotics/buff-code.git

or if you don't have ssh keys setup

	git clone https://github.com/CU-Robotics/buff-code.git

Load environment variables

	source buff.bash

Now run the install from the root of the project

	source scripts/install.bash 

  *the install has a bug that may require this additional command*

    sudo apt install --reinstall ros-melodic-desktop-full

After this installs the dependencies you will have full functionality.

To test the install run

	source buff.bash
	buff -h

you should see the output from above.

## Architecture
buff-code
  - buffpy: A python package to handle the ugly backend
    - bin: binaries (buff, teensy)
    - lib: installed files
  - config: A place for any and all setup/configuration/secret files
    - install: Files containing install info
    - lib: File containing misc info
    - sensitive: shhh... it's a secret
  - data: a Temporary folder for handling data (should get cleared regularly, if missing will cause issues)
  - dev: python3 notebooks displaying our dev process
  - docs: A better verion of this document
  - scripts: arbitrary scripts
  - src: The source code for our controllers
    - buff_ros
      - Our ros package
    - teensy
      - teensy build files
  - buff.bash: a setup script (needs to be run every session)
  - README.md: you're reading it
  - .gitignore: keeps the secrets safe

## Dev Notes

When working on this project
  - Document all install and setup procedures
  - Test changes and document the tests and changes
  - Make sure to push when you finish working, otherwise no one else sees your code
  - Do not push broken changes, this will break things for everyone
  - CV is done in the buffpy package
  - Controls are developed in src
  - The more documentation the better 

## CHANGES
*Changes include all PRs that modify the directory structure, the installed binaries and any changes that will effect workspace usage*
 - Version 0.02
   - Date: October 19, 2021
   - Editor: Mitchell D Scott
   - Status: Only debug build and clean were tested
   - Description: 
      - Converted to a catkin workspace now installs ROS. 
      - The python lib files now live in src/buff_ros and get installed to buffpy/lib
 - Version 0.01
   - Date: October 7, 2021
   - Editor: Mitchell D Scott
   - Status: Stable
   - Description: added sub-directories to the config folder and moved buff and firmware binaries to bin and moved python files to lib
 - Version 0.00
   - Date: October 1, 2021
   - Editor: Mitchell D Scott
   - Status: Untested
   - Description: brought the project into existence


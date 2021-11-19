# Buff-Code
CU Robotics' development repo

This project is desinged to run in an Ubuntu20 environment or on a Jetson device (Windows support coming eventually). This repo provides a python3 package buffpy that is meant to aid in the development of robot software. The package contains a command line tool 'buff'. This tool helps streamline managing data, building locally or to devices, flashing firmware and hopefully soon running regressions on the developed subsystems.

First always load the environment variables, if you're not sure if you did just do it again. Also this script loads the path variables relative to where it was run. Make sure to run it from the root of this repo.

	source buff.bash
	
The majority of functionality is based in the buffpy package. This package is setup as a command line tool in buff.bash. All you need to utilize this package is call 'buff' from the command line.

	usage: buffpy [-h] [--sshBot] [--setBot ROBOT_IP] [--GDrive ACTION FOLDER_ID] [--installKeys] [--launch LOCATION]
						[--botPull] [--build PROFILE] [--clean] [--flash FQBN FW]

	CU-Robotics Digital House-Keeper

	optional arguments:
	-h, --help            show this help message and exit
	--sshBot              SSH into the bot at the ROBOT_IP env variable
	--setBot ROBOT_IP     Set the IP of the robot in development
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
	buffpy -h

you should see the output from above.

## Building

Now that we know buff works we can build the workspace. 

  buffpy --build debug

You can change the build by specifying a different profile.

  buffpy --build release

If you have issues building the workspace you probably need to source buff.bash or you can clean the workspace.

  buffpy --clean

Be careful with this command because it can remove things you don't want it too. This function will remove whatever is at $PROJECT_ROOT/data and $PROJECT_ROOT/buffpy/lib. By changing the PROJECT_ROOT variable from bash or in buff.bash you will change what gets removed.

## Installing to the robot

The buff executable supports installing the workspace to a Jetson Nano (this is how you should always install). After building the workspace run:

  buffpy --install

This will copy all of the lib and binary files as well as install scripts, buff.bash and config files. Make sure the $ROBOT_IP variable points to the machine you want or you might end up pushing to the work device. Adjust this variable in buff.bash if necessary.

# Launching

To launch the main cognition program use the launch binary (which is already on your path).

  run

This script spawns the necessary threads for the vision pipeline (roscore, buffvision and other debug nodes)

## Architecture
buff-code
  - buffpy: A python package to handle the ugly backend
    - bin: binaries (buffpy, teensy, run)
    - lib: installed files
  - config: A place for any and all setup/configuration/secret files
    - install: Files containing install info
    - lib: File containing misc info
    - sensitive: shhh... it's a secret
  - data: a Temporary folder for handling data (should get cleared regularly, if missing will cause issues)
  - dev: python3 notebooks displaying our dev process (will be moved to google drive)
  - docs: A better verion of this document
  - scripts: arbitrary scripts (mostly install)
  - src: The source code for our controllers
    - buff_ros
      - Our ros package
    - teensy
      - teensy build files
  - buff.bash: a setup script (needs to be run every development session)
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
 - Version 0.03
   - Date: November 10, 2021
   - Editor: Mitchell D Scott
   - Status: Install, Build and Clean have been tested with standard input
   - Description: 
      - Removed google drive functionailty from buff so now buff works without building
      - BuffVision now runs python2 to support cv_bridge
      - Moved scripts/main.py to buffpy/bin/run
      - Moved buffpy/bin/buff to buffpy/bin/buffpy
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


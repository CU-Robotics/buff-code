# Buff-Code
CU Robotics' development repo

This project is a workspace with tools for building, installing and analyzing code on multiple robots. buff-code is desinged to run in an Ubuntu18 environment or on a Jetson device (you can also launch it in a docker container). 

This repo provides a few tools that users should become familiar with. The tools are in the python3 package buffpy in the project's root. The two most useful tools are buffpy and run. buffpy is used to build/clean, install to a robot, ssh to a robot and hopefully soon flash a microcontroller. buffpy has been refered to as a digital mom. It's really just a collection of ros andlinux tools in a python CLI. The novel part of buff-code is the ability to debug and manage multiple robots. 

In addition to buffpy there is a tool called run. run will read in a system config yaml or a python script and launch all the necessary programs. The wiki has much more documentation on these so be sure to check it out. 

## Quick start
checkout the [Getting started wiki](https://github.com/CU-Robotics/buff-code/wiki/Getting-Started) for more details.

### Ubuntu

Clone this repo into your home directory (/home/$USER/buff-code).

    git clone git@github.com:CU-Robotics/buff-code.git
    
Now run the install from the root of the project

	source buffpy/scripts/install.bash 

First always load the environment variables, if you're not sure if you did just do it again. Also this script loads the path variables relative to where it was run. Make sure to run it from the root of this repo (usually /home/$USER/buff-code).

	source buffpy/buff.bash
	
The majority of functionality is based in the buffpy package. This package is setup as a command line tool in buff.bash. All you need to utilize this package is call 'buffpy' from the command line.

	usage: buffpy [-h] [--sshBot] [--GDrive ACTION FOLDER_ID] [--installKeys] [--launch LOCATION]
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
	
To launch a system use the run command:

    run <name_of_config>.yaml
  
run will spawn all of the nodes defined in the config as well as uploading any config files to the rosparam  server.

run can also be used to spawn a python script (the script must be in buffpy/lib):

    run <python_script>.py

## Architecture
buff-code
  - buffpy: A python package to handle the ugly backend
    - bin: binaries (buffpy, teensy, run)
    - lib: installed files (only python3 atm)
    - config: A place for any and all setup/configuration/secret files
      - install: Files containing install info
      - lib: File containing misc info
      - sensitive: shhh... it's a secret
    - scripts: arbitrary scripts (installs & entrypoints)
    - buff.bash: a setup script (needs to be run every development session, docker runs this automatically)
  - data: a Temporary folder for handling data (should get cleared regularly, if missing will cause issues)
  - docs: A better verion of this document (moving to wiki)
  - src: The source code for our controllers
    - buff_ros
      - Our misc ros package
    - omega
      - Camera node src
    - crosshair
      - Detector node src
    - hunter
      - Tracker node src
    - wrecker
      - Controls src
    - echo
      - Serial node src (python)
    - tech
      - N/A
  - README.md: you're reading it
  - .gitignore: keeps the secrets safe

## Dev Notes

When working on this project
  - Document all install and setup procedures
  - Test changes and document the tests and changes
  - Make sure to push when you finish working, otherwise no one else sees your code
  - Do not push broken changes, this will break things for everyone
  - The more documentation the better 

## CHANGES
*Changes include all PRs that modify the directory structure, the installed binaries and any changes that will effect workspace usage*
 - Version 0.05
   - Date: February 19, 2022
   - Editor: Mitchell D Scott
   - Status: Mostly tested (docker, run, all sub-systems)
   - Description: 
      - Moved some folders into buffpy for smoother install
      - began setup for multi-agent install
      - crosshair model loading, predictions and debug looks ok
      - small gdrive improvements (can set the name of a downloaded file and download files instead of folders)
 - Version 0.04
   - Date: January 11, 2022
   - Editor: Mitchell D Scott
   - Status: Mostly tested (docker, run, some sub-systems)
   - Description: 
      - The source for different projects was moved into their own ros packages
      - cv_bridge python3 fix in docker (not regular install)
      - run uploads params to the rosparam server (now its is the only program that reads configs, it will read all of them, see run wiki)
      - Everything python3 seems to work...
      - Teensy src path change may have unpredicted affects
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


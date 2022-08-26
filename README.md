![alt text](https://github.com/CU-Robotics/buff-code/blob/master/buffcode-card.png?raw=true)


### TODO

Omega
- Wrote camera/video publisher
- ROS package will be deprecated soon (Luxonis through Crosshair)

Crosshair
- Write a program to detect armor plates in an image
    - developed YOLOv5 in demo notebook (trained with NYU's dataset)
    - integrated model download and execution script
    - setup debug topic (image_annotated)
- Convert detection script to the input output handler of the Luxonis camera (temporary, Hunter's ros package will take over) 

Hunter
- Write a program that tracks and computes a control signal for the turret.
    - developed 2d tracking demo notebook (dead reckoning)
    - integrated tracker with repo
    - setup generic equations for trajectory updating (on detections) and yaw pitch control signals
- Needs to be tuned for our robots
- Need to write a diagnostic program
- Move detections and tracking onto Luxonis hardware (requires debug/diagnostic modifications)

Echo
- Wrote a serial to ROS script
    - publishes data from the serial port to a topic defined by the serial message
    - subscribes to a set of topics and writes the message to the serial device
- Needs hardware testing
- Will integrate with the firmware once it's functional


### CU Robotics' Development Repository

Buff-Code is Multi-Agent build, deployment and management system. Buff-Code is supported in Ubuntu18 or on a Jetson device (Also we support docker). 

This repo provides a few tools that users should become familiar with. The tools are in the python3 package BuffPy in the project's root. The two most useful tools are BuffPy and Run. BuffPy is used to build/clean, install to a robot, ssh to a robot and hopefully soon flash a microcontroller. It's really just a collection of ros and linux tools in a python CLI. The novel part of Buff-Code is the ability to debug and manage multiple robots. 

## Quick start
checkout the [Getting started wiki](https://github.com/CU-Robotics/buff-code/wiki/Getting-Started) for more details.

### Ubuntu

Clone this repo into your home directory (/home/$USER/buff-code).

    git clone git@github.com:CU-Robotics/buff-code.git
    
Now run the install from the root of the project

	source buffpy/scripts/install.bash 

First always load the environment variables, if you're not sure if you did, just do it again. Also this script loads the path variables relative to where it was run. Make sure to run it from the root of this repo (usually /home/$USER/buff-code).

	source buffpy/buff.bash
	
The majority of functionality is based in the buffpy package. This package is setup as a command line tool in buff.bash. All you need to utilize this package is call 'buffpy' from the command line.

	usage: /home/m_dyse/buff-code/buffpy/bin/buffpy [-h] [--installKeys] [--launch LOCATION] [--botPull] [--build PROFILE] [--install] [--diagnostic] [--initialize] [--clean]

        CU-Robotics Multi-Agent Deployment Manager

        optional arguments:
          -h, --help         show this help message and exit
          --installKeys      Push local sshkeys to the robots
          --launch LOCATION  Launch the robots software on robots
          --botPull          Pull data from the robot at ROBOT_IP
          --build PROFILE    Builds the workspace locally (use profile debug)
          --install          Installs build to the registered robots
          --diagnostic       Tests a workspace installation and tools
          --initialize       Initializes registered devices
          --clean            Clean the current bin and data, NOT recoverable; only run this if you are sure you want to
	
To launch a system use the run command:

    run <name_of_config>.yaml
  
run will spawn all of the nodes defined in the config as well as uploading any config files to the rosparam server.

run can also be used to spawn a python script (the script must be in buffpy/lib):

    run <python_script>.py

## Architecture
buff-code
  - buffpy: A python package to handle the ugly backend
    - bin: binaries (buffpy, teensy, run)
    - lib: installed files (only python3 atm)
    - config: A place for any and all setup/configuration/secret files
      - install: Files containing install info
      - system: Folder of system yamls
      - data: Folder containing misc info for programs
      - sensitive: shhh... it's a secret
    - scripts: arbitrary scripts (installs & entrypoints)
    - buff.bash: a setup script (needs to be run every development session, docker runs this automatically)
  - data: a Temporary folder for handling data (should get cleared regularly, if missing will cause issues)
  - docs: Removing soon, I swear ...
  - container
    - Base image dockerfile, the dev image needs a base
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
  - Dockerfile: The dev file for our docker image (requires base image)

## Dev Notes

When working on this project
  - Document all install and setup procedures
  - Test changes and document the tests and changes
  - Make sure to push when you finish working, otherwise no one else sees your code
  - Do not push broken changes, this will break things for everyone
  - The more documentation the better 

Version info
  - Major number: increments when large dependency changes occur (eg python3.6 -> python3.9, or Melodic -> Noetic)
  - Minor number: increments with patches and edits to workspace tools

## CHANGES
*Changes include all PRs that modify the directory structure, the installed binaries and any changes that will effect workspace usage*
 - Version 0.07
   - Date: June 30, 2022
   - Editor: Mitchell D Scott
   - Status: Seems stable
   - Description: 
      - Odyssey is functional
      - buffpy makes use of platformio and cargo to build files
      - buffpy can successfully install to all devices
      - buffpy installation is still a little buggy (doesn't include cargo atm)
 - Version 0.06
   - Date: March 17, 2022
   - Editor: Mitchell D Scott
   - Status: Mostly tested (issues with buffnet and serial understood)
   - Description: 
      - Split yamls into seperate folder (system, data)
      - Major Install patches runs successfully on Jetson and Ubuntu 18, remote initialize not ready
      - BuffPy upgrades, needed to handle new installation
      - Docker containers are now out of date and need an upgrade (should be functional though)
      - Systemd service created but untested and likely not functional yet
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


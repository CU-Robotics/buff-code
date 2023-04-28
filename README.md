![alt text](https://github.com/CU-Robotics/buff-code/blob/mdsdev/title-card.png?raw=true)

### CU Robotics' Code Repository

Buff-code is a workspace manager for software systems, it is a minimalized implementation of some ROS features. Buff-Code's purpose is to provide development tools to engineers building and designing perception systems, firmware drivers, interdevice communications and control algorithms. Given a set of projects Buff-Code will build an installable package of executables and configurations. Buff-Code will also remotely deploy the pakcages to any number of robots with any number of robot types. While Buff-Code replaces some ROS features, it relies heavily on others. Once a system is built and deployed engineers use standard rosbash tools to connect to a roscore (on a robot) and visualize different aspects of the system.

## Quick start
checkout the [Getting started wiki](https://github.com/CU-Robotics/buff-code/wiki/Getting-Started) for more details.

### Ubuntu

Clone this repo into your home directory (/home/$USER).

    git clone git@github.com:CU-Robotics/buff-code.git

First always load the environment variables, if you're not sure you did, just do it again. Also this script loads the path variables relative to where it was run. Make sure to run it from the root of this repo (usually /home/$USER/buff-code).

	source buffpy/buff.bash

Now run the install from the root of the project (/home/$USER/buff-code)

	source buffpy/scripts/install.bash 

The majority of functionality is based in the buffpy package. This package is setup as a command line tool in buff.bash. All you need to utilize this package is call 'buffpy' from the command line.

	usage: /home/m_dyse/buff-code/buffpy/bin/buffpy [-h] [--installKeys] [--launch LOCATION] [--botPull] [--build PROFILE] [--install] [--diagnostic] [--initialize] [--clean]

        CU-Robotics Multi-Agent Deployment Manager

        optional arguments:     Deprecated
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

    run <name_of_robot>
  
run will spawn all of the nodes defined in the config as well as uploading any config files to the rosparam server. The robots are all defined in buffpy/config/robots

## Architecture Deprecated (someone should fix this)
buff-code
  - buffpy: A python package to handle the ugly backend
    - bin: binaries (buffpy, teensy, run)
    - lib: installed files (only python3 atm)
    - config: A place for any and all setup/configuration/secret files
      - install: Files containing install info
      - robots: Folder of node configurations
      - data: Folder containing misc info for programs
      - sensitive: shhh... it's a secret
    - scripts: arbitrary scripts (installs & entrypoints)
    - buff.bash: a setup script (needs to be run every development session, docker runs this automatically)
  - data: a Temporary folder for handling data (should get cleared regularly, if missing will cause issues)
  - docs: Removing soon, I swear ...
  - container
    - Base image dockerfile, the dev image needs a base
    - Dev image dockerfile
  - src: The source code for our controllers
    - buff_rust
      - Our rust nodes, the cognition center of the robot
    - firware
      - firware for teensy configured with PlatformIO
    - crosshair
      - Detector node src
    - hunter
      - Tracker node src
  - README.md: you're reading it
  - .gitignore: keeps the secrets safe

## Dev Notes

When working on this project
  - Document all install and setup procedures
  - Test changes and document the tests and changes
  - Make sure to push when you finish working, otherwise no one else sees your code
  - Do not push broken changes, this will break things for everyone

Version info
  - Major number: increments when large dependency changes occur (eg python3.6 -> python3.9, or Melodic -> Noetic)
  - Minor number: increments with patches and edits to workspace tools (first digit increments for incompatibilities)

## CHANGES
*Changes include all PRs that modify the directory structure, the installed binaries and any changes that will effect workspace usage*
 - Version 2.10
   - Date: April, 2023
   - Editor: Mitchell D Scott
   - Status: back to unstable (due to the incident)
   - Description: 
      - Firmware uses a makefile and arduino2 install to build/upload
      - Updates to controls and hid
      - moved safety switch to dr16 (cause of the incident) NEEDS TESTING ASAP
      - progress on perception, blocked by npu issue, quadtree put on hold (but still exists, please someone do something with it)
      - Updated docker containers ~recently
      - removed lots of deprecated things
 - Version 2.00
   - Date: Decemberish, 2022
   - Editor: Mitchell D Scott
   - Status: Arguably stable
   - Description: 
      - Controls back to firmware, though hid initializes all gains, control types, motor configurations etc
      - PC side HID was split into 3 threads, one for reading, one for writing 
      - Major changes to the yaml configurations that make buffpy
      - Corresponding updates to buffpy scripts to reflect config changes
      - demos for unit testing in fw, still leaves something to be desired
      - Major dependencies update (phasing out melodic and python3.6, mostly gone)
      - Docker containers are old and need an update
 - Version 1.00
   - Date: August 26, 2022
   - Editor: Mitchell D Scott
   - Status: Seems stable
   - Description: 
      - We are rustaceans now
      - firmware was gutted and now is mostly just IO
      - controls are migrating to rust (and ros)
      - run no longer supports scripts and now uses better definitions of robots to start systems
      - buffpy got the usual adjustments aside from dig/clean not much is new
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


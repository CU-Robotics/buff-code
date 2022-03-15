#! /bin/bash


# Doesn't Actually work :/

#
#  Export some variables
#

export ROS_PKG=ros-base					# Basic ROS (haha only communication and services)
export ROS_DISTRO=melodic				# ROS for Ubuntu18

export PROJECT_ROOT=$PWD				# Path to buff-code
export DEBIAN_FRONTEND=noninteractive	# prevent prompts in docker and everywhere else


#
#	Assert start-up directory
#

echo -e "Running purge from ${PWD}"

if [[ $PROJECT_ROOT != *"/buff-code" ]]; then
	echo -e "source buffpy/buff.bash and run this script"
	exit
fi 


#
#	Source buff.bash
#

source ${PROJECT_ROOT}/buffpy/buff.bash


#
#	Uninstall BuffCode Python requirments
#

pip uninstall -r ${PROJECT_ROOT}/buffpy/config/install/ros_dependencies.txt
pip uninstall -r ${PROJECT_ROOT}/buffpy/config/install/python3_requirements.txt
pip uninstall pip

#
#	Uninstall BuffCode Dependencies
#

sudo xargs apt purge -y <${PROJECT_ROOT}/buffpy/config/install/dependencies.txt
sudo apt purge -y ros-${ROS_DISTRO}-${ROS_PKG} ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-common-plugins

if [[ "${HOSTNAME}" != "edge"* ]]; then
	sudo apt purge -y ros-${ROS_DISTRO}-robot-plugins
fi

sudo rm -rf /home/cu-robotics/opencv_ws

sudo apt autoremove -y
sudo apt clean
sudo apt update


#
#	Install Utilities
#

if [[ "${HOSTNAME}" != "edge"* ]]; then
	echo -e "\n\tPurge Purge Purge...\n"

	# #	Pull teensy files from pjrc.com
	# # teensy binary and objects
	# curl https://www.pjrc.com/teensy/teensy_linux64.tar.gz -O
	# # teensy rules file
	# curl https://www.pjrc.com/teensy/00-teensy.rules -O

	# # mv rules into rules.d and set the proper file permissions
	# mv 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
	# chmod 0644 /etc/udev/rules.d/00-teensy.rules

	# # extract the tar to buffpy/bin
	# tar -xvsf teensy_linux64.tar.gz -C ${PROJECT_ROOT}/buffpy/bin
	# # remove unecessary tar.gz 
	# rm teensy_linux64.tar.gz

else
	#	Copy our startup service to the system units directory
	sudo rm -rf /etc/systemd/system/buffbot.service

fi



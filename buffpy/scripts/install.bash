#! /bin/bash


#
#  Export some variables
#

export ROS_PKG=ros-base					# Basic ROS (haha only communication and services)
export ROS_DISTRO=melodic				# ROS for Ubuntu18

export PROJECT_ROOT=$PWD				# Path to buff-code
export DEBIAN_FRONTEND=noninteractive	# prevent prompts in docker and everywhere else


#
#	Source buff.bash
#

source ${PROJECT_ROOT}/buffpy/buff.bash


#
#	Update the apt package manager
#

echo -e "\n\tapt updating...\n"

sudo apt update

# Make some space if on Jetson
if [[ "${HOSTNAME}" == "edge"* ]]; then
	sudo apt purge -y thunderbird libreoffice-*
fi

sudo apt upgrade -y


#
#	Install BuffCode
#

source ${PROJECT_ROOT}/buffpy/scripts/install_buffpy.bash

sudo apt autoremove -y	
sudo apt clean
sudo apt update


#
#	Check for ROS install (installs if none)
#

if [[ ! -d /opt/ros/${ROS_DISTRO} ]]; then
	source "${PROJECT_ROOT}/buffpy/scripts/install_ros.bash"
fi

sudo apt autoremove -y
sudo apt clean
sudo apt update


#
#	Also install Sublime Text-editor
# Deprecated, IDE on edge devices/containers is slow
# source "${PROJECT_ROOT}/buffpy/scripts/install_sublime.bash"


#
#	Install Utilities
#

if [[ "${HOSTNAME}" == "edge"* ]]; then
	echo -e "\n\tInstalling TYcmd...\n"

	# #	Pull teensy files from pjrc.com
	# # teensy binary and objects
	# curl https://www.pjrc.com/teensy/teensy_linux64.tar.gz -O
	# # teensy rules file
	# curl https://www.pjrc.com/teensy/00-teensy.rules -O

	# # mv rules into rules.d and set the proper file permissions
	# sudo mv 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
	# sudo chmod 0644 /etc/udev/rules.d/00-teensy.rules

	# # extract the tar to buffpy/bin
	# tar -xvsf teensy_linux64.tar.gz -C ${PROJECT_ROOT}/buffpy/bin
	# # remove unecessary tar.gz 
	# rm teensy_linux64.tar.gz
	sudo cp ${PROJECT_ROOT}/buffpy/scripts/buffbot.service /etc/systemd/system

fi



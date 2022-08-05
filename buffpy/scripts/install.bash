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

source "${PROJECT_ROOT}/buffpy/scripts/install_tytools.bash"

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -sSf | sh -s -- -y

if [[ "${HOSTNAME}" == "edge"* ]]; then

	sudo cp ${PROJECT_ROOT}/buffpy/scripts/buffbot.service /etc/systemd/system

fi



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

#	Setup Teensy Rules
curl https://www.pjrc.com/teensy/00-teensy.rules -O

# mv rules into rules.d and set the proper file permissions
sudo mv 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
sudo chmod 0644 /etc/udev/rules.d/00-teensy.rules

if [[ "${HOSTNAME}" == "edge"* ]]; then
	echo -e "\n\tInstalling TYcmd...\n"

	cd "${PROJECT_ROOT}/.."

	sudo apt-get install build-essential cmake libudev-dev qtbase5-dev pkg-config

	wget https://github.com/Koromix/tytools/archive/refs/tags/v0.9.7.tar.gz

	tar -xf v0.9.7.tar.gz

	cd tytools-0.9.7
	
	mkdir -p build/linux && cd build/linux
	cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../..
	make

	sudo make install

	cd "${PROJECT_ROOT}"

	rm -rf ../tytools-0.9.7

	sudo cp ${PROJECT_ROOT}/buffpy/scripts/buffbot.service /etc/systemd/system

fi



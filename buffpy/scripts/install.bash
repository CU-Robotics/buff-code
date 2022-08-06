#! /bin/bash


#
#  Export some variables
#

export UBUNTU_VERSION=$(cut -f2 <<< $(lsb_release -r))
export DEBIAN_FRONTEND=noninteractive	# prevent prompts in docker and everywhere else

export ROS_PKG=ros-base

if [[ "${UBUNTU_VERSION}" == "20.04" ]]; then
	export ROS_DISTRO=noetic				# ROS for Ubuntu18
elif [[ "${UBUNTU_VERSION}" == "18.04" ]]; then
	export ROS_DISTRO=melodic
fi

#
#	Source buff.bash
#

source ${PROJECT_ROOT}/buffpy/buff.bash


#
#	Update the apt package manager
#

echo -e "\n\tapt updating...\n"

$SUDO apt update

# Make some space if on Jetson
if [[ "${HOSTNAME}" == "edge"* ]]; then
	$SUDO apt purge -y thunderbird libreoffice-*
fi

$SUDO apt upgrade -y


#
#	Install BuffCode
#

source ${PROJECT_ROOT}/buffpy/scripts/install_buffpy.bash

$SUDO apt autoremove -y	
$SUDO apt clean
$SUDO apt update


#
#	Check for ROS install (installs if none)
#

if [[ ! -d /opt/ros/${ROS_DISTRO} ]]; then
	source ${PROJECT_ROOT}/buffpy/scripts/install_ros.bash
fi

$SUDO apt autoremove -y
$SUDO apt clean
$SUDO apt update


#
#	Also install Sublime Text-editor
# Deprecated, IDE on edge devices/containers is slow
# source "${PROJECT_ROOT}/buffpy/scripts/install_sublime.bash"


#
#	Install Utilities
#

source ${PROJECT_ROOT}/buffpy/scripts/install_tytools.bash

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -sSf | sh -s -- -y

echo "source ${HOME}/.cargo/env" >> ~/.bashrc

if [[ "${DOCKER}" == "False" ]]; then
	source ${PROJECT_ROOT}/buffpy/scripts/install_docker.bash
fi

if [[ "${HOSTNAME}" == "edge"* ]]; then

	$SUDO cp ${PROJECT_ROOT}/buffpy/scripts/buffbot.service /etc/systemd/system

fi



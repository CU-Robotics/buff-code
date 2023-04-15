#! /bin/bash


#
#  Export some variables
#

export UBUNTU_VERSION=$(cut -f2 <<< $(lsb_release -r))
export DEBIAN_FRONTEND=noninteractive	# prevent prompts in docker and everywhere else

#		Setup robot params
export DOCKER=False
export PROJECT_ROOT=${PWD}
export HOSTNAME=$HOSTNAME 
export SUDO='sudo'

if [[ -f /.dockerenv ]]; then
	SUDO=''
	DOCKER=True
	PROJECT_ROOT=/home/cu-robotics/buff-code
fi

if [[ "${UBUNTU_VERSION}" == "22.04" ]]; then
	export ROS_DISTRO=humble

elif [[ "${UBUNTU_VERSION}" == "20.04" ]]; then
	export ROS_DISTRO=noetic				# ROS for Ubuntu18

elif [[ "${UBUNTU_VERSION}" == "18.04" ]]; then
	export ROS_DISTRO=melodic
fi

export ROS_PKG=desktop


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
	if [[ "${UBUNTU_VERSION}" == "22.04" ]]; then
		source ${PROJECT_ROOT}/buffpy/scripts/install_ros2.bash
	
	else
		source ${PROJECT_ROOT}/buffpy/scripts/install_ros.bash
	fi
	
fi

$SUDO apt autoremove -y
$SUDO apt clean
$SUDO apt update


#
#	Install Utilities
#

source ${PROJECT_ROOT}/buffpy/scripts/install_tytools.bash

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -sSf | sh -s -- -y

if [[ "${HOSTNAME}" == "edge"* ]]; then
	#	install startup script
	$SUDO cp ${PROJECT_ROOT}/buffpy/scripts/buffbot.service /etc/systemd/system

	# 	install realsense
	source ${PROJECT_ROOT}/buffpy/scripts/install_realsense_source.bash

elif [[ "${DOCKER}" == "False" ]]; then
	#	install docker
	source ${PROJECT_ROOT}/buffpy/scripts/install_docker.bash

	#	install gazebo
	curl -sSL http://get.gazebosim.org | sh

	#	install realsense
	sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
	sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
	sudo apt install librealsense2-dkms
	sudo apt install librealsense2-utils
	sudo apt install librealsense2-dev
fi



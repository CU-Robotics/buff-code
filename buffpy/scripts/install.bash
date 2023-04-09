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
    echo "WARNING: You are using Ubuntu 22.04. ros noetic is not supported, but we will attempt installation from source anyway... (^C to cancel)"
    sleep 10
	export ROS_DISTRO=noetic
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
		# source ${PROJECT_ROOT}/buffpy/scripts/install_ros2.bash
        source ${PROJECT_ROOT}/buffpy/scripts/install_ros_source.bash
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
	$SUDO cp ${PROJECT_ROOT}/buffpy/scripts/buffbot.service /etc/systemd/system
elif [[ "${DOCKER}" == "False" ]]; then
	source ${PROJECT_ROOT}/buffpy/scripts/install_docker.bash
	curl -sSL http://get.gazebosim.org | sh
fi



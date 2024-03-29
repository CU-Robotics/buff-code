#! /bin/bash

export UBUNTU_VERSION=$(cut -f2 <<< $(lsb_release -r))

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

if [[ "${UBUNTU_VERSION}" == "20.04" ]]; then
	export ROS_DISTRO=noetic				# ROS for Ubuntu18
elif [[ "${UBUNTU_VERSION}" == "18.04" ]]; then
	export ROS_DISTRO=melodic
fi


if [[ "${DOCKER}" == "False" ]]; then
	alias spinup="cd ${PROJECT_ROOT}/containers && \
		docker compose run "
	
else
	export LC_ALL=C.UTF-8
    export LANG=C.UTF-8
fi

if [[ "${PROJECT_ROOT}" != */buff-code ]]; then
	echo -e "running from ${PWD}, is this your project root?"
	return
fi

PYTHONPATH=

# If ROS is installed source the setup file

if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
	source /opt/ros/${ROS_DISTRO}/setup.bash
fi

#		setup Cargo tools

if [[ "${PATH}" != *"/.cargo"*  && -f ${HOME}/.cargo/env ]]; then
	source ${HOME}/.cargo/env
fi

#		Setup python tools

if [[ "${PATH}" != *"${PROJECT_ROOT}/buffpy/scripts"* ]]; then
	export PATH="${PROJECT_ROOT}/buffpy/scripts:${PATH}"
fi 

# if [[ "${PATH}" != *"/.local/bin:"* ]]; then
# 	export PATH="${HOME}/.local/bin:${PATH}"
# fi 

# Only export if if not already in path

if [[ "${PYTHONPATH}" != *"${PROJECT_ROOT}/buffpy/lib:"* ]]; then	
	export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}" 
fi

# set ROS package path to buff-code so it can see buffpy

if [[ "${ROS_PACKAGE_PATH}" != *"buff-code"* ]]; then
	export ROS_PACKAGE_PATH="${PROJECT_ROOT}:${ROS_PACKAGE_PATH}"
fi

# Not totally clear but this solves an 
# illegal instruction error with rospy.
# Only for Jetson
# the status of this issue needs to be double checked
if [[ "${HOSTNAME}" == "edge"* ]]; then
	export OPENBLAS_CORETYPE=ARMV8
	export USER_IP= # Should find a way to get the users IP from the robot, ssh env variable?
else
	# if not on jetson set the user IP
	# should figure out how to set it if it is on the jetson
	# export USER_IP=$(/sbin/ip -o -4 addr list wlp3s0 | awk '{print $4}' | cut -d/ -f1) # Needs testing

	alias bc="cd ${PROJECT_ROOT}"
	alias br="cd ${PROJECT_ROOT}/src/buff_rust"
	alias fw="cd ${PROJECT_ROOT}/src/firmware"
	alias buildr="buffpy --build rust"
	alias buildf="buffpy --build firmware"
	alias builda="buffpy --build all"
	alias tnsy-upload="buffpy --upload"
	alias setup-live-tests="roscore & sleep 2 && rosparam set /buffbot/robot_name penguin"
	alias buff-test="br && cargo test"
fi


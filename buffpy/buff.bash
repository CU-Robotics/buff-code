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

#		Setup python tools (if reset replace the bin links)
if [[ "$1" == "reset" ]]; then
	if [[ -f "/usr/local/bin/buffpy" ]]; then
		${SUDO} rm -rf "/usr/local/bin/buffpy"
	fi
	if [[ -f "/usr/local/bin/run" ]]; then
		${SUDO} rm -rf "/usr/local/bin/run"
	fi
fi

if [[ ! -f "/usr/local/bin/buffpy" ]]; then
	${SUDO} touch "/usr/local/bin/buffpy"
	echo "/usr/bin/env python3 \${PROJECT_ROOT}/buffpy/src/cli.py \$@" | ${SUDO} tee "/usr/local/bin/buffpy"
	${SUDO} chmod +x "/usr/local/bin/buffpy"
fi 
if [[ ! -f "/usr/local/bin/run" ]]; then
	${SUDO} touch "/usr/local/bin/run"
	echo "/usr/bin/env python3 \${PROJECT_ROOT}/buffpy/src/robot_spawner.py \$@" | ${SUDO} tee "/usr/local/bin/run"
	${SUDO} chmod +x "/usr/local/bin/run"
fi


# Only export if if not already in path
if [[ "${PYTHONPATH}" != *"${PROJECT_ROOT}/buffpy/lib:"* ]]; then	
	export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}" 
fi
if [[ "${PYTHONPATH}" != *"${PROJECT_ROOT}/buffpy/src:"* ]]; then	
	export PYTHONPATH="${PROJECT_ROOT}/buffpy/src:${PYTHONPATH}" 
fi

if [[ "${PYTHONPATH}" != *"*:/usr/lib/python3.8/site-packages"* ]]; then	
	export PYTHONPATH="${PYTHONPATH}:/usr/lib/python3.8/site-packages"
fi

# set ROS package path to buff-code so it can see buffpy
if [[ "${ROS_PACKAGE_PATH}" != *"buff-code"* ]]; then
	export ROS_PACKAGE_PATH="${PROJECT_ROOT}:${ROS_PACKAGE_PATH}"
fi

# Not totally clear but this solves an 
# illegal instruction error with rospy.
# Only for Jetson
# the status of this issue needs to be double checked
alias bc="cd ${PROJECT_ROOT}"
alias br="cd ${PROJECT_ROOT}/src/buff_rust"
alias fw="cd ${PROJECT_ROOT}/src/firmware"
alias bn="cd ${PROJECT_ROOT}/src/rknn_buffnet"

alias monitor="tycmd monitor --reconnect"

if [[ "${HOSTNAME}" == "edge"* ]]; then
	export OPENBLAS_CORETYPE=ARMV8
	export ROS_IP=$(/sbin/ip -o -4 addr list wlan0 | awk '{print $4}' | cut -d/ -f1)
	export ROS_MASTER_URI=http://${ROS_IP}:11311
else
	# if not on jetson set the user IP
	# should figure out how to set it if it is on the jetson
	# export USER_IP=$(/sbin/ip -o -4 addr list wlp3s0 | awk '{print $4}' | cut -d/ -f1) # Needs testing

	alias buildr="buffpy -b rust-debug"
	alias buildf="buffpy -b fw"
	alias builda="buffpy -b all"
	alias buff-test="br && cargo test"
	alias sshbot="ssh -X cu-robotics@edgek.local"
	alias scp-src="buffpy -c rust && scp -r src/buff_rust/ cu-robotics@edgek.local:/home/cu-robotics/buff-code/src/"
	set-ros-master () {
		export ROS_MASTER_URI=http://$1:11311
	}
fi


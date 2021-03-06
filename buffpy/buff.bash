#! /usr/bin/bash


if [[ "${PWD}" != */buff-code ]]; then
	echo -e "running from ${PWD}, is this your project root?"
	return
fi

#		Setup robot params
export DOCKER=False
export PROJECT_ROOT=${PWD}
export HOSTNAME=$HOSTNAME 

if [[ -f /proc/1/cgroup ]]; then
	if grep -q docker /proc/1/cgroup; then 
	   DOCKER=True
	elif [[ "${HOSTNAME}" == "docker-desktop" ]]; then
		DOCKER=True
	fi
fi


if [[ "${DOCKER}" == "False" ]]; then
	if [[ "$(uname)" == "MINGW"* ]]; then
		alias spinup="winpty docker run -it \
		-v ${PROJECT_ROOT}:/home/cu-robotics/buff-code \
		-e DISPLAY=host.docker.internal:0 \
		--net=host "
	else
		alias spinup="docker run -it \
		-e DISPLAY=host.docker.internal:0 \
		-v ${PROJECT_ROOT}:/home/cu-robotics/buff-code \
		--net=host "
	fi
else
	export LC_ALL=C.UTF-8
    export LANG=C.UTF-8
fi

PYTHONPATH=

# If ROS is installed source the setup file

if [[ -f /opt/ros/melodic/setup.bash ]]; then
	source /opt/ros/melodic/setup.bash
fi


#	DEPRECATED
# if [[ "${DOCKER}" == "False" ]]; then
# 	export PYTHONPATH="${HOME}/.local/lib/python3.6/dist-packages:${PYTHONPATH}" 
# # else
# # 	export PYTHONPATH="/usr/local/lib/python3.6/dist-packages:${PYTHONPATH}" 
# fi

# DEPRECATED
# Only needed if we are using ros packages
# if [[ -d ${PROJECT_ROOT}/install ]]; then
# 	source {PROJECT_ROOT}/install/setup.bash
# fi

#		Setup python tools

if [[ "${PATH}" != *"${PROJECT_ROOT}/buffpy/bin"* ]]; then
	export PATH="${PROJECT_ROOT}/buffpy/bin:${PATH}"
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
# else
	# if not on jetson set the user IP
	# should figure out how to set it if it is on the jetson
	# export USER_IP=$(/sbin/ip -o -4 addr list wlp3s0 | awk '{print $4}' | cut -d/ -f1) # Needs testing

fi

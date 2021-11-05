#! /usr/bin/bash

if [[ "${PWD}" != */buff-code ]]; then
	echo -e "running from ${PWD}, is this your project root?"
	return
fi

#		Setup robot params
export PROJECT_ROOT=${PWD}
export HOSTNAME=$HOSTNAME
export ROBOT_IP='10.0.0.161' # '128.138.157.251' #'10.0.0.160' for mitchell's home network
export ROBOT_ADDRESS="cu-robotics@${ROBOT_IP}"
export ROBOT_ROOT="/home/cu-robotics/buff-code"

# If ROS is installed source the setup file
if [[ -d /opt/ros/melodic ]]; then
	source /opt/ros/melodic/setup.bash
fi

# Only needed if we are using ros packages
# if [[ -d ${PROJECT_ROOT}/install ]]; then
# 	source {PROJECT_ROOT}/install/setup.bash
# fi

#		Setup python tools
if [[ "${PATH}" != *"buffpy"* ]]; then
	export PATH="${PROJECT_ROOT}/buffpy/bin:${PATH}"
fi 

# Only export if if not already in path
if [[ "${PYTHONPATH}" != *"buffpy"* ]]; then	
	export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}"
fi

# Not totally clear but this solves an 
# illegal instruction error with rospy.
# Only for Jetson
if [[ "${HOSTNAME}" == "edge"* ]]; then
	export OPENBLAS_CORETYPE=ARMV8
	export USER_IP= # Should find a way to get the users IP from the robot
else
	# if not on jetson set the user IP
	# should figure out how to set it if it is on the jetson
	export USER_IP=$(/sbin/ip -o -4 addr list wlp3s0 | awk '{print $4}' | cut -d/ -f1) # Needs testing

fi

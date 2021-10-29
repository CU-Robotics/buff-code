#! /usr/bin/bash
if [[ "${PWD}" != */buff-code ]]; then
	echo -e "running from ${PWD}, is this your project root?"
	return
fi

#		Setup robot params
export PROJECT_ROOT=${PWD}
export ROBOT_IP='10.0.0.21'
export ROBOT_ADDRESS="cu-robotics@${ROBOT_IP}"
export ROBOT_ROOT="/home/cu-robotics/buff-code"
#export USER_IP=$(/sbin/ip -o -4 addr list wlp3s0 | awk '{print $4}' | cut -d/ -f1) # Needs testing

# If ROS is installed source the setup file
if [[ -d /opt/ros/melodic ]]; then
	source /opt/ros/melodic/setup.bash
fi

# if [[ -d ${PROJECT_ROOT}/install ]]; then
# 	source {PROJECT_ROOT}/install/setup.bash
# fi

#		Setup python tools
export PATH="${PROJECT_ROOT}/buffpy/bin:${PATH}"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/lib:${PYTHONPATH}"

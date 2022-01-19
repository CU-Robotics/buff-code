#! /usr/bin/bash

if [[ "${PWD}" != */buff-code ]]; then
	echo -e "running from ${PWD}, is this your project root?"
	return
fi

#		Setup robot params
export PROJECT_ROOT=${PWD}
export HOSTNAME=$HOSTNAME 
				#	John-Os		# Idea Forge 		# John-Os 2		# Idea Forge	# Idea Forge		# John-Os		# Idea Forge		# Mitchells
export ROBOT_IP='128.138.157.151' #'10.0.0.160' # '128.138.157.122' # '10.0.0.162' #'128.138.157.240' # '128.138.157.140' #'10.0.0.161' # '128.138.157.251' #'10.0.0.160'
export ROBOT_ADDRESS="cu-robotics@${ROBOT_IP}"
export ROBOT_ROOT="/home/cu-robotics/buff-code"

if [[ "$(uname)" == "MINGW"* ]]; then
	alias spinup="winpty docker run -it \
	-e DISPLAY=host.docker.internal:0 \
	-v ${PROJECT_ROOT}:/home/cu-robotics/buff-code \
	--net=host "
else
	alias spinup="docker run -it \
	-e DISPLAY=host.docker.internal:0 \
	-v ${PROJECT_ROOT}:/home/cu-robotics/buff-code \
	--net=host "
fi

PYTHONPATH=

# If ROS is installed source the setup file
if [[ -f /opt/ros/melodic/setup.bash ]]; then
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
if [[ "${PYTHONPATH}" != *"/usr/lib/python3"* ]]; then	
	export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH}" 
fi

# Only export if if not already in path
if [[ "${PYTHONPATH}" != *"buffpy"* ]]; then	
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

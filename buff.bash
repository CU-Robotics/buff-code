#! /usr/bin/bash
if [[ "${PWD}" != */buff-code ]]; then
	echo -e "running from ${PWD}, is this your project root?"
	return
fi
#		Setup robot params
export PROJECT_ROOT=${PWD}
export USER_IP='192.168.0.9'
export ROBOT_IP='192.168.0.13'
export ROBOT_ADDRESS="cu-robotics@${ROBOT_IP}"
export ROBOT_ROOT="/home/cu-robotics/buff-code"

#		Setup python tools
export PATH="${PROJECT_ROOT}/buffpy:${PROJECT_ROOT}/buffpy/arduino-cli_0.19.2:${PATH}"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/bin:${PYTHONPATH}"

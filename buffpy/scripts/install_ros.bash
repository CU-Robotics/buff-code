#! /bin/bash


#
#	Setup ROS repositories
#

echo -e "\n\tSetting up ROS ${ROS_DISTRO} ${ROS_PKG}\n"

$SUDO sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


#
# Setup ROS keys
# 

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | $SUDO apt-key add -

$SUDO apt update


#
# Install ROS
#

echo -e "\n\tInstalling ros-${ROS_DISTRO}-${ROS_PKG}\n"

$SUDO apt install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG} ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-common-plugins ros-${ROS_DISTRO}-catkin ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rqt-robot-plugins python3-catkin-tools

$SUDO apt update


#
# Init rosdep
#

if [[ "${HOSTNAME}" != "edge"* ]]; then
	echo -e "\n\tSetting up rosdep\n"

	$SUDO apt update
	$SUDO apt install python3-rosdep2
	
	cd /opt/ros/${ROS_DISTRO}

	$SUDO rosdep init
	rosdep update
fi

$SUDO apt update 

#
#	Return to project
# 
cd ${PROJECT_ROOT}







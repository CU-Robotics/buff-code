#! /bin/bash

ROS_PKG=$2
ROS_DISTRO=$1 

echo -e "\n\tSetting up ROS ${ROS_DISTRO}\n"

# ROS installation
# add repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup ROS keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

# Install ROS

sudo apt install -y ros-${ROS_DISTRO}-${ROS_PKG}

# Install ROS dependencies
pip3 install -r -y ${PROJECT_ROOT}/config/install/ros_dependencies.txt

# clean the apt cache
rm -rf /var/lib/apt/lists/*

echo -e "\n\tFinishing ROS setup...\n"

# Init rosdep
sudo rosdep init
rosdep update

# clean cache
rm -rf /var/lib/apt/lists/*
#! /bin/bash


#
#	Setup ROS repositories
#

sudo apt install software-properties-common
sudo add-apt-repository universe

#
# Setup ROS keys
# 

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#
# Install ROS
#

echo -e "\n\tInstalling ros-${ROS_DISTRO}-${ROS_PKG}\n"

$SUDO apt update
$SUDO apt upgrade 

$SUDO apt install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG}


#
#	Return to project
# 
cd ${PROJECT_ROOT}







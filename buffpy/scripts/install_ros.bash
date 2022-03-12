#! /bin/bash

ROS_PKG=$2
ROS_DISTRO=$1 

echo -e "\n\tSetting up ROS ${ROS_DISTRO} ${ROS_DISTRO}\n"


# ROS installation
# add repositories
# sudo
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#
# setup ROS keys
#
# sudocurl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | 
apt-key add -

# sudo
apt update

#
# Install ROS
#
# sudo
apt install -y ros-${ROS_DISTRO}-${ROS_PKG}
echo -e "\n\tapt Installing ros-${ROS_DISTRO}-${ROS_PKG}\n"


#
# Install ROS dependencies
#

echo -e "\n\tpip Installing from ${PROJECT_ROOT}/buffpy/config/install/ros_dependencies.txt\n"
python3 -m pip install -r ${PROJECT_ROOT}/buffpy/config/install/ros_dependencies.txt


echo -e "\n\tFinishing ROS setup...\n"

#
# Init rosdep
#
# sudo
rm -rf /var/lib/apt/lists/*
# sudo
apt update
# sudo
apt install python-rosdep
# sudo
cd /opt/ros/melodic
rosdep init
rosdep update

#
# Install cv2 bridge for python3
#
# sudo
rm -rf /var/lib/apt/lists/*
# sudo
apt update 
cd ${PROJECT_ROOT}/..
echo -e "\n\tgit cloning vision_opencv to ${pwd}/opencv_ws\n"
mkdir opencv_ws 
cd opencv_ws
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
cd src/vision_opencv 
git checkout 1.13.0 
cd ../.. 
source /opt/ros/melodic/setup.bash
catkin init 
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so --install --extend /opt/ros/melodic
catkin build cv_bridge
cp -r install/lib/python3/dist-packages/* /usr/lib/python3/dist-packages/
cd $HOME
rm -rf opencv_ws
cd ${PROJECT_ROOT}







#! /bin/bash


#
#	Setup ROS repositories
#
echo -e "\n\tSetting up ROS ${ROS_DISTRO} ${ROS_PKG}\n"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


#
# Setup ROS keys
# 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update


#
# Install ROS
#
echo -e "\n\tInstalling ros-${ROS_DISTRO}-${ROS_PKG}\n"

sudo apt install -y ros-${ROS_DISTRO}-${ROS_PKG} ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-common-plugins

if [[ "${HOSTNAME}" != "edge"* ]]; then
	sudo apt install -y ros-${ROS_DISTRO}-robot-plugins
fi

sudo apt update
sudo apt dist-upgrade -y

#
# Install ROS dependencies
#
echo -e "\n\tInstalling ros dependencies\n"

python3 -m pip install -r ${PROJECT_ROOT}/buffpy/config/install/ros_dependencies.txt


#
# Init rosdep
#
echo -e "\n\tSetting up rosdep\n"

sudo apt update
sudo apt install -y python-rosdep

cd /opt/ros/${ROS_DISTRO}

sudo rosdep init
rosdep update


#
#	Source buff.bash
#
sudo apt update 
cd ${PROJECT_ROOT}
source ${PROJECT_ROOT}/buffpy/buff.bash

#
# Install cv2 bridge for python3
#

echo -e "\n\tCloning vision_opencv to ${pwd}/../opencv_ws\n"

cd ../ && mkdir opencv_ws && cd opencv_ws && \

git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \

cd src/vision_opencv && git checkout 1.13.0 && \

cd ../.. && source /opt/ros/melodic/setup.bash && \

echo -e "\n\tInstalling cv_bridge to /usr/lib/python3/dist-packages/" && \

catkin init && \

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so --install --extend /opt/ros/melodic && \

catkin build cv_bridge && \

cp -r install/lib/python3/dist-packages/* /usr/lib/python3/dist-packages/ && cd $HOME && \

rm -rf opencv_ws


#
#	Return to project
# 
cd ${PROJECT_ROOT}








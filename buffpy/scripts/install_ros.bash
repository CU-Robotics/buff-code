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

sudo apt install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG} ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-common-plugins

if [[ "${HOSTNAME}" != "edge"* ]]; then
	sudo apt install -y --no-install-recommends ros-${ROS_DISTRO}-robot-plugins
fi

sudo apt update


#
# Install ROS dependencies
#

echo -e "\n\tInstalling ros dependencies\n"

pip install -r ${PROJECT_ROOT}/buffpy/config/install/ros_python_requirements.txt


#
# Init rosdep
#

echo -e "\n\tSetting up rosdep\n"

sudo apt update

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

echo -e "\n\tCloning vision_opencv to ${HOME}/opencv_ws\n"

cd $HOME && mkdir opencv_ws && cd opencv_ws && \

git clone -b noetic https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \

cd src/vision_opencv  && sed -i 's/python37/python3/g' cv_bridge/CMakeLists.txt && cd ../.. && \

source /opt/ros/melodic/setup.bash && catkin init && \

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so --install --extend /opt/ros/melodic && \

catkin build cv_bridge && cp -r install/lib/python3/dist-packages/* ${HOME}/.local/lib/python3.6/site-packages/ && cd $HOME

if [[ -d ${HOME}/opencv_ws ]]; then
	rm -rf ${HOME}/opencv_ws
fi

#
#	Return to project
# 
cd ${PROJECT_ROOT}








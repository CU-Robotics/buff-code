
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

# change this to select ARM or x86_64
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1 # ubuntu:18.04  # 
FROM ${BASE_IMAGE}

ENV ROS_PKG=desktop
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV PROJECT_ROOT=/home/cu-robotics/buff-code

ENV DEBIAN_FRONTEND=noninteractive


#
# Create a new user
#
RUN useradd -ms /bin/bash cu-robotics

#
# Install Dependencies
#
RUN apt-get update && \
		apt-get install -y --no-install-recommends \
		vim=2:8.0.1453-1ubuntu1.7 \
		git=1:2.17.1-1ubuntu0.9 \
		curl=7.58.0-2ubuntu3.16 \
		wget=1.19.4-1ubuntu2.2 \
		cmake=3.10.2-1ubuntu2.18.04.2 \ 
		gnupg2=2.2.4-1ubuntu1.4 \
		jupyter=4.4.0-2 \
		python3-pip \
		lsb-release=9.20170808ubuntu1 \ 
		python3-opencv=3.2.0+dfsg-4ubuntu0.1 \ 
		build-essential=12.4ubuntu1 \
		ca-certificates=20210119~18.04.2 \
		python3-setuptools=39.0.1-2 \
		openssh-server \
		openssh-client \
		iputils-ping && \
		rm -rf /var/lib/apt/lists/*

# 
# Install ROS 
#   see ros install script for more info
#	Also installs our python reqs
 RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	apt-get update && \
	apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG} && \
	python3 -m pip install --upgrade pip==21.3.1 && \
	apt-get remove -y python3-pip && \
	python3 -m pip install \
		cython==0.29.26 \
		rosdep==0.21.0 \
		wstool==0.1.17 \
		pydrive==1.3.1 \
		rosinstall==0.7.8 \
		defusedxml==0.7.1 \
		catkin-tools==0.8.2 \
		rosinstall-generator==0.1.22 && \
	apt-get update && \
	rm -rf /var/lib/apt/lists/* && \
	cd /opt/ros/${ROS_DISTRO} && \
	rosdep init && \
	rosdep update && \
	rm -rf /var/lib/apt/lists/*

#
# setup catkin and cv_bridge
#
RUN apt update && \
	cd /home/cu-robotics && mkdir opencv_ws && cd opencv_ws && \
	git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \
	cd src/vision_opencv && git checkout 1.13.0 && cd ../.. && \
	bash /opt/ros/melodic/setup.sh && \
	catkin init && \
	catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so --install --extend /opt/ros/melodic && \
	catkin build cv_bridge && \
	cp -r install/lib/python3/dist-packages/* /usr/lib/python3/dist-packages/ && \
	cd /home/cu-robotics && rm -rf opencv_ws


#
# Install sublime text
#   see install script for more info
#   sublime in docker is slow when using ARM64
#   for reasons unknown
RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | apt-key add - && \
	echo "deb https://download.sublimetext.com/ apt/stable/" | tee /etc/apt/sources.list.d/sublime-text.list && \
	apt-get update && \
	apt-get install -y --no-install-recommends apt-transport-https \
		sublime-text


#
#	Teensy Loader Install
#   Excluded for now, requires device access which is not currently available


#
#	Clean APT/Cache
#
RUN apt-get update && \
	apt-get autoremove --purge -y && \
	apt-get clean -y

# 
# setup entrypoint
#
CMD ["bash"]
WORKDIR /home/cu-robotics

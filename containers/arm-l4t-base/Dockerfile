#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

#   Set as Ubuntu 18.04 AMD64 / l4t base is ARM64
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1 # ubuntu:18.04 # 
FROM ${BASE_IMAGE}

ENV ROS_PKG=desktop
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV PROJECT_ROOT=/home/cu-robotics/buff-code

ENV DEBIAN_FRONTEND=noninteractive


#
# Install Dependencies
#
RUN apt-get update && \
		apt-get install -y --no-install-recommends \
		vim \
		git \
		curl \
		wget \
		cmake \
		gnupg2 \
		jupyter \
		python3-pip \
		lsb-release \
		python3-opencv \
		build-essential \
		ca-certificates \
		python3-setuptools \
		&& rm -rf /var/lib/apt/lists/*

# 
# Install ROS 
#   see ros install script for more info
 RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	apt-get update && \
	apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG} && \
	python3 -m pip install --upgrade pip==21.3.1 && \
	apt remove -y python3-pip && \
	python3 -m pip install \
		rosdep==0.21.0 \
		wstool==0.1.17 \
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
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc
CMD ["bash"]
WORKDIR /home

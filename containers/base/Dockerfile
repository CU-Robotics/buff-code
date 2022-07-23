#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

ARG IMAGE_NAME

FROM ${IMAGE_NAME}

SHELL ["/bin/bash", "-c"]

ENV ROS_PKG=ros-base
ENV ROS_DISTRO=melodic
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV DOCKER_ROOT=/home/cu-robotics/buff-code
ENV PATH=/home/cu-robotics/.local/bin:${PATH}

#
# Create a new user
#

RUN useradd -ms /bin/bash cu-robotics


#
# Install Dependencies
#

ADD . /home/cu-robotics/buff-code


RUN cd /home/cu-robotics/buff-code && \
	apt update && \
	apt upgrade -y && \
	xargs apt install -y --no-install-recommends <buffpy/config/install/dependencies.txt
#		htop=2.1.0-3 \
#		tmux=2.6-3ubuntu0.2 \
#		wget=1.19.4-1ubuntu2.2 \
#		gnupg2=2.2.4-1ubuntu1.4 \
#		curl=7.58.0-2ubuntu3.18 \
#		git=1:2.17.1-1ubuntu0.11 \
#		python3-dev=3.6.7-1~18.04 \
#		vim=2:8.0.1453-1ubuntu1.8 \
#		build-essential=12.4ubuntu1 \
#		lsb-release=9.20170808ubuntu1 \
#		iputils-ping=3:20161105-1ubuntu3 \
#		ca-certificates=20210119~18.04.2 \
#		openssh-client=1:7.6p1-4ubuntu0.7 \
#		openssh-server=1:7.6p1-4ubuntu0.7

RUN curl https://bootstrap.pypa.io/pip/3.6/get-pip.py -o get-pip.py && \
	python3 get-pip.py && \
	rm get-pip.py && \
	cd /home/cu-robotics/buff-code && \
	pip3 install -r buffpy/config/install/python3_requirements.txt


#
#	Clean apt
#

RUN apt update && \
	apt clean && \
	apt autoremove -y


# 
# Install ROS 
#   see ros install script for more info
#


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	apt update && \
	apt install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG} python-rosdep \
	ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-common-plugins  ros-${ROS_DISTRO}-rqt-robot-plugins

RUN cd /home/cu-robotics/buff-code && pip3 install -r buffpy/config/install/ros_python_requirements.txt
#RUN pip3 install \
#		pyyaml==6.0 \
#		torch==1.9.0 \
#		tqdm==4.62.3 \
#		numpy==1.19.5 \
#		pandas==1.1.5 \
#		pillow==8.4.0 \
#		pyserial==3.5 \
#		pydrive==1.3.1 \
#		seaborn==0.11.2 \
#		cython==0.29.26 \
#		protobuf==3.12.4 \
#		requests==2.27.1 \
#		depthai==2.15.2.0 \
#		platformio==6.0.1 \
#		torchvision==0.10.0 \
#		opencv-python==4.5.5.64 \
#		rospkg==1.4.0 \
#		wstool==0.1.17 \
#		rosinstall==0.7.8 \
#		defusedxml==0.7.1 \
#		catkin-tools==0.8.2 \
#		rosinstall-generator==0.1.22



RUN cd ${ROS_ROOT} && \
	rosdep init && \
	rosdep update


#
#	Clean apt
#

RUN apt update && \
	apt clean && \
	apt autoremove -y


#
# 	Setup catkin and cv_bridge
# so blocky eww

RUN cd /home/cu-robotics && mkdir opencv_ws && cd opencv_ws && \
	git clone -b noetic https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \
	cd src/vision_opencv && sed -i 's/python37/python3/g' cv_bridge/CMakeLists.txt && cd ../.. && \
	bash /opt/ros/melodic/setup.sh && catkin init && \
	catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so --install --extend /opt/ros/melodic && \
	catkin build cv_bridge && \
	rm -rf /opt/ros/melodic/lib/python2.7/dist-packages/cv_bridge* && \
	cp -r /home/cu-robotics/opencv_ws/install/lib/python3/dist-packages/* /opt/ros/melodic/lib/python2.7/dist-packages/ && \
	rm -rf /home/cu-robotics/opencv_ws


RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -sSf | sh -s -- -y
#
#	Clean apt
#

RUN apt update && \
	apt clean && \
	apt autoremove -y


# 
# setup entrypoint
#

CMD ["bash"]
WORKDIR /home/cu-robotics/buff-code

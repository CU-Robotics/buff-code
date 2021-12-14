# Base image to build in
FROM ubuntu:18.04


# 		Set project root var
ARG PROJECT_ROOT=/home/cu-robotics/buff-code


# 		Author
MAINTAINER Mitchell D Scott <misc4432@colorado.edu>


# 		Use bash to install
CMD ["bash"]


# 		Disable interactve front-end
ARG DEBIAN_FRONTEND=non-interactive


# 		Create a new user
RUN useradd -ms /bin/bash cu-robotics


# 		Update APT
RUN apt-get update


# 		Add our configs for setup
ADD config ${PROJECT_ROOT}/config


# 		Run apt install for general dependencies
RUN apt-get update && \
    xargs apt-get install -y --no-install-recommends <${PROJECT_ROOT}/config/install/dependencies.txt && \
	rm -rf /var/lib/apt/lists/*


#		ROS Install

# Setup Ros keys
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS deps
RUN apt-get update && \
    xargs apt-get install -y --no-install-recommends <${PROJECT_ROOT}/config/install/ros_python_deps.txt && \
	rm -rf /var/lib/apt/lists/*

# Init rosdep
RUN apt-get update && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

# Switch user to avoid pip/rosdep error
USER cu-robotics
RUN rosdep update


#		Python Requirements Install

RUN pip3 install --upgrade -U pip && \
	pip3 install -U -r ${PROJECT_ROOT}/config/install/python3_requirements.txt


# Switch back to root to use apt and curl
USER root

#		Teensy Loader Install



#		Clean APT/Cache
RUN apt-get update && \
  apt-get autoremove --purge -y && \
  apt-get clean -y


# 		Set Working Dir and Run Bash
WORKDIR /home/cu-robotics/buff-code
CMD ["bash"]
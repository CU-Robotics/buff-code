#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

ARG IMAGE_NAME

FROM ${IMAGE_NAME}

ENV ROS_PKG=ros-base
ENV ROS_DISTRO=noetic
ENV UBUNTU_VERSION=20.04
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV PROJECT_ROOT=/home/cu-robotics/buff-code
ENV PATH=/home/cu-robotics/.local/bin:${PATH}

#
# Create a new user
#

RUN useradd -ms /bin/bash cu-robotics

#
# Install Dependencies
#

# USER cu-robotics
COPY buffpy/ /home/cu-robotics/buff-code/buffpy

#
# Install Buffpy
#

# USER root
RUN apt update && \
	xargs apt install -y --no-install-recommends <${PROJECT_ROOT}/buffpy/data/install/dependencies.txt && \
	curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
	python3 get-pip.py && rm -rf get-pip.py && \
	pip install -r ${PROJECT_ROOT}/buffpy/data/install/python3_requirements.txt

#
# Install ROS Noetic
#

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	apt update && \
	apt install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG} ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-common-plugins ros-${ROS_DISTRO}-catkin ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rqt-robot-plugins

#
# Install Rosdep
#	(idk why, thats just what people do)

RUN apt update && apt install -y python3-rosdep && cd /opt/ros/${ROS_DISTRO} && rosdep init && rosdep update

#
# Install Rust
#

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -sSf | sh -s -- -y

# 
# Clean apt
#

RUN apt update && \
 	apt clean && \
 	apt autoremove -y && \
 	apt upgrade -y

# 
# setup entrypoint
#

RUN echo "source /home/cu-robotics/buff-code/buffpy/buff.bash" >> ~/.bashrc

CMD ["bash"]
WORKDIR /home/cu-robotics/buff-code

#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

ARG BASE_IMAGE=ubuntu:18.04 # nvcr.io/nvidia/l4t-base:r32.6.1
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
# Add your code
#
ADD . /home/cu-robotics/buff-code


#
# Install base dependencies
#
RUN apt-get update && \
    xargs apt-get install -y --no-install-recommends <${PROJECT_ROOT}/config/install/dependencies.txt && \
  rm -rf /var/lib/apt/lists/*

#
# Install python requirements
#
USER cu-robotics

RUN python3.6 -m pip install --upgrade pip==21.3.1 && \
  python3.6 -m pip install -r ${PROJECT_ROOT}/config/install/python3_requirements.txt

USER root

# 
# Install ROS 
#   see ros install script for more info
 RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
  apt-get update && \
  apt-get install -y ros-${ROS_DISTRO}-${ROS_PKG} && \
  xargs apt-get install -y <${PROJECT_ROOT}/config/install/ros_dependencies.txt && \
  rm -rf /var/lib/apt/lists/* && \
  cd /opt/ros/${ROS_DISTRO} && \
  rosdep init && \
  rosdep update && \
  rm -rf /var/lib/apt/lists/*


#
# Install sublime text
#   see install script for more info
RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | apt-key add - && \
  echo "deb https://download.sublimetext.com/ apt/stable/" | tee /etc/apt/sources.list.d/sublime-text.list && \
  apt-get update && \
  apt-get install -y apt-transport-https && \
  apt-get install -y sublime-text


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
# Set up the environment
# not sure how to utilize the project_root variable
RUN echo "source /home/cu-robotics/buff-code/buff.bash" >> /root/.bashrc
CMD ["bash"]
ENTRYPOINT ["/home/cu-robotics/buff-code/scripts/buff_entrypoint.sh"]
WORKDIR /home/cu-robotics/buff-code

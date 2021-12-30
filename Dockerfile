#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
FROM ${BASE_IMAGE}

ARG ROS_PKG=desktop
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
# Install ROS packages
#
RUN .${PROJECT_ROOT}/scripts/install_ros_melodic.bash ${ROS_DISTRO} ${ROS_PKG}


#
# Install sublime text
#
RUN .${PROJECT_ROOT}/scripts/install_sublime.bash


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
#
RUN echo "source ${PROJECT_ROOT}/buff.bash" >> /root/.bashrc
CMD ["bash"]
ENTRYPOINT ["${PROJECT_ROOT}/scripts/buff_entrypoint.sh"]
WORKDIR ${PROJECT_ROOT}

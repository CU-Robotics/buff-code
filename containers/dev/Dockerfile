#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

#   Select base image here (ARM64 or AMD64)

ARG IMAGE_NAME
FROM ${IMAGE_NAME}

ENV PROJECT_ROOT=/home/cu-robotics/buff-code
ENV DEBIAN_FRONTEND=noninteractive

#
# Add your code
#

ADD . /home/cu-robotics/buff-code


#
# Build Buff-Code
#

RUN /home/cu-robotics/buff-code/buffpy/buff.bash && \
	python3 /home/cu-robotics/buff-code/buffpy/scripts/buffpy --build all


#
# Set up the environment
# 


CMD ["bash"]
ENTRYPOINT ["/home/cu-robotics/buff-code/buffpy/scripts/buff_entrypoint.sh"]
WORKDIR /home/cu-robotics/buff-code


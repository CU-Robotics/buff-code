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

COPY src /home/cu-robotics/buff-code/
COPY buffpy /home/cu-robotics/buff-code/


#
# Build Buff-Code
#

# RUN /home/cu-robotics/buff-code/buffpy/buff.bash && \
#	python3 /home/cu-robotics/buff-code/buffpy/scripts/buffpy --build deploy && \
#	rm -rf /home/cu-robotics/buff-code/src


#
# Set up the environment
# 


CMD ["/bin/bash"]
# ENTRYPOINT ["/home/cu-robotics/buff-code/buffpy/scripts/buff_entrypoint.sh"]
WORKDIR /home/cu-robotics/buff-code


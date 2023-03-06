#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

#   Select base image here (ARM64 or AMD64)

ARG IMAGE_NAME
FROM ${IMAGE_NAME}

ENV PATH="/root/.cargo/bin:${PATH}"
ENV PROJECT_ROOT=/home/cu-robotics/buff-code
ENV DEBIAN_FRONTEND=noninteractive

#
# Add your code
#
USER cu-robotics
COPY src /home/cu-robotics/buff-code/src
COPY buffpy /home/cu-robotics/buff-code/buffpy


#
# Build Buff-Code
#

USER root
#RUN /home/cu-robotics/buff-code/buffpy/scripts/buffpy -b rust-debug && \
#	rm -rf /home/cu-robotics/buff-code/src


#
# Set up the environment
# 

CMD ["/bin/bash"]
ENTRYPOINT ["/home/cu-robotics/buff-code/buffpy/scripts/buff_entrypoint.sh"]
WORKDIR /home/cu-robotics/buff-code


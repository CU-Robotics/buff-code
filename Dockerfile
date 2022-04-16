#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#

#   Select base image here (ARM64 or AMD64)
ARG BASE_IMAGE=mdsdev0/buffbox:x86_64-base # mdsdev0/buffbox:aarch64-base # 
FROM ${BASE_IMAGE}

ENV PROJECT_ROOT=/home/cu-robotics/buff-code

#
# Add your code
#
ADD . /home/cu-robotics/buff-code


#
#	Clean APT/Cache
#
RUN apt-get update && \
	apt-get autoremove --purge -y && \
	apt-get clean -y

#
# Set up the environment
# not sure how to utilize the project_root variable
# hard coding = bad, doesn't really matter tho
RUN echo "source /home/cu-robotics/buff-code/buffpy/buff.bash" >> /root/.bashrc
CMD ["bash"]
ENTRYPOINT ["/home/cu-robotics/buff-code/buffpy/scripts/buff_entrypoint.sh"]
WORKDIR /home/cu-robotics/buff-code

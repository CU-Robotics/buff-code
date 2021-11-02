#! /usr/bin/env bash

#  run this script with 'source scripts/install.bash'

PROJECT_ROOT=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tapt updating...\n"

#	update the apt package manager
sudo apt update

echo -e "\n\tInstalling Dependencies...\n"

#	Using apt and pip install all the dependencies for the project
xargs sudo apt install <${PROJECT_ROOT}/config/install/dependencies.txt -y

echo -e "\n\tUpgrading pip3\n"
python3.6 -m pip install --upgrade pip

python3.6 -m pip install -r ${PROJECT_ROOT}/config/install/python3_requirements.txt

# echo -e "\n\tSetting python alternatives\n"
# sudo update-alternaitves --set python3 /usr/bin/python3.8 1

echo -e "\n\tSetting up ROS melodic\n"

# ROS installation
# add repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup ROS keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

# Install ROS
if [[ "${HOSTNAME}" == "edge"* ]]; then
	sudo apt install ros-melodic-desktop -y
else
	sudo apt install ros-melodic-desktop-full -y
fi

# Source the ros/setup.bash
source /opt/ros/melodic/setup.bash

# Install ROS dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools python3-catkin-pkg python3-rospkg -y

echo -e "\n\tFinishing ROS setup...\n"

# Init rosdep
sudo rosdep init
rosdep update

# DEPRECATED
#	Install arduino-cli
#	Switching to teensy loader
#curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh
#	Install SamD boards (subject to change, need to know the teensy's fqbn)
# arduino-cli core install arduino:samd


if [[ "${HOSTNAME}" != "edge"* ]]; then
	echo -e "\n\tInstalling Teensy loader...\n"

	#	Pull teensy files from pjrc.com
	# teensy binary and objects
	curl https://www.pjrc.com/teensy/teensy_linux64.tar.gz -O
	# teensy rules file
	curl https://www.pjrc.com/teensy/00-teensy.rules -O

	# mv rules into rules.d and set the proper file permissions
	sudo mv 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
	sudo chmod 0644 /etc/udev/rules.d/00-teensy.rules

	# extract the tar to buffpy/bin
	tar -xvsf teensy_linux64.tar.gz -C ${PROJECT_ROOT}/buffpy/bin
	# remove unecessary tar.gz 
	rm teensy_linux64.tar.gz
fi

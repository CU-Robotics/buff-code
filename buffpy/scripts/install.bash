#! /bin/bash

#  run this script with 'source scripts/install.bash'

ROS_PKG=desktop
ROS_DISTRO=melodic
export PROJECT_ROOT=$PWD

echo -e "Running Install from ${PWD}"

if [[ $PROJECT_ROOT != *"/buff-code" ]]; then
	echo -e "Run this script from the project root"
	exit
fi 

echo -e "\n\tapt updating...\n"
#	update the apt package manager
sudo apt update


echo -e "\n\tapt Installing from ${PROJECT_ROOT}/buffpy/config/install/dependencies.txt\n"
#	Using apt and pip install all the dependencies for the project
xargs sudo apt install -y <${PROJECT_ROOT}/buffpy/config/install/dependencies.txt
sudo rm -rf /var/lib/apt/lists/*

echo -e "\n\tUpgrading pip3 to 21.3.1\n"
# Not sure whats happening apt only installs pip==9.0.1
sudo apt purge -y python3-pip
# upgrade pip before installing dependencies
python3 -m pip install --upgrade pip==21.3.1

echo -e "\n\tpip3 Installing from ${PROJECT_ROOT}/buffpy/config/install/python3_requirements.txt\n"
python3 -m pip install -r ${PROJECT_ROOT}/buffpy/config/install/python3_requirements.txt

# If no ROS, install it
if [[ ! -d /opt/ros/${ROS_DISTRO} ]]; then
	source "${PROJECT_ROOT}/buffpy/scripts/install_ros.bash" $ROS_DISTRO $ROS_PKG
fi

# Also install Sublime Text-editor
## source "${PROJECT_ROOT}/buffpy/scripts/install_sublime.bash"

if [[ "${HOSTNAME}" != "edge"* ]]; then
	# echo -e "\n\tInstalling Teensy loader...\n"

	# #	Pull teensy files from pjrc.com
	# # teensy binary and objects
	# curl https://www.pjrc.com/teensy/teensy_linux64.tar.gz -O
	# # teensy rules file
	# curl https://www.pjrc.com/teensy/00-teensy.rules -O

	# # mv rules into rules.d and set the proper file permissions
	# sudo mv 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
	# sudo chmod 0644 /etc/udev/rules.d/00-teensy.rules

	# # extract the tar to buffpy/bin
	# tar -xvsf teensy_linux64.tar.gz -C ${PROJECT_ROOT}/buffpy/bin
	# # remove unecessary tar.gz 
	# rm teensy_linux64.tar.gz

else
	sudo cp ${PROJECT_ROOT}/buffpy/scripts/buffbot.service /etc/systemd/system

fi

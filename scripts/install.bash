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
pip3 install -r ${PROJECT_ROOT}/config/install/python3_requirements.txt

# DEPRECATED
#	Install arduino-cli
#	Switching to teensy loader
#curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh
#	Install SamD boards (subject to change, need to know the teensy's fqbn)
# arduino-cli core install arduino:samd

#	Pull teensy files
# teensy binary and objects
curl https://www.pjrc.com/teensy/teensy_linux64.tar.gz -O
# teensy rules file
curl https://www.pjrc.com/teensy/00-teensy.rules -O

# mv rules into rules.d and set the proper file permissions
sudo mv 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
sudo chmod 0644 /etc/udev/rules.d/00-teensy.rules

# extract the tar to buffpy/bin
tar -xvsf teensy_linux.tar.gz -C ${PROJECT_ROOT}/buffpy/bin
# remove unecessary tar.gz 
rm teensy_linux.tar.gz

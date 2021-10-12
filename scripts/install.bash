#! /usr/bin/env bash

#  run this script with 'source scripts/install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tapt updating...\n"

#	update the apt package manager
sudo apt update

echo -e "\n\tInstalling Dependencies...\n"

#	Using apt and pip install all the dependencies for the project
xargs sudo apt install <${PROJECT_ROOT}/config/install/dependencies.txt -y
pip3 install -r ${PROJECT_ROOT}/config/install/python3_requirements.txt

#	Install arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh

#	Install SamD boards (subject to change, need to know the teensy's fqbn)
arduino-cli core install arduino:samd
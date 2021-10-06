#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tapt updating...\n"

sudo apt update

echo -e "\n\tInstalling Dependencies...\n"

xargs sudo apt install <${PROJECT_ROOT}/config/dependencies.txt -y
pip3 install -r ${PROJECT_ROOT}/config/python3_requirements.txt

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh

arduino-cli core install arduino:samd
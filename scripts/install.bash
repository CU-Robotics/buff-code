#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tapt updating...\n"

sudo apt update

echo -e "\n\tInstalling Dependencies...\n"

xargs sudo apt install <${PROJECT_ROOT}/config/dependencies.txt -y
pip3 install -r ${PROJECT_ROOT}/config/python3_requirements.txt

export PATH="${PROJECT_ROOT}/buffpy:${PROJECT_ROOT}/buffpy/arduino-cli_0.19.2_Linux_64bit:${PATH}"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/bin:${PYTHONPATH}"

arduino-cli core install arduino:samd
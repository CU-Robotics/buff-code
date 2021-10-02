#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tbrew updating...\n"

brew update

echo -e "\n\tInstalling Dependencies...\n"

brew install python3
python3 -m pip install --upgrade pip
brew install arduino-cli
pip install testresources

pip3 install -r ${PROJECT_ROOT}/config/python3_requirements.txt

export PATH="${PROJECT_ROOT}/buffpy:${PROJECT_ROOT}/buffpy/arduino-cli_0.19.2_Linux_64bit:${PATH}"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/bin:${PYTHONPATH}"
arduino-cli core install arduino:samd
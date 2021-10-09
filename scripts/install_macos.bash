#! /usr/bin/env bash

#  run this script with 'source install.bash'

DIR=$PWD

echo -e "Running Install from ${PWD}"
echo -e "\n\tbrew updating...\n"

brew update
brew upgrade

echo -e "\n\tInstalling Dependencies...\n"

xargs brew install <${PWD}/config/install/dependencies.txt 
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=${DIR}/buffpy/bin sh

pip3 install -r ${PWD}/config/install/python3_requirements.txt

export PATH="${PROJECT_ROOT}/buffpy:${PROJECT_ROOT}/buffpy/arduino-cli"
export PYTHONPATH="${PROJECT_ROOT}/buffpy/bin:${PYTHONPATH}"